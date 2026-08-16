// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, The OpenROAD Authors

// Unit tests for the per-layer via-rect generation in Connect -- the stage
// that decides which candidate shapes each layer of a via stack is offered
// before the via generators are asked to fit one.
//
// The focus is the option surface of add_pdn_connect that shapes those rects
// (-min_width_layers in particular), plus the intermediate-layer discovery
// and the complex-stacked-via path they interact with.

#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "PdnTest.h"
#include "connect.h"
#include "domain.h"
#include "gmock/gmock.h"
#include "grid.h"
#include "gtest/gtest.h"
#include "odb/PtrSetMap.h"
#include "odb/db.h"
#include "odb/geom.h"
#include "pdn/PdnGen.hh"
#include "spdlog/sinks/ostream_sink.h"
// Connect's via map holds unique_ptr<DbGenerateStackedVia>, which in turn
// holds a unique_ptr<TechLayer>; both need to be complete here.
#include "techlayer.h"
#include "utl/Logger.h"

namespace pdn {
namespace {

// Connect keeps the rect-generation helpers at protected scope for tests;
// re-publish them here.
class TestableConnect : public Connect
{
 public:
  using Connect::Connect;

  using Connect::ViaLayerRects;

  using Connect::generateComplexStackedViaRects;
  using Connect::generateMinEnclosureViaRects;
  using Connect::generateViaRects;
  using Connect::getMinWidth;
  using Connect::isComplexStackedVia;
};

// Fixture providing the minimum scaffolding a Connect needs: a PdnGen, a
// voltage domain and a grid (Connect reaches the logger and block through
// its grid), plus a routing stack to connect across.
class TestConnect : public PdnTest
{
 protected:
  void SetUp() override
  {
    PdnTest::SetUp();

    power_ = odb::dbNet::create(block(), "VDD");
    power_->setSigType(odb::dbSigType::POWER);
    power_->setSpecial();
    ground_ = odb::dbNet::create(block(), "VSS");
    ground_->setSigType(odb::dbSigType::GROUND);
    ground_->setSpecial();
    block()->setDieArea(odb::Rect(0, 0, 100000, 100000));
    block()->setCoreArea(odb::Rect(1000, 1000, 99000, 99000));

    pdngen_ = std::make_unique<PdnGen>(db_.get(), getLogger());
    domain_ = std::make_unique<VoltageDomain>(pdngen_.get(),
                                              block(),
                                              power_,
                                              ground_,
                                              std::vector<odb::dbNet*>{},
                                              getLogger());
    auto grid = std::make_unique<TestGrid>(domain_.get(),
                                           "grid",
                                           /*starts_with_power=*/true,
                                           std::vector<odb::dbTechLayer*>{});
    grid_ = grid.get();
    domain_->addGrid(std::move(grid));

    // M1(H) V1 M2(V) V2 M3(H) V3 M4(V), every routing layer 100 wide.
    stack_ = makeRoutingStack(/*levels=*/4, kMinWidth);
  }

  odb::dbTechLayer* m(int level) const { return stack_.routing[level - 1]; }

  // The rects a full M1 -> M4 stack offers each layer, after min-enclosure
  // filtering. Index 0 is M1, 1 is M2, 2 is M3, 3 is M4.
  static std::vector<TestableConnect::ViaLayerRects> stackRects(
      const TestableConnect& connect,
      const odb::Rect& lower,
      const odb::Rect& upper)
  {
    auto rects = connect.generateViaRects(lower, upper);
    connect.generateMinEnclosureViaRects(rects);
    return rects;
  }

  // Connect::report() writes through the logger, so tap it with a temporary
  // sink and hand back everything it emitted.
  std::string captureReport(const Connect& connect)
  {
    std::ostringstream stream;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(stream);
    sink->set_pattern("%v");
    getLogger()->addSink(sink);
    connect.report();
    getLogger()->removeSink(sink);
    return stream.str();
  }

  static constexpr int kMinWidth = 100;

  std::unique_ptr<PdnGen> pdngen_;
  std::unique_ptr<VoltageDomain> domain_;
  Grid* grid_ = nullptr;
  odb::dbNet* power_ = nullptr;
  odb::dbNet* ground_ = nullptr;
  RoutingStack stack_;
};

// -------- Layer discovery --------

// An M1 -> M4 connect picks up M2 and M3 as intermediate routing layers, and
// the cut layers between them as (non-routing) intermediate layers.
TEST_F(TestConnect, IntermediateRoutingLayersAreDiscoveredInOrder)
{
  const TestableConnect connect(grid_, m(1), m(4));

  EXPECT_EQ(connect.getLowerLayer(), m(1));
  EXPECT_EQ(connect.getUpperLayer(), m(4));
  EXPECT_TRUE(connect.isMultiLayerVia());
  EXPECT_THAT(connect.getIntermediteRoutingLayers(),
              ::testing::ElementsAre(m(2), m(3)));
  EXPECT_THAT(connect.getIntermediteLayers(),
              ::testing::ElementsAre(
                  stack_.cuts[0], m(2), stack_.cuts[1], m(3), stack_.cuts[2]));
}

// Adjacent layers give a single-layer via with no intermediate routing layers.
TEST_F(TestConnect, AdjacentLayersFormASingleLayerVia)
{
  const TestableConnect connect(grid_, m(1), m(2));

  EXPECT_TRUE(connect.isSingleLayerVia());
  EXPECT_TRUE(connect.getIntermediteRoutingLayers().empty());
}

// The constructor normalizes the layer order, so the caller may pass the
// endpoints either way round.
TEST_F(TestConnect, LayerOrderIsNormalized)
{
  const TestableConnect connect(grid_, m(4), m(1));

  EXPECT_EQ(connect.getLowerLayer(), m(1));
  EXPECT_EQ(connect.getUpperLayer(), m(4));
}

// -------- generateViaRects --------

// The plain path offers the lower shape, the upper shape, and the overlap of
// the two to every intermediate layer.
TEST_F(TestConnect, GenerateViaRectsOffersTheOverlapToEachIntermediateLayer)
{
  const TestableConnect connect(grid_, m(1), m(4));

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const odb::Rect overlap(300, 0, 700, 400);

  const auto rects = connect.generateViaRects(lower, upper);
  ASSERT_EQ(rects.size(), 4u);  // M1, M2, M3, M4
  EXPECT_THAT(rects[0], ::testing::ElementsAre(lower));
  EXPECT_THAT(rects[1], ::testing::ElementsAre(overlap));
  EXPECT_THAT(rects[2], ::testing::ElementsAre(overlap));
  EXPECT_THAT(rects[3], ::testing::ElementsAre(upper));
}

// -------- generateMinEnclosureViaRects: the default (no -min_width_layers) --

// By default each intermediate layer is offered both the full overlap and a
// min-width variant of it, so the generator can pick whichever fits. The
// min-width variant is clamped along the layer's *width* direction only.
TEST_F(TestConnect, MinEnclosureRectsAddAMinWidthVariantPerIntermediateLayer)
{
  const TestableConnect connect(grid_, m(1), m(4));

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const odb::Rect overlap(300, 0, 700, 400);
  const auto rects = stackRects(connect, lower, upper);

  // M2 is VERTICAL: x (its width axis) is clamped to 100 about the center at
  // x = 500; y is untouched.
  EXPECT_THAT(
      rects[1],
      ::testing::UnorderedElementsAre(overlap, odb::Rect(450, 0, 550, 400)));
  // M3 is HORIZONTAL: y is clamped to 100 about the center at y = 200.
  EXPECT_THAT(
      rects[2],
      ::testing::UnorderedElementsAre(overlap, odb::Rect(300, 150, 700, 250)));
  // The endpoint layers are never touched.
  EXPECT_THAT(rects[0], ::testing::ElementsAre(lower));
  EXPECT_THAT(rects[3], ::testing::ElementsAre(upper));
}

// An overlap already at min width in the layer's width direction produces a
// min-width variant identical to the overlap, so nothing is added.
TEST_F(TestConnect, MinEnclosureRectsAddNothingWhenAlreadyAtMinWidth)
{
  const TestableConnect connect(grid_, m(1), m(4));

  const odb::Rect lower(0, 0, 1000, kMinWidth);
  const odb::Rect upper(450, 0, 450 + kMinWidth, 1000);
  const odb::Rect overlap(450, 0, 550, 100);
  const auto rects = stackRects(connect, lower, upper);

  EXPECT_THAT(rects[1], ::testing::ElementsAre(overlap));
  EXPECT_THAT(rects[2], ::testing::ElementsAre(overlap));
}

// The variant is forced to *exactly* min width, so a sub-min-width overlap is
// grown rather than left alone.
TEST_F(TestConnect, MinEnclosureRectsGrowAnUndersizedOverlap)
{
  const TestableConnect connect(grid_, m(1), m(4));

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(480, 0, 520, 1000);  // 40 wide, under the 100 min
  const odb::Rect overlap(480, 0, 520, 400);
  const auto rects = stackRects(connect, lower, upper);

  EXPECT_THAT(
      rects[1],
      ::testing::UnorderedElementsAre(overlap, odb::Rect(450, 0, 550, 400)));
}

// A single-layer via has no intermediate layers, so min-enclosure filtering
// is a no-op on both endpoints.
TEST_F(TestConnect, MinEnclosureRectsLeaveASingleLayerViaAlone)
{
  const TestableConnect connect(grid_, m(1), m(2));

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const auto rects = stackRects(connect, lower, upper);

  ASSERT_EQ(rects.size(), 2u);
  EXPECT_THAT(rects[0], ::testing::ElementsAre(lower));
  EXPECT_THAT(rects[1], ::testing::ElementsAre(upper));
}

// -------- -min_width_layers --------

// A layer named in -min_width_layers is offered *only* the min-width rect:
// the full-overlap candidate is dropped so the via array cannot fill the
// stripe with extra rows across the layer's width. Layers not named keep both.
TEST_F(TestConnect, MinWidthLayerDropsTheFullOverlapCandidate)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({m(2)});

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const odb::Rect overlap(300, 0, 700, 400);
  const auto rects = stackRects(connect, lower, upper);

  EXPECT_THAT(rects[1], ::testing::ElementsAre(odb::Rect(450, 0, 550, 400)));
  EXPECT_THAT(
      rects[2],
      ::testing::UnorderedElementsAre(overlap, odb::Rect(300, 150, 700, 250)));
}

// The clamp follows each layer's own routing direction: the VERTICAL M2 keeps
// its full y extent while the HORIZONTAL M3 keeps its full x extent. Neither
// is squared off.
TEST_F(TestConnect, MinWidthLayersClampOnlyTheirOwnWidthDirection)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({m(2), m(3)});

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const auto rects = stackRects(connect, lower, upper);

  ASSERT_EQ(rects[1].size(), 1u);
  ASSERT_EQ(rects[2].size(), 1u);
  const odb::Rect& m2_rect = *rects[1].begin();
  const odb::Rect& m3_rect = *rects[2].begin();

  EXPECT_EQ(m2_rect, odb::Rect(450, 0, 550, 400));
  EXPECT_EQ(m2_rect.dx(), kMinWidth);
  EXPECT_EQ(m2_rect.dy(), 400);  // preferred direction untouched

  EXPECT_EQ(m3_rect, odb::Rect(300, 150, 700, 250));
  EXPECT_EQ(m3_rect.dy(), kMinWidth);
  EXPECT_EQ(m3_rect.dx(), 400);  // preferred direction untouched
}

// Naming a layer whose overlap is already min width changes nothing -- the
// candidate set was already a single rect.
TEST_F(TestConnect, MinWidthLayerIsANoOpWhenAlreadyAtMinWidth)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({m(2), m(3)});

  const odb::Rect lower(0, 0, 1000, kMinWidth);
  const odb::Rect upper(450, 0, 450 + kMinWidth, 1000);
  const odb::Rect overlap(450, 0, 550, 100);
  const auto rects = stackRects(connect, lower, upper);

  EXPECT_THAT(rects[1], ::testing::ElementsAre(overlap));
  EXPECT_THAT(rects[2], ::testing::ElementsAre(overlap));
}

// A sub-min-width overlap is still grown to min width; -min_width_layers only
// removes the full-overlap alternative, it never shrinks below min width.
TEST_F(TestConnect, MinWidthLayerStillGrowsAnUndersizedOverlap)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({m(2)});

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(480, 0, 520, 1000);
  const auto rects = stackRects(connect, lower, upper);

  EXPECT_THAT(rects[1], ::testing::ElementsAre(odb::Rect(450, 0, 550, 400)));
}

// Naming the stack's own endpoints (or any layer that is not an intermediate
// routing layer) is inert -- those layers land on real straps and are never
// min-width filtered.
TEST_F(TestConnect, MinWidthLayersIgnoreNonIntermediateLayers)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({m(1), m(4), stack_.cuts[0]});

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const odb::Rect overlap(300, 0, 700, 400);
  const auto rects = stackRects(connect, lower, upper);

  EXPECT_THAT(rects[0], ::testing::ElementsAre(lower));
  EXPECT_THAT(rects[3], ::testing::ElementsAre(upper));
  // The intermediate layers still get both candidates.
  EXPECT_EQ(rects[1].size(), 2u);
  EXPECT_THAT(
      rects[2],
      ::testing::UnorderedElementsAre(overlap, odb::Rect(300, 150, 700, 250)));
}

// setMinWidthLayers accumulates rather than replacing, matching setOnGrid.
TEST_F(TestConnect, SetMinWidthLayersAccumulatesAcrossCalls)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({m(2)});
  connect.setMinWidthLayers({m(3)});

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const auto rects = stackRects(connect, lower, upper);

  EXPECT_THAT(rects[1], ::testing::ElementsAre(odb::Rect(450, 0, 550, 400)));
  EXPECT_THAT(rects[2], ::testing::ElementsAre(odb::Rect(300, 150, 700, 250)));
}

// An empty list leaves the default behavior in place.
TEST_F(TestConnect, SetMinWidthLayersWithNoLayersIsANoOp)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({});

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const auto rects = stackRects(connect, lower, upper);

  EXPECT_EQ(rects[1].size(), 2u);
  EXPECT_EQ(rects[2].size(), 2u);
}

// The min-width rect is exactly min width even when that is odd, with the
// integer-division rounding landing on the low side of the center.
TEST_F(TestConnect, MinWidthLayerHandlesAnOddMinWidth)
{
  m(2)->setWidth(101);
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({m(2)});

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(300, 0, 700, 1000);
  const auto rects = stackRects(connect, lower, upper);

  ASSERT_EQ(rects[1].size(), 1u);
  const odb::Rect& m2_rect = *rects[1].begin();
  EXPECT_EQ(m2_rect.dx(), 101);
  EXPECT_EQ(m2_rect, odb::Rect(450, 0, 551, 400));
}

// -------- Interaction with the complex-stacked-via path --------

// An overlap narrower than an intermediate layer's min width makes the stack
// "complex": the intermediate rect is pre-grown to fit a via before the
// min-enclosure pass runs.
TEST_F(TestConnect, ComplexStackedViaIsDetectedFromAnUndersizedOverlap)
{
  const TestableConnect connect(grid_, m(1), m(4));

  // 40-wide overlap, under M2/M3's 100 min width.
  EXPECT_TRUE(connect.isComplexStackedVia(odb::Rect(0, 0, 1000, 400),
                                          odb::Rect(480, 0, 520, 1000)));
  // A comfortably sized overlap is not complex.
  EXPECT_FALSE(connect.isComplexStackedVia(odb::Rect(0, 0, 1000, 400),
                                           odb::Rect(300, 0, 700, 1000)));
}

// A single-layer via is never complex -- it has no intermediate layer that
// could be too narrow.
TEST_F(TestConnect, SingleLayerViaIsNeverComplex)
{
  const TestableConnect connect(grid_, m(1), m(2));

  EXPECT_FALSE(connect.isComplexStackedVia(odb::Rect(0, 0, 1000, 400),
                                           odb::Rect(480, 0, 520, 1000)));
}

// With no enclosure rules in the tech, the width a complex stack targets is
// just the layer min width.
TEST_F(TestConnect, GetMinWidthIsTheLayerMinWidthWithoutEnclosureRules)
{
  const TestableConnect connect(grid_, m(1), m(4));

  EXPECT_EQ(connect.getMinWidth(m(2)), kMinWidth);
}

// A cut enclosure rule on a neighboring cut layer widens the target so a via
// is guaranteed to fit: min width + 2 * the worst-case overhang.
TEST_F(TestConnect, GetMinWidthAddsTheWorstCaseCutEnclosure)
{
  addCutEnclosureRule(stack_.cuts[1],
                      /*first_overhang=*/30,
                      /*second_overhang=*/45);
  const TestableConnect connect(grid_, m(1), m(4));

  EXPECT_EQ(connect.getMinWidth(m(2)), kMinWidth + 2 * 45);
}

// The complex path grows the intermediate rect to min width, and the
// min-enclosure pass then clamps that grown rect back in the width direction.
// With -min_width_layers only the clamped rect survives.
TEST_F(TestConnect, MinWidthLayersComposeWithTheComplexStackedPath)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setMinWidthLayers({m(3)});

  const odb::Rect lower(0, 0, 1000, 400);
  const odb::Rect upper(480, 0, 520, 1000);  // 40-wide overlap -> complex
  ASSERT_TRUE(connect.isComplexStackedVia(lower, upper));

  auto rects = connect.generateComplexStackedViaRects(lower, upper);
  // Both intermediate layers were grown to 100 in x by the complex path.
  EXPECT_THAT(rects[1], ::testing::ElementsAre(odb::Rect(450, 0, 550, 400)));
  EXPECT_THAT(rects[2], ::testing::ElementsAre(odb::Rect(450, 0, 550, 400)));

  connect.generateMinEnclosureViaRects(rects);
  // M2 (vertical) is already 100 wide in x, so its candidate set collapses to
  // the one rect; M3 (horizontal) is clamped in y and, being min-width, keeps
  // only the clamped rect.
  EXPECT_THAT(rects[1], ::testing::ElementsAre(odb::Rect(450, 0, 550, 400)));
  EXPECT_THAT(rects[2], ::testing::ElementsAre(odb::Rect(450, 150, 550, 250)));
}

// -------- Other option accessors --------

// The cut pitch and max rows/columns options round-trip.
TEST_F(TestConnect, CutPitchAndMaxRowsColumnsRoundTrip)
{
  TestableConnect connect(grid_, m(1), m(4));

  EXPECT_FALSE(connect.hasCutPitch());
  connect.setCutPitch(120, 240);
  EXPECT_TRUE(connect.hasCutPitch());
  EXPECT_EQ(connect.getCutPitchX(), 120);
  EXPECT_EQ(connect.getCutPitchY(), 240);

  connect.setMaxRows(3);
  connect.setMaxColumns(5);
  EXPECT_EQ(connect.getMaxRows(), 3);
  EXPECT_EQ(connect.getMaxColumns(), 5);
}

// setSplitCuts drops the stack's endpoints -- only intermediate layers can be
// split -- and reports pitch/stagger per remaining layer.
TEST_F(TestConnect, SplitCutsAreKeptOnlyForIntermediateLayers)
{
  TestableConnect connect(grid_, m(1), m(4));

  odb::PtrMap<odb::dbTechLayer, Connect::SplitCut> splits;
  splits[m(1)] = {/*pitch=*/200, /*stagger=*/false};
  splits[m(2)] = {/*pitch=*/300, /*stagger=*/true};
  splits[m(4)] = {/*pitch=*/400, /*stagger=*/false};
  connect.setSplitCuts(splits);

  EXPECT_EQ(connect.getSplitCutPitch(m(1)), 0);
  EXPECT_FALSE(connect.getSplitCutStagger(m(1)));
  EXPECT_EQ(connect.getSplitCutPitch(m(2)), 300);
  EXPECT_TRUE(connect.getSplitCutStagger(m(2)));
  EXPECT_EQ(connect.getSplitCutPitch(m(4)), 0);
  // A layer with no entry reports no split.
  EXPECT_EQ(connect.getSplitCutPitch(m(3)), 0);
  EXPECT_FALSE(connect.getSplitCutStagger(m(3)));
}

// -------- report --------

// report() names the min-width layers, the same way it names the ongrid
// layers -- the option is invisible in `pdngen -report_only` otherwise.
TEST_F(TestConnect, ReportNamesTheMinWidthLayers)
{
  TestableConnect connect(grid_, m(1), m(4));
  connect.setOnGrid({m(2)});
  connect.setMinWidthLayers({m(2), m(3)});

  const std::string report = captureReport(connect);
  EXPECT_THAT(report, ::testing::HasSubstr("Connect layers M1 -> M4"));
  EXPECT_THAT(report, ::testing::HasSubstr("Ongrid layers: M2"));
  EXPECT_THAT(report, ::testing::HasSubstr("Minimum width layers: M2 M3"));
}

// Without the option the line is omitted entirely, so untouched connect
// rules keep their existing report output.
TEST_F(TestConnect, ReportOmitsMinWidthLayersWhenUnset)
{
  const TestableConnect connect(grid_, m(1), m(4));

  EXPECT_THAT(captureReport(connect),
              ::testing::Not(::testing::HasSubstr("Minimum width layers")));
}

// containsIntermediateLayer covers cut layers as well as routing layers, and
// excludes the endpoints.
TEST_F(TestConnect, ContainsIntermediateLayerExcludesTheEndpoints)
{
  const TestableConnect connect(grid_, m(1), m(4));

  EXPECT_FALSE(connect.containsIntermediateLayer(m(1)));
  EXPECT_TRUE(connect.containsIntermediateLayer(m(2)));
  EXPECT_TRUE(connect.containsIntermediateLayer(stack_.cuts[0]));
  EXPECT_FALSE(connect.containsIntermediateLayer(m(4)));
}

// getAllLayers walks the whole stack; getAllRoutingLayers skips the cuts.
TEST_F(TestConnect, GetAllLayersAndGetAllRoutingLayers)
{
  const TestableConnect connect(grid_, m(1), m(4));

  EXPECT_THAT(connect.getAllLayers(),
              ::testing::ElementsAre(m(1),
                                     stack_.cuts[0],
                                     m(2),
                                     stack_.cuts[1],
                                     m(3),
                                     stack_.cuts[2],
                                     m(4)));
  EXPECT_THAT(connect.getAllRoutingLayers(),
              ::testing::ElementsAre(m(1), m(2), m(3), m(4)));
}

// overlaps / startsBelow compare the layer ranges of two connect rules.
TEST_F(TestConnect, OverlapsAndStartsBelowCompareLayerRanges)
{
  const TestableConnect low(grid_, m(1), m(2));
  const TestableConnect high(grid_, m(3), m(4));
  const TestableConnect spanning(grid_, m(2), m(3));

  EXPECT_TRUE(low.startsBelow(&high));
  EXPECT_FALSE(high.startsBelow(&low));

  EXPECT_FALSE(low.overlaps(&high));
  EXPECT_FALSE(high.overlaps(&low));
  EXPECT_TRUE(low.overlaps(&spanning));
  EXPECT_TRUE(spanning.overlaps(&high));
}

}  // namespace
}  // namespace pdn
