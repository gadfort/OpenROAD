// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, The OpenROAD Authors

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "PdnTest.h"
#include "domain.h"
#include "gtest/gtest.h"
#include "odb/db.h"
#include "pdn/PdnGen.hh"

namespace pdn {
namespace {

// Fixture for VoltageDomain tests. Creates power/ground nets and a
// floorplan area so getDomainArea/getRows have something to work with.
class TestDomain : public PdnTest
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
    block()->setDieArea(odb::Rect(0, 0, 10000, 10000));
    block()->setCoreArea(odb::Rect(100, 100, 9900, 9900));
    pdngen_ = std::make_unique<PdnGen>(db_.get(), getLogger());
  }

  std::unique_ptr<PdnGen> pdngen_;
  odb::dbNet* power_ = nullptr;
  odb::dbNet* ground_ = nullptr;
};

// -------- Core (named) constructor --------

// The core 6-arg constructor stores all of its arguments and sets the
// name to "Core".
TEST_F(TestDomain, CoreConstructorStoresValuesAndDefaultName)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_,
                  /*secondary=*/{}, getLogger());
  EXPECT_EQ(d.getName(), "Core");
  EXPECT_EQ(d.getBlock(), block());
  EXPECT_EQ(d.getLogger(), getLogger());
  EXPECT_EQ(d.getPDNGen(), pdngen_.get());
  EXPECT_EQ(d.getPower(), power_);            // no switched -> falls through
  EXPECT_EQ(d.getAlwaysOnPower(), power_);
  EXPECT_EQ(d.getGround(), ground_);
  EXPECT_EQ(d.getSwitchedPower(), nullptr);
  EXPECT_FALSE(d.hasSwitchedPower());
  EXPECT_FALSE(d.hasRegion());
  EXPECT_EQ(d.getRegion(), nullptr);
}

// -------- Named region constructor --------

// The 7-arg constructor stores the supplied name and region.
TEST_F(TestDomain, NamedConstructorStoresRegion)
{
  odb::dbRegion* region = odb::dbRegion::create(block(), "macroDom");
  odb::dbBox::create(region, /*x1=*/0, /*y1=*/0, /*x2=*/100, /*y2=*/100);

  VoltageDomain d(pdngen_.get(), "macroDom", block(), power_, ground_,
                  /*secondary=*/{}, region, getLogger());
  EXPECT_EQ(d.getName(), "macroDom");
  EXPECT_TRUE(d.hasRegion());
  EXPECT_EQ(d.getRegion(), region);
}

// A region with zero boundary rectangles makes the constructor log an
// error (which throws).
TEST_F(TestDomain, NamedConstructorThrowsOnEmptyRegion)
{
  odb::dbRegion* region = odb::dbRegion::create(block(), "emptyDom");
  // No dbBox::create() call -> region has 0 rects.
  EXPECT_THROW(
      VoltageDomain(pdngen_.get(), "emptyDom", block(), power_, ground_,
                    /*secondary=*/{}, region, getLogger()),
      std::runtime_error);
}

// A region with multiple boundary rectangles is rejected with an error.
TEST_F(TestDomain, NamedConstructorThrowsOnMultiRectRegion)
{
  odb::dbRegion* region = odb::dbRegion::create(block(), "multiDom");
  odb::dbBox::create(region, 0, 0, 100, 100);
  odb::dbBox::create(region, 200, 200, 300, 300);
  EXPECT_THROW(
      VoltageDomain(pdngen_.get(), "multiDom", block(), power_, ground_,
                    /*secondary=*/{}, region, getLogger()),
      std::runtime_error);
}

// -------- determinePowerGroundNets --------

// When power is supplied but ground is null, the constructor finds the
// ground net by sig type and logs an info message.
TEST_F(TestDomain, CoreConstructorFindsGroundFromTypeWhenNull)
{
  VoltageDomain d(pdngen_.get(), block(), power_, /*ground=*/nullptr,
                  {}, getLogger());
  EXPECT_EQ(d.getGround(), ground_);
}

// And the symmetric case: ground supplied, power null -> looked up.
TEST_F(TestDomain, CoreConstructorFindsPowerFromTypeWhenNull)
{
  VoltageDomain d(pdngen_.get(), block(), /*power=*/nullptr, ground_,
                  {}, getLogger());
  EXPECT_EQ(d.getAlwaysOnPower(), power_);
}

// When the lookup finds zero matching nets, the constructor errors.
// We force this by retyping the fixture's POWER net to SIGNAL so the
// block contains no power-typed nets at all.
TEST_F(TestDomain, CoreConstructorThrowsWhenNoPowerNet)
{
  power_->setSigType(odb::dbSigType::SIGNAL);  // no POWER nets remain
  EXPECT_THROW(
      VoltageDomain(pdngen_.get(), block(), /*power=*/nullptr, ground_,
                    {}, getLogger()),
      std::runtime_error);
}

// When the lookup finds multiple matching nets, the constructor errors.
// Add a second POWER-typed net so findDomainNet returns more than one.
TEST_F(TestDomain, CoreConstructorThrowsWhenMultiplePowerNets)
{
  odb::dbNet* p2 = odb::dbNet::create(block(), "VDD2");
  p2->setSigType(odb::dbSigType::POWER);
  EXPECT_THROW(
      VoltageDomain(pdngen_.get(), block(), /*power=*/nullptr, ground_,
                    {}, getLogger()),
      std::runtime_error);
}

// -------- getNets ordering --------

// start_with_power=true puts power first, then switched, then ground.
TEST_F(TestDomain, GetNetsStartWithPowerOrder)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  const auto nets = d.getNets(/*start_with_power=*/true);
  ASSERT_EQ(nets.size(), 2u);
  EXPECT_EQ(nets[0], power_);
  EXPECT_EQ(nets[1], ground_);
}

// start_with_power=false swaps the order.
TEST_F(TestDomain, GetNetsStartWithGroundOrder)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  const auto nets = d.getNets(/*start_with_power=*/false);
  ASSERT_EQ(nets.size(), 2u);
  EXPECT_EQ(nets[0], ground_);
  EXPECT_EQ(nets[1], power_);
}

// Secondary nets are appended at the end regardless of order.
TEST_F(TestDomain, GetNetsAppendsSecondaryNets)
{
  odb::dbNet* sec = odb::dbNet::create(block(), "SEC");
  sec->setSigType(odb::dbSigType::POWER);
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {sec},
                  getLogger());
  const auto nets_p = d.getNets(true);
  ASSERT_EQ(nets_p.size(), 3u);
  EXPECT_EQ(nets_p.back(), sec);
  const auto nets_g = d.getNets(false);
  ASSERT_EQ(nets_g.size(), 3u);
  EXPECT_EQ(nets_g.back(), sec);
}

// When a switched power net is set, it appears between power and ground
// (start_with_power=true).
TEST_F(TestDomain, GetNetsIncludesSwitchedPowerStartWithPower)
{
  odb::dbNet* sw = odb::dbNet::create(block(), "VDDS");
  sw->setSigType(odb::dbSigType::POWER);
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  d.setSwitchedPower(sw);
  EXPECT_TRUE(d.hasSwitchedPower());
  EXPECT_EQ(d.getSwitchedPower(), sw);

  const auto nets = d.getNets(/*start_with_power=*/true);
  ASSERT_EQ(nets.size(), 3u);
  EXPECT_EQ(nets[0], power_);
  EXPECT_EQ(nets[1], sw);
  EXPECT_EQ(nets[2], ground_);
}

// And the symmetric start_with_power=false ordering.
TEST_F(TestDomain, GetNetsIncludesSwitchedPowerStartWithGround)
{
  odb::dbNet* sw = odb::dbNet::create(block(), "VDDS2");
  sw->setSigType(odb::dbSigType::POWER);
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  d.setSwitchedPower(sw);

  const auto nets = d.getNets(/*start_with_power=*/false);
  ASSERT_EQ(nets.size(), 3u);
  EXPECT_EQ(nets[0], ground_);
  EXPECT_EQ(nets[1], power_);
  EXPECT_EQ(nets[2], sw);
}

// getPower returns switched_power_ when set, else power_.
TEST_F(TestDomain, GetPowerReturnsSwitchedWhenSet)
{
  odb::dbNet* sw = odb::dbNet::create(block(), "VDDS3");
  sw->setSigType(odb::dbSigType::POWER);
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  EXPECT_EQ(d.getPower(), power_);
  d.setSwitchedPower(sw);
  EXPECT_EQ(d.getPower(), sw);
}

// -------- Grid management --------

// addGrid stores the grid; getGrids returns the stored grids in order.
TEST_F(TestDomain, AddGridAppendsAndGetGridsReturnsThem)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  EXPECT_TRUE(d.getGrids().empty());

  auto grid = std::make_unique<TestGrid>(
      &d, "g1", /*starts_with_power=*/true,
      std::vector<odb::dbTechLayer*>{});
  Grid* raw = grid.get();
  d.addGrid(std::move(grid));
  ASSERT_EQ(d.getGrids().size(), 1u);
  EXPECT_EQ(d.getGrids().front().get(), raw);
}

// clearGrids removes all grids.
TEST_F(TestDomain, ClearGridsEmptiesTheVector)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  d.addGrid(std::make_unique<TestGrid>(
      &d, "g", true, std::vector<odb::dbTechLayer*>{}));
  ASSERT_EQ(d.getGrids().size(), 1u);
  d.clearGrids();
  EXPECT_TRUE(d.getGrids().empty());
}

// removeGrid removes the specific grid by pointer.
TEST_F(TestDomain, RemoveGridErasesMatchingGrid)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  auto g1 = std::make_unique<TestGrid>(
      &d, "g1", true, std::vector<odb::dbTechLayer*>{});
  Grid* g1_raw = g1.get();
  auto g2 = std::make_unique<TestGrid>(
      &d, "g2", true, std::vector<odb::dbTechLayer*>{});
  Grid* g2_raw = g2.get();
  d.addGrid(std::move(g1));
  d.addGrid(std::move(g2));
  ASSERT_EQ(d.getGrids().size(), 2u);
  d.removeGrid(g1_raw);
  ASSERT_EQ(d.getGrids().size(), 1u);
  EXPECT_EQ(d.getGrids().front().get(), g2_raw);
}

// -------- getDomainArea --------

// Without a region, getDomainArea returns the block's core area.
TEST_F(TestDomain, GetDomainAreaWithoutRegionReturnsCoreArea)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  EXPECT_EQ(d.getDomainArea(), block()->getCoreArea());
}

// With a region, getDomainArea returns the region's first boundary rect.
TEST_F(TestDomain, GetDomainAreaWithRegionReturnsRegionRect)
{
  odb::dbRegion* region = odb::dbRegion::create(block(), "areaDom");
  odb::dbBox::create(region, /*x1=*/200, /*y1=*/300, /*x2=*/800,
                     /*y2=*/700);
  VoltageDomain d(pdngen_.get(), "areaDom", block(), power_, ground_, {},
                  region, getLogger());
  EXPECT_EQ(d.getDomainArea(), odb::Rect(200, 300, 800, 700));
}

// -------- getRows --------

// With no rows on the block, getRows returns empty.
TEST_F(TestDomain, GetRowsEmptyBlock)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  EXPECT_TRUE(d.getRows().empty());
}

// getRows skips PAD rows and includes core rows.
TEST_F(TestDomain, GetRowsSkipsPadRows)
{
  odb::dbSite* core_site
      = odb::dbSite::create(db_->findLib("lib"), "core");
  core_site->setWidth(200);
  core_site->setHeight(2800);
  core_site->setClass(odb::dbSiteClass::CORE);
  odb::dbRow::create(block(), "core_row", core_site, 100, 100,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     /*num_sites=*/10, /*spacing=*/200);

  odb::dbSite* pad_site
      = odb::dbSite::create(db_->findLib("lib"), "pad");
  pad_site->setWidth(500);
  pad_site->setHeight(500);
  pad_site->setClass(odb::dbSiteClass::PAD);
  odb::dbRow::create(block(), "pad_row", pad_site, 0, 0,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     1, 500);

  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  const auto rows = d.getRows();
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front()->getName(), "core_row");
}

// getRows with a region returns rows that overlap the region.
TEST_F(TestDomain, GetRowsWithRegionFiltersByOverlap)
{
  odb::dbSite* core_site
      = odb::dbSite::create(db_->findLib("lib"), "rcore");
  core_site->setWidth(200);
  core_site->setHeight(100);
  core_site->setClass(odb::dbSiteClass::CORE);
  // Row inside the region.
  odb::dbRow::create(block(), "in", core_site, 100, 100,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     5, 200);
  // Row outside the region.
  odb::dbRow::create(block(), "out", core_site, 5000, 5000,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     5, 200);

  odb::dbRegion* region = odb::dbRegion::create(block(), "regdom");
  odb::dbBox::create(region, 0, 0, 2000, 2000);
  VoltageDomain d(pdngen_.get(), "regdom", block(), power_, ground_, {},
                  region, getLogger());
  const auto rows = d.getRows();
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front()->getName(), "in");
}

// getRegionRows skips PAD-class rows the same way the core-domain path
// does. With one PAD row overlapping the region and one CORE row, only
// the CORE row should be returned.
TEST_F(TestDomain, GetRegionRowsSkipsPadRows)
{
  odb::dbSite* core_site
      = odb::dbSite::create(db_->findLib("lib"), "regcore");
  core_site->setWidth(200);
  core_site->setHeight(100);
  core_site->setClass(odb::dbSiteClass::CORE);
  odb::dbSite* pad_site
      = odb::dbSite::create(db_->findLib("lib"), "regpad");
  pad_site->setWidth(500);
  pad_site->setHeight(500);
  pad_site->setClass(odb::dbSiteClass::PAD);

  odb::dbRow::create(block(), "rcore", core_site, 100, 100,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     2, 200);
  odb::dbRow::create(block(), "rpad", pad_site, 200, 200,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     1, 500);
  odb::dbRegion* region = odb::dbRegion::create(block(), "padfilter");
  odb::dbBox::create(region, 0, 0, 5000, 5000);
  VoltageDomain d(pdngen_.get(), "padfilter", block(), power_, ground_,
                  {}, region, getLogger());
  const auto rows = d.getRows();
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front()->getName(), "rcore");
}

// When the core domain is registered with the PdnGen, getDomainRows
// iterates pdngen->getDomains() and skips the domain that == this.
TEST_F(TestDomain, GetRowsCoreDomainSkipsItselfInPdnGen)
{
  // Register a core domain with the pdngen. getDomains() will then
  // include this very instance.
  pdngen_->setCoreDomain(power_, /*switched=*/nullptr, ground_, {});
  VoltageDomain* core = pdngen_->getDomains().front();
  ASSERT_NE(core, nullptr);

  // Add one core row so getRows has something to consider.
  odb::dbSite* site = odb::dbSite::create(db_->findLib("lib"), "selfsite");
  site->setWidth(200);
  site->setHeight(100);
  site->setClass(odb::dbSiteClass::CORE);
  odb::dbRow::create(block(), "self_row", site, 100, 100,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     2, 200);

  const auto rows = core->getRows();
  EXPECT_EQ(rows.size(), 1u);
}

// When the block contains a region with zero boundary rectangles, the
// core domain's getRows -> getDomainRows -> getRegionBoundary call
// exercises the empty-boundary fall-through (returning {}). The region
// is just registered in the block; we don't have to wrap it in a domain.
TEST_F(TestDomain, GetRowsHandlesBlockRegionWithoutBoundaries)
{
  odb::dbRegion::create(block(), "empty_region");  // no boundary
  odb::dbSite* site = odb::dbSite::create(db_->findLib("lib"), "drsite");
  site->setWidth(200);
  site->setHeight(100);
  site->setClass(odb::dbSiteClass::CORE);
  odb::dbRow::create(block(), "drow", site, 100, 100,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     2, 200);
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  EXPECT_NO_THROW(d.getRows());
}

// A core-domain (no region) excludes rows already claimed by another
// region domain registered with the PdnGen.
TEST_F(TestDomain, GetRowsCoreDomainExcludesClaimedRows)
{
  odb::dbSite* core_site
      = odb::dbSite::create(db_->findLib("lib"), "csite");
  core_site->setWidth(200);
  core_site->setHeight(100);
  core_site->setClass(odb::dbSiteClass::CORE);
  odb::dbRow::create(block(), "claimed", core_site, 100, 100,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     2, 200);
  odb::dbRow::create(block(), "free", core_site, 5000, 5000,
                     odb::dbOrientType::R0, odb::dbRowDir::HORIZONTAL,
                     2, 200);

  // Register a region domain with the PdnGen that owns the "claimed" row.
  odb::dbRegion* region = odb::dbRegion::create(block(), "claimsRegion");
  odb::dbBox::create(region, 0, 0, 1000, 1000);
  pdngen_->makeRegionVoltageDomain("claimsRegion", power_, /*sw=*/nullptr,
                                   ground_, {}, region);

  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  const auto rows = d.getRows();
  // Only "free" row remains for the core domain.
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front()->getName(), "free");
}

// -------- getRegionRectCount / getRegionBoundary (via getDomainArea) --------

// getDomainArea on a domain whose region had a single rect returns
// exactly that rect (covered above), and a region created with a
// removed/empty boundary just returns an inverted/default Rect.
// (Behavior is reachable through getDomainArea even though
// getRegionBoundary is private.)
// -- additional coverage via the empty-region constructor error path
// already exercised by NamedConstructorThrowsOnEmptyRegion above.

// -------- report / checkSetup (no-throw smoke tests) --------

// report on a bare domain logs without error.
TEST_F(TestDomain, ReportEmitsLines)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  EXPECT_NO_THROW(d.report());
}

// report with switched power + secondary + region exercises every
// branch of the reporting code.
TEST_F(TestDomain, ReportEmitsAllOptionalFields)
{
  odb::dbNet* sw = odb::dbNet::create(block(), "VDDSW");
  sw->setSigType(odb::dbSigType::POWER);
  odb::dbNet* sec = odb::dbNet::create(block(), "SEC2");
  sec->setSigType(odb::dbSigType::POWER);
  odb::dbRegion* region = odb::dbRegion::create(block(), "fullDom");
  odb::dbBox::create(region, 0, 0, 100, 100);
  VoltageDomain d(pdngen_.get(), "fullDom", block(), power_, ground_,
                  {sec}, region, getLogger());
  d.setSwitchedPower(sw);
  EXPECT_NO_THROW(d.report());
  // Also include at least one grid so report() walks the grids loop.
  d.addGrid(std::make_unique<TestGrid>(
      &d, "g", true, std::vector<odb::dbTechLayer*>{}));
  EXPECT_NO_THROW(d.report());
}

// checkSetup on a bare domain (no grids) is a no-op.
TEST_F(TestDomain, CheckSetupOnEmptyDomainIsNoOp)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  EXPECT_NO_THROW(d.checkSetup());
}

// checkSetup forwards to each grid's checkSetup.
TEST_F(TestDomain, CheckSetupDelegatesToGrids)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  d.addGrid(std::make_unique<TestGrid>(
      &d, "g", true, std::vector<odb::dbTechLayer*>{}));
  EXPECT_NO_THROW(d.checkSetup());
}

// -------- resetGrids --------

// resetGrids forwards to each grid's resetShapes (no-op for a fresh
// TestGrid).
TEST_F(TestDomain, ResetGridsForwardsToEachGrid)
{
  VoltageDomain d(pdngen_.get(), block(), power_, ground_, {}, getLogger());
  d.addGrid(std::make_unique<TestGrid>(
      &d, "g", true, std::vector<odb::dbTechLayer*>{}));
  EXPECT_NO_THROW(d.resetGrids());
}

}  // namespace
}  // namespace pdn
