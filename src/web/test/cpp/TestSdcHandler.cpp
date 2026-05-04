// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors
//
// Tests for the SDC web handler. There are two fixtures:
//
//   * NullStaTest exercises the empty-fallback path (no STA loaded). All
//     handlers must return a well-formed JSON envelope without crashing.
//     Parameterised so each handler is one line in a table.
//
//   * SdcHandlerWithStaTest builds a small Nangate45 design programmatically,
//     applies a rich set of SDC commands, and asserts on the actual JSON
//     payload values (not substring matches).

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "boost/json.hpp"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "gtest/gtest.h"
#include "odb/db.h"
#include "request_handler.h"
#include "sta/Clock.hh"
#include "sta/ClockGroups.hh"
#include "sta/ExceptionPath.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Mode.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDelay.hh"
#include "sta/Sdc.hh"
#include "sta/SdcClass.hh"
#include "sta/Transition.hh"
#include "sta/Units.hh"
#include "tile_generator.h"
#include "tst/nangate45_fixture.h"

namespace web {
namespace {

namespace bj = boost::json;

static std::string payloadStr(const WebSocketResponse& resp)
{
  return std::string(resp.payload.begin(), resp.payload.end());
}

// Parse the payload as JSON. ADD_FAILURE on parse error so the test still
// reports both the parse error and any earlier expectations.
static bj::value parsePayload(const WebSocketResponse& resp)
{
  const std::string s = payloadStr(resp);
  try {
    return bj::parse(s);
  } catch (const std::exception& e) {
    ADD_FAILURE() << "Payload is not valid JSON: " << e.what()
                  << "\nPayload: " << s;
    return bj::value();
  }
}

// Find the first object in `arr` for which obj["name_key"] == name.
// Returns nullptr if not found.
static const bj::object* findByName(const bj::array& arr,
                                    std::string_view name_key,
                                    std::string_view name)
{
  for (const auto& el : arr) {
    if (!el.is_object()) {
      continue;
    }
    const auto& obj = el.as_object();
    auto it = obj.find(name_key);
    if (it != obj.end() && it->value().is_string()
        && it->value().as_string() == name) {
      return &obj;
    }
  }
  return nullptr;
}

// Relative-tolerance double comparison (printf "%g" round-trips lose ~6 sig
// figs).
static bool nearly(double a, double b, double rel_tol = 1e-5)
{
  double diff = std::fabs(a - b);
  if (diff <= rel_tol) {
    return true;
  }
  double scale = std::max(std::fabs(a), std::fabs(b));
  return diff <= rel_tol * scale;
}

// Read a numeric field whether boost stored it as int64 or double.
static double asNumber(const bj::value& v)
{
  if (v.is_int64()) {
    return static_cast<double>(v.as_int64());
  }
  if (v.is_uint64()) {
    return static_cast<double>(v.as_uint64());
  }
  return v.as_double();
}

static WebSocketRequest makeReq(uint32_t id, WebSocketRequest::Type type)
{
  WebSocketRequest req;
  req.id = id;
  req.type = type;
  // Empty json is fine — extract_string returns "" and extract_int_or
  // returns the default when keys are missing.
  return req;
}

// Convenience: assemble a minimal request body with the given key/value
// pairs. Returns a boost::json::object directly so the test can assign
// it straight into `WebSocketRequest::json`.
static bj::object makeJson(
    std::initializer_list<std::pair<const char*, std::string>> string_fields,
    std::initializer_list<std::pair<const char*, int>> int_fields = {})
{
  bj::object out;
  for (const auto& [k, v] : string_fields) {
    out[k] = v;
  }
  for (const auto& [k, v] : int_fields) {
    out[k] = v;
  }
  return out;
}

// Parse a JSON-string literal into a `boost::json::object`. Convenience
// for the few tests that hand-craft a raw JSON string instead of using
// `makeJson` (escaped names, dynamic substitutions, etc.).
static bj::object parseObj(const std::string& s)
{
  return bj::parse(s).as_object();
}

// ─── NullStaTest: parameterised null-fallback coverage ──────────────────────
//
// One TEST_P body verifies the schema-shape contract every handler upholds
// when STA is absent: response envelope is a JSON object that contains the
// listed top-level keys. Replaces ~30 hand-written substring-existence tests.

struct NullCase
{
  const char* name;
  WebSocketRequest::Type type;
  // Top-level keys the empty-fallback payload must always emit so the
  // frontend can read them unconditionally.
  std::vector<const char*> required_keys;
  const char* pin_name = nullptr;
  const char* mode_name = nullptr;
};

class NullStaTest : public tst::Nangate45Fixture,
                    public ::testing::WithParamInterface<NullCase>
{
 protected:
  void SetUp() override
  {
    block_->setDieArea(odb::Rect(0, 0, 100000, 100000));
    gen_ = std::make_shared<TileGenerator>(
        getDb(), /*sta=*/nullptr, getLogger());
    handler_ = std::make_unique<SdcHandler>(gen_);
  }

  WebSocketResponse dispatch(const WebSocketRequest& req)
  {
    switch (req.type) {
      case WebSocketRequest::kSdcClocks:
        return handler_->handleSdcClocks(req);
      case WebSocketRequest::kSdcClockModes:
        return handler_->handleSdcClockModes(req);
      case WebSocketRequest::kSdcPortDelays:
        return handler_->handleSdcPortDelays(req);
      case WebSocketRequest::kSdcExceptions:
        return handler_->handleSdcExceptions(req);
      case WebSocketRequest::kSdcLimits:
        return handler_->handleSdcLimits(req);
      case WebSocketRequest::kSdcClockGroups:
        return handler_->handleSdcClockGroups(req);
      case WebSocketRequest::kSdcEndpoint:
        return handler_->handleSdcEndpoint(req);
      case WebSocketRequest::kSdcEndpointList:
        return handler_->handleSdcEndpointList(req);
      case WebSocketRequest::kSdcEndpointCounts:
        return handler_->handleSdcEndpointCounts(req);
      case WebSocketRequest::kSdcListModes:
        return handler_->handleSdcListModes(req);
      case WebSocketRequest::kSdcSetMode:
        return handler_->handleSdcSetMode(req);
      case WebSocketRequest::kSdcResolveGenClocks:
        return handler_->handleSdcResolveGenClocks(req);
      default:
        ADD_FAILURE() << "unhandled request type";
        return {};
    }
  }

  std::shared_ptr<TileGenerator> gen_;
  std::unique_ptr<SdcHandler> handler_;
};

TEST_P(NullStaTest, EnvelopeAndKeys)
{
  const NullCase& tc = GetParam();
  WebSocketRequest req = makeReq(/*id=*/123, tc.type);
  // sdc_pin_name was historically used for both endpoint ("pin") and
  // endpoint_list ("pattern"). Set both keys; each handler reads only
  // the one that's relevant to it.
  //
  // Each branch calls makeJson() inline rather than building a
  // shared `std::initializer_list` variable: an initializer_list's
  // backing array's lifetime is tied to the expression that creates
  // it. Storing one in a variable produces a dangling reference and
  // corrupts the heap when downstream code reads it (manifests as
  // std::bad_alloc — was reproducible on the SetMode case).
  if (tc.pin_name && tc.mode_name) {
    req.json = makeJson({{"pin", tc.pin_name},
                             {"pattern", tc.pin_name},
                             {"mode", tc.mode_name}});
  } else if (tc.pin_name) {
    req.json = makeJson({{"pin", tc.pin_name}, {"pattern", tc.pin_name}});
  } else if (tc.mode_name) {
    req.json = makeJson({{"mode", tc.mode_name}});
  } else {
    req.json = makeJson({});
  }

  auto resp = dispatch(req);
  EXPECT_EQ(resp.id, 123u);
  EXPECT_EQ(resp.type, WebSocketResponse::kJson);

  auto val = parsePayload(resp);
  ASSERT_TRUE(val.is_object()) << "payload must be a JSON object";
  const auto& obj = val.as_object();
  for (const char* key : tc.required_keys) {
    EXPECT_TRUE(obj.contains(key)) << "missing required key: " << key;
  }
}

INSTANTIATE_TEST_SUITE_P(
    AllHandlers,
    NullStaTest,
    ::testing::Values(
        NullCase{"Clocks",
                 WebSocketRequest::kSdcClocks,
                 {"clocks", "clock_tree", "time_unit"}},
        NullCase{
            "Modes",
            WebSocketRequest::kSdcClockModes,
            {"current_mode", "scene_names", "case_analysis", "logic_values"}},
        NullCase{"PortDelays",
                 WebSocketRequest::kSdcPortDelays,
                 {"time_unit", "total", "offset", "port_delays"}},
        NullCase{"Exceptions",
                 WebSocketRequest::kSdcExceptions,
                 {"time_unit", "exceptions"}},
        NullCase{"Limits",
                 WebSocketRequest::kSdcLimits,
                 {"time_unit",
                  "cap_unit",
                  "clock_latencies",
                  "clock_insertions",
                  "clock_uncertainties",
                  "port_loads",
                  "disabled_timing"}},
        NullCase{"ClockGroups", WebSocketRequest::kSdcClockGroups, {"groups"}},
        NullCase{"Endpoint",
                 WebSocketRequest::kSdcEndpoint,
                 {"found", "multi", "pins", "time_unit"},
                 /*pin_name=*/"any_pin"},
        NullCase{"EndpointList",
                 WebSocketRequest::kSdcEndpointList,
                 {"endpoints", "kinds_total", "total"},
                 /*pin_name=*/"u_top/*"},
        NullCase{"ListModes",
                 WebSocketRequest::kSdcListModes,
                 {"modes", "current"}},
        NullCase{"SetMode",
                 WebSocketRequest::kSdcSetMode,
                 {"ok", "current", "error"},
                 /*pin_name=*/nullptr,
                 /*mode_name=*/"functional"},
        NullCase{"ResolveGenClocks",
                 WebSocketRequest::kSdcResolveGenClocks,
                 {"ok", "resolved"}}),
    [](const auto& info) { return info.param.name; });

// Spot-check a few semantic invariants the schema test alone doesn't catch.

TEST(NullStaSpecific, EndpointReportsNotFound)
{
  SdcHandler handler(nullptr);
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpoint);
  req.json = makeJson({{"pin", "no_such_pin"}});

  auto val = bj::parse(payloadStr(handler.handleSdcEndpoint(req)));
  ASSERT_TRUE(val.is_object());
  const auto& obj = val.as_object();
  ASSERT_TRUE(obj.contains("found"));
  EXPECT_EQ(obj.at("found").as_bool(), false);
  ASSERT_TRUE(obj.contains("pins"));
  EXPECT_EQ(obj.at("pins").as_array().size(), 0u);
}

TEST(NullStaSpecific, SetModeReportsFailure)
{
  SdcHandler handler(nullptr);
  WebSocketRequest req = makeReq(2, WebSocketRequest::kSdcSetMode);
  req.json = makeJson({{"mode", "any"}});

  auto val = bj::parse(payloadStr(handler.handleSdcSetMode(req)));
  const auto& obj = val.as_object();
  ASSERT_TRUE(obj.contains("ok"));
  EXPECT_FALSE(obj.at("ok").as_bool());
  ASSERT_TRUE(obj.contains("error"));
  EXPECT_FALSE(obj.at("error").as_string().empty());
}

TEST(NullStaSpecific, ResolveGenClocksReportsFailure)
{
  SdcHandler handler(nullptr);
  WebSocketRequest req = makeReq(3, WebSocketRequest::kSdcResolveGenClocks);

  auto val = bj::parse(payloadStr(handler.handleSdcResolveGenClocks(req)));
  const auto& obj = val.as_object();
  ASSERT_TRUE(obj.contains("ok"));
  EXPECT_FALSE(obj.at("ok").as_bool());
  ASSERT_TRUE(obj.contains("resolved"));
  EXPECT_EQ(asNumber(obj.at("resolved")), 0.0);
}

TEST(NullStaSpecific, PortDelaysOffsetClampedToZero)
{
  SdcHandler handler(nullptr);
  WebSocketRequest req = makeReq(4, WebSocketRequest::kSdcPortDelays);
  req.json = makeJson({}, {{"offset", -5}, {"limit", 10}});

  auto val = bj::parse(payloadStr(handler.handleSdcPortDelays(req)));
  const auto& obj = val.as_object();
  ASSERT_TRUE(obj.contains("offset"));
  EXPECT_EQ(asNumber(obj.at("offset")), 0.0);
}

// ─── SdcHandlerWithStaTest: real STA + design + SDC ─────────────────────────
//
// Builds a tiny Nangate45 design programmatically and applies a rich set of
// SDC commands. This is the first real-STA fixture in src/web/test/cpp/ —
// most existing web tests run with sta=nullptr because they don't depend on
// timing. The SDC handler is the one exception, since its entire job is to
// emit timing-constraint state.

class SdcHandlerWithStaTest : public tst::Nangate45Fixture
{
 protected:
  void SetUp() override
  {
    library_ = readLiberty("_main/test/Nangate45/Nangate45_typ.lib");
    ASSERT_NE(library_, nullptr);

    db_network_ = sta_->getDbNetwork();
    db_network_->setBlock(block_);
    block_->setDieArea(odb::Rect(0, 0, 1000, 1000));
    sta_->postReadDef(block_);

    buildDesign();
    applySdc();

    gen_ = std::make_shared<TileGenerator>(getDb(), sta_.get(), getLogger());
    handler_ = std::make_unique<SdcHandler>(gen_);
  }

  // Build a tiny flop chain:
  //
  //   in1 ──► ff1[D] ─ff1[Q]─► ff2[D] ─ff2[Q]──► out1
  //                        clk_in ─► both CK pins
  //
  // and one combinational pass-through (in2 → buf → out2) so we have an
  // input-delay constraint on a non-flop endpoint as well.
  void buildDesign()
  {
    const char* layer = "metal1";
    auto pinRect = [](int x) { return odb::Rect(x, 0, x + 10, 10); };

    clk_in_ = makeBTerm(block_,
                        "clk_in",
                        {.bpins = {{.layer_name = layer, .rect = pinRect(0)}}});
    in1_ = makeBTerm(
        block_, "in1", {.bpins = {{.layer_name = layer, .rect = pinRect(20)}}});
    in2_ = makeBTerm(
        block_, "in2", {.bpins = {{.layer_name = layer, .rect = pinRect(40)}}});
    out1_ = makeBTerm(block_,
                      "out1",
                      {.io_type = odb::dbIoType::OUTPUT,
                       .bpins = {{.layer_name = layer, .rect = pinRect(60)}}});
    out2_ = makeBTerm(block_,
                      "out2",
                      {.io_type = odb::dbIoType::OUTPUT,
                       .bpins = {{.layer_name = layer, .rect = pinRect(80)}}});

    odb::dbMaster* dff = db_->findMaster("DFF_X1");
    ASSERT_NE(dff, nullptr);
    odb::dbMaster* buf = db_->findMaster("BUF_X1");
    ASSERT_NE(buf, nullptr);

    ff1_ = tst::Fixture::makeInst(
        block_,
        dff,
        "ff1",
        {.location = {100, 100},
         .status = odb::dbPlacementStatus::PLACED,
         .iterms = {{.net_name = "in1", .term_name = "D"},
                    {.net_name = "clk_in", .term_name = "CK"},
                    {.net_name = "n_q1", .term_name = "Q"}}});
    ff2_ = tst::Fixture::makeInst(
        block_,
        dff,
        "ff2",
        {.location = {200, 100},
         .status = odb::dbPlacementStatus::PLACED,
         .iterms = {{.net_name = "n_q1", .term_name = "D"},
                    {.net_name = "clk_in", .term_name = "CK"},
                    {.net_name = "out1", .term_name = "Q"}}});
    tst::Fixture::makeInst(
        block_,
        buf,
        "u_buf",
        {.location = {300, 100},
         .status = odb::dbPlacementStatus::PLACED,
         .iterms = {{.net_name = "in2", .term_name = "A"},
                    {.net_name = "out2", .term_name = "Z"}}});
    // Real Liberty integrated clock-gating cell — Nangate45's
    // CLKGATE_X1 declares `clock_gating_integrated_cell :
    // latch_posedge`. Endpoints-list classifier should emit
    // kind=clock_gate plus the flavor + pin roles.
    odb::dbMaster* clkgate = db_->findMaster("CLKGATE_X1");
    if (clkgate) {
      tst::Fixture::makeInst(
          block_,
          clkgate,
          "icg1",
          {.location = {400, 100},
           .status = odb::dbPlacementStatus::PLACED,
           .iterms = {{.net_name = "clk_in", .term_name = "CK"},
                      {.net_name = "in1", .term_name = "E"},
                      {.net_name = "icg1_gck", .term_name = "GCK"}}});
      // Two more ICGs sharing the clock — used by the
      // KindsTotalIsStableAcrossKindFilter test to reproduce the
      // multi-ICG count drift seen in the SDC demo (3 → 2).
      tst::Fixture::makeInst(
          block_,
          clkgate,
          "icg2",
          {.location = {400, 200},
           .status = odb::dbPlacementStatus::PLACED,
           .iterms = {{.net_name = "clk_in", .term_name = "CK"},
                      {.net_name = "in2", .term_name = "E"},
                      {.net_name = "icg2_gck", .term_name = "GCK"}}});
      tst::Fixture::makeInst(
          block_,
          clkgate,
          "icg3",
          {.location = {400, 300},
           .status = odb::dbPlacementStatus::PLACED,
           .iterms = {{.net_name = "clk_in", .term_name = "CK"},
                      {.net_name = "in1", .term_name = "E"},
                      {.net_name = "icg3_gck", .term_name = "GCK"}}});
    }
  }

  void applySdc()
  {
    sta::Sdc* sdc = sta_->cmdMode()->sdc();
    auto* mode = sta_->cmdMode();
    sta::Network* net = sta_->network();

    sta::Pin* clk_pin = db_network_->dbToSta(clk_in_);
    sta::Pin* in1_pin = db_network_->dbToSta(in1_);
    sta::Pin* in2_pin = db_network_->dbToSta(in2_);
    sta::Pin* out1_pin = db_network_->dbToSta(out1_);
    sta::Pin* out2_pin = db_network_->dbToSta(out2_);
    ASSERT_NE(clk_pin, nullptr);
    ASSERT_NE(in1_pin, nullptr);

    // ---- create_clock clk_in @ period=10ns ----
    const float period = sta_->units()->timeUnit()->userToSta(kClockPeriodNs);
    sta::FloatSeq waveform{0.0f, period / 2.0f};
    sta::PinSet clk_pins(net);
    clk_pins.insert(clk_pin);
    sta_->makeClock("clk",
                    clk_pins,
                    /*add_to_pins=*/false,
                    period,
                    waveform,
                    /*comment=*/"primary",
                    mode);
    sta::Clock* clk = sdc->findClock("clk");
    ASSERT_NE(clk, nullptr);

    // ---- create_generated_clock clk_div2 -divide_by 2 ----
    odb::dbITerm* ff1_q = ff1_->findITerm("Q");
    ASSERT_NE(ff1_q, nullptr);
    sta::Pin* gen_src_pin = db_network_->dbToSta(ff1_q);
    sta::PinSet gen_pins(net);
    gen_pins.insert(gen_src_pin);
    sta_->makeGeneratedClock("clk_div2",
                             gen_pins,
                             /*add_to_pins=*/false,
                             gen_src_pin,
                             /*master_clk=*/clk,
                             /*divide_by=*/2,
                             /*multiply_by=*/0,
                             /*duty_cycle=*/0.0f,
                             /*invert=*/false,
                             /*combinational=*/false,
                             /*edges=*/sta::IntSeq{},
                             /*edge_shifts=*/sta::FloatSeq{},
                             /*comment=*/"div2",
                             mode);

    const sta::RiseFallBoth* rf = sta::RiseFallBoth::riseFall();
    const sta::RiseFall* rise = sta::RiseFall::rise();
    const sta::RiseFall* fall = sta::RiseFall::fall();
    const sta::MinMaxAll* mm = sta::MinMaxAll::all();

    // ---- set_input_delay 2.5 [get_ports in1] -clock clk ----
    sta_->setInputDelay(in1_pin,
                        rf,
                        clk,
                        rise,
                        /*ref_pin=*/nullptr,
                        /*src_lat=*/false,
                        /*net_lat=*/false,
                        mm,
                        /*add=*/false,
                        sta_->units()->timeUnit()->userToSta(kInputDelayNs),
                        sdc);

    // Second input gets a fall-launch delay to exercise the multi-lane path.
    sta_->setInputDelay(in2_pin,
                        rf,
                        clk,
                        fall,
                        /*ref_pin=*/nullptr,
                        /*src_lat=*/false,
                        /*net_lat=*/false,
                        mm,
                        /*add=*/false,
                        sta_->units()->timeUnit()->userToSta(kInputDelayNs / 2),
                        sdc);

    // ---- set_output_delay 1.5 [get_ports out1] -clock clk ----
    if (out1_pin) {
      sta_->setOutputDelay(out1_pin,
                           rf,
                           clk,
                           rise,
                           /*ref_pin=*/nullptr,
                           false,
                           false,
                           mm,
                           false,
                           sta_->units()->timeUnit()->userToSta(kOutputDelayNs),
                           sdc);
    }
    if (out2_pin) {
      sta_->setOutputDelay(out2_pin,
                           rf,
                           clk,
                           rise,
                           /*ref_pin=*/nullptr,
                           false,
                           false,
                           mm,
                           false,
                           sta_->units()->timeUnit()->userToSta(kOutputDelayNs),
                           sdc);
    }

    // ---- set_clock_uncertainty -setup 0.3 -hold 0.15 [get_clocks clk] ----
    // SetupHoldAll == MinMaxAll. setup → max(), hold → min().
    sta_->setClockUncertainty(clk,
                              sta::SetupHoldAll::max(),
                              sta_->units()->timeUnit()->userToSta(kSetupUnc));
    sta_->setClockUncertainty(clk,
                              sta::SetupHoldAll::min(),
                              sta_->units()->timeUnit()->userToSta(kHoldUnc));

    // ---- set_clock_latency 0.5 [get_clocks clk] ----
    sta_->setClockLatency(clk,
                          /*pin=*/nullptr,
                          rf,
                          mm,
                          sta_->units()->timeUnit()->userToSta(kClockLatency),
                          sdc);
    // ---- set_clock_latency -source 0.2 [get_clocks clk] ----
    sta_->setClockInsertion(clk,
                            /*pin=*/nullptr,
                            rf,
                            mm,
                            sta::EarlyLateAll::all(),
                            sta_->units()->timeUnit()->userToSta(kClockInsert),
                            sdc);

    // ---- set_false_path -from in1 -to out1 ----
    sta_->makeFalsePath(makeFromPin(net, in1_pin),
                        /*thrus=*/nullptr,
                        makeToPin(net, out1_pin),
                        mm,
                        /*comment=*/"unused timing path",
                        sdc);

    // ---- set_multicycle_path 2 -from in2 -to out2 ----
    sta_->makeMulticyclePath(makeFromPin(net, in2_pin),
                             /*thrus=*/nullptr,
                             makeToPin(net, out2_pin),
                             mm,
                             /*use_end_clk=*/false,
                             /*path_multiplier=*/kMcpMultiplier,
                             /*comment=*/"",
                             sdc);

    // ---- set_max_delay 4.0 -from in1 -to out2 ----
    sta_->makePathDelay(makeFromPin(net, in1_pin),
                        /*thrus=*/nullptr,
                        makeToPin(net, out2_pin),
                        sta::MinMax::max(),
                        /*ignore_clk_latency=*/false,
                        /*break_path=*/false,
                        sta_->units()->timeUnit()->userToSta(kPathDelayNs),
                        /*comment=*/"",
                        sdc);

    // ---- set_clock_groups -asynchronous -group {clk} -group {clk_div2} ----
    sta::ClockGroups* groups = sta_->makeClockGroups(
        /*name=*/"async_clk_clkdiv2",
        /*logically_exclusive=*/false,
        /*physically_exclusive=*/false,
        /*asynchronous=*/true,
        /*allow_paths=*/false,
        /*comment=*/"",
        sdc);
    {
      auto* g1 = new sta::ClockSet;
      g1->insert(clk);
      sta_->makeClockGroup(groups, g1, sdc);
      sta::Clock* clk_div2 = sdc->findClock("clk_div2");
      ASSERT_NE(clk_div2, nullptr);
      auto* g2 = new sta::ClockSet;
      g2->insert(clk_div2);
      sta_->makeClockGroup(groups, g2, sdc);
    }

    // ---- set_case_analysis 0 [get_pins ff1/D] ----
    if (auto* ff1_d = ff1_->findITerm("D")) {
      sta::Pin* p = db_network_->dbToSta(ff1_d);
      sta_->setCaseAnalysis(p, sta::LogicValue::zero, mode);
    }

    // ---- set_logic_one [get_pins ff2/D] ----
    if (auto* ff2_d = ff2_->findITerm("D")) {
      sta::Pin* p = db_network_->dbToSta(ff2_d);
      sta_->setLogicValue(p, sta::LogicValue::one, mode);
    }

    // ---- set_disable_timing [get_pins ff1/QN] ----
    if (auto* ff1_qn = ff1_->findITerm("QN")) {
      sta::Pin* p = db_network_->dbToSta(ff1_qn);
      sdc->disable(p);
    }

    // ── Additions covering the rest of the SDC surface the widget renders
    // (kept after the original setup so existing tests' counts only grow,
    // not change shape).

    // ---- create_clock clk_virt -period 5 (virtual, no source pins) ----
    sta::PinSet empty_pins(net);
    sta_->makeClock("clk_virt",
                    empty_pins,
                    /*add_to_pins=*/false,
                    sta_->units()->timeUnit()->userToSta(kVirtClockNs),
                    sta::FloatSeq{0.0f, kVirtClockNs / 2.0f},
                    /*comment=*/"virtual",
                    mode);

    // ---- create_generated_clock clk_x2 -multiply_by 2 -invert ----
    // Sourced from u_buf/Z so the multiplied clock's domain doesn't
    // bleed into the existing flop chain — out1's domain (clk via the
    // ff1→ff2 path) stays clean of clk_x2 for the endpoint test.
    if (auto* buf_inst = block_->findInst("u_buf")) {
      if (auto* buf_z = buf_inst->findITerm("Z")) {
        sta::Pin* x2_src = db_network_->dbToSta(buf_z);
        sta::PinSet x2_pins(net);
        x2_pins.insert(x2_src);
        sta_->makeGeneratedClock("clk_x2",
                                 x2_pins,
                                 false,
                                 x2_src,
                                 clk,
                                 /*divide_by=*/0,
                                 /*multiply_by=*/2,
                                 /*duty_cycle=*/0.0f,
                                 /*invert=*/true,
                                 /*combinational=*/true,
                                 sta::IntSeq{},
                                 sta::FloatSeq{},
                                 /*comment=*/"doubled+inverted",
                                 mode);
      }
    }

    // ---- set_propagated_clock [get_clocks clk_virt] ----
    // Virtual clock so the propagated flag is set on a clock without
    // disturbing clk's clock-latency entry in the Limits-tab output.
    if (auto* clk_virt = sdc->findClock("clk_virt")) {
      sta_->setPropagatedClock(clk_virt, mode);
    }

    // ---- set_clock_uncertainty 0.4 -to [get_pins ff2/D] ----
    // Pin-anchored uncertainty — surfaces in the Limits tab's
    // "Pin-Anchored Uncertainty" section.
    if (auto* ff2_d = ff2_->findITerm("D")) {
      sta::Pin* p = db_network_->dbToSta(ff2_d);
      sta_->setClockUncertainty(
          p,
          sta::SetupHoldAll::max(),
          sta_->units()->timeUnit()->userToSta(kPinUncSetup),
          sdc);
    }

    // ---- set_logic_zero [get_pins in2] ----
    sta_->setLogicValue(in2_pin, sta::LogicValue::zero, mode);

    // ---- set_group_path -name g1 -from in2 -to out1 ----
    sta_->makeGroupPath("g1",
                        /*is_default=*/false,
                        makeFromPin(net, in2_pin),
                        /*thrus=*/nullptr,
                        makeToPin(net, out1_pin),
                        /*comment=*/"",
                        sdc);

    // ---- set_clock_groups -logically_exclusive {clk} {clk_x2} ----
    if (sta::Clock* clk_x2 = sdc->findClock("clk_x2")) {
      sta::ClockGroups* le_groups
          = sta_->makeClockGroups("le_clk_clkx2",
                                  /*logically_exclusive=*/true,
                                  /*physically_exclusive=*/false,
                                  /*asynchronous=*/false,
                                  /*allow_paths=*/false,
                                  /*comment=*/"",
                                  sdc);
      auto* le1 = new sta::ClockSet;
      le1->insert(clk);
      sta_->makeClockGroup(le_groups, le1, sdc);
      auto* le2 = new sta::ClockSet;
      le2->insert(clk_x2);
      sta_->makeClockGroup(le_groups, le2, sdc);

      // ---- set_clock_groups -physically_exclusive -allow_paths
      //                        {clk_div2} {clk_x2} ----
      if (sta::Clock* clk_div2 = sdc->findClock("clk_div2")) {
        sta::ClockGroups* pe_groups
            = sta_->makeClockGroups("pe_clkdiv2_clkx2",
                                    /*logically_exclusive=*/false,
                                    /*physically_exclusive=*/true,
                                    /*asynchronous=*/false,
                                    /*allow_paths=*/true,
                                    /*comment=*/"",
                                    sdc);
        auto* pe1 = new sta::ClockSet;
        pe1->insert(clk_div2);
        sta_->makeClockGroup(pe_groups, pe1, sdc);
        auto* pe2 = new sta::ClockSet;
        pe2->insert(clk_x2);
        sta_->makeClockGroup(pe_groups, pe2, sdc);
      }
    }

    // ---- Limits: port / cell / pin scoped maxima ----
    sta::Network* sta_net = sta_->network();
    sta::Cell* top_cell = sta_net->cell(sta_net->topInstance());
    sta::Port* in1_port = sta_net->port(in1_pin);
    sta::Port* out1_port = sta_net->port(out1_pin);
    if (top_cell && in1_port && out1_port) {
      // set_max_transition 0.5 [get_ports out1] — slew uses the same
      // time scaler as periods/delays. The handler emits these
      // already scaled back into user units.
      sta_->setSlewLimit(
          out1_port,
          sta::MinMax::max(),
          static_cast<float>(
              sta_->units()->timeUnit()->userToSta(kPortSlewLim)),
          sdc);
      // set_max_transition 0.4 [current_design] (design-wide)
      sta_->setSlewLimit(
          top_cell,
          sta::MinMax::max(),
          static_cast<float>(
              sta_->units()->timeUnit()->userToSta(kDesignSlewLim)),
          sdc);
      // set_max_capacitance 0.05 [current_design] — cap scaled to
      // farads via the capacitance-unit scaler (the handler divides
      // by cap_scale before emitting, so user-unit values land back
      // unchanged in JSON).
      sta_->setCapacitanceLimit(
          top_cell,
          sta::MinMax::max(),
          static_cast<float>(
              sta_->units()->capacitanceUnit()->userToSta(kDesignCapLim)),
          sdc);
      // set_max_fanout 32 [get_ports in1]
      sta_->setFanoutLimit(in1_port, sta::MinMax::max(), kPortFanoutLim, sdc);
    }
    // set_max_capacitance 0.02 [get_pins ff1/D]
    if (auto* ff1_d = ff1_->findITerm("D")) {
      sta::Pin* p = db_network_->dbToSta(ff1_d);
      sta_->setCapacitanceLimit(
          p,
          sta::MinMax::max(),
          static_cast<float>(
              sta_->units()->capacitanceUnit()->userToSta(kPinCapLim)),
          sdc);
    }

    // set_clock_gating_check on each ICG E pin — mirrors the demo
    // structure that triggered the user-reported "kinds_total drifts
    // from 3 to 2 across kind=all/kind=clock_gate calls" bug.
    for (const char* iname : {"icg1", "icg2", "icg3"}) {
      odb::dbInst* icg = block_->findInst(iname);
      if (!icg) {
        continue;
      }
      odb::dbITerm* e_iterm = icg->findITerm("E");
      if (!e_iterm) {
        continue;
      }
      sta::Pin* e_pin = db_network_->dbToSta(e_iterm);
      if (!e_pin) {
        continue;
      }
      const float setup_margin = sta_->units()->timeUnit()->userToSta(0.10);
      const float hold_margin = sta_->units()->timeUnit()->userToSta(0.05);
      sta_->setClockGatingCheck(e_pin,
                                rf,
                                sta::SetupHold::max(),
                                setup_margin,
                                sta::LogicValue::unknown,
                                sdc);
      sta_->setClockGatingCheck(e_pin,
                                rf,
                                sta::SetupHold::min(),
                                hold_margin,
                                sta::LogicValue::unknown,
                                sdc);
    }

    sta_->updateGeneratedClks();
    // Build the timing graph so handlers that walk it (clockDomains, the
    // endpoint set used by handleSdcEndpointList) see populated state.
    sta_->ensureGraph();
    sta_->ensureLevelized();
  }

  // makeFalsePath / makeMulticyclePath / makePathDelay take ownership of the
  // ExceptionFrom / ExceptionTo (via own_pts=true on the pin set).
  static sta::ExceptionFrom* makeFromPin(sta::Network* net, sta::Pin* pin)
  {
    auto* pins = new sta::PinSet(net);
    pins->insert(pin);
    return new sta::ExceptionFrom(pins,
                                  /*clks=*/nullptr,
                                  /*insts=*/nullptr,
                                  sta::RiseFallBoth::riseFall(),
                                  /*own_pts=*/true,
                                  net);
  }
  static sta::ExceptionTo* makeToPin(sta::Network* net, sta::Pin* pin)
  {
    auto* pins = new sta::PinSet(net);
    pins->insert(pin);
    return new sta::ExceptionTo(pins,
                                /*clks=*/nullptr,
                                /*insts=*/nullptr,
                                sta::RiseFallBoth::riseFall(),
                                sta::RiseFallBoth::riseFall(),
                                /*own_pts=*/true,
                                net);
  }

  // Test constants — chosen so each appears at most once in the JSON output
  // and is unique enough to spot-check via parsed assertions.
  static constexpr float kClockPeriodNs = 10.0f;
  static constexpr float kInputDelayNs = 2.5f;
  static constexpr float kOutputDelayNs = 1.5f;
  static constexpr float kSetupUnc = 0.3f;
  static constexpr float kHoldUnc = 0.15f;
  static constexpr float kClockLatency = 0.5f;
  static constexpr float kClockInsert = 0.2f;
  static constexpr int kMcpMultiplier = 2;
  static constexpr float kPathDelayNs = 4.0f;
  // Constants for the round-2 SDC additions.
  static constexpr float kVirtClockNs = 5.0f;
  static constexpr float kPinUncSetup = 0.4f;
  static constexpr float kPortSlewLim = 0.5f;
  static constexpr float kDesignSlewLim = 0.4f;
  static constexpr float kDesignCapLim = 0.05f;
  static constexpr float kPortFanoutLim = 32.0f;
  static constexpr float kPinCapLim = 0.02f;

  sta::LibertyLibrary* library_{nullptr};
  sta::dbNetwork* db_network_{nullptr};
  odb::dbBTerm* clk_in_{nullptr};
  odb::dbBTerm* in1_{nullptr};
  odb::dbBTerm* in2_{nullptr};
  odb::dbBTerm* out1_{nullptr};
  odb::dbBTerm* out2_{nullptr};
  odb::dbInst* ff1_{nullptr};
  odb::dbInst* ff2_{nullptr};

  std::shared_ptr<TileGenerator> gen_;
  std::unique_ptr<SdcHandler> handler_;
};

// ─── Clocks tab ─────────────────────────────────────────────────────────────

TEST_F(SdcHandlerWithStaTest, ClocksListsEveryDefinedClock)
{
  auto val = parsePayload(
      handler_->handleSdcClocks(makeReq(1, WebSocketRequest::kSdcClocks)));
  const auto& obj = val.as_object();
  ASSERT_TRUE(obj.contains("clocks"));
  const auto& clocks = obj.at("clocks").as_array();
  // Fixture defines 4 clocks: primary `clk`, generated `clk_div2`,
  // generated `clk_x2`, and virtual `clk_virt`.
  EXPECT_EQ(clocks.size(), 4u);
  EXPECT_NE(findByName(clocks, "name", "clk"), nullptr);
  EXPECT_NE(findByName(clocks, "name", "clk_div2"), nullptr);
  EXPECT_NE(findByName(clocks, "name", "clk_x2"), nullptr);
  EXPECT_NE(findByName(clocks, "name", "clk_virt"), nullptr);
}

TEST_F(SdcHandlerWithStaTest, ClocksPrimaryHasCorrectPeriodAndWaveform)
{
  auto val = parsePayload(
      handler_->handleSdcClocks(makeReq(1, WebSocketRequest::kSdcClocks)));
  const auto* clk
      = findByName(val.as_object().at("clocks").as_array(), "name", "clk");
  ASSERT_NE(clk, nullptr);
  EXPECT_TRUE(nearly(asNumber(clk->at("period")), kClockPeriodNs));
  const auto& wf = clk->at("waveform").as_array();
  ASSERT_EQ(wf.size(), 2u);
  EXPECT_TRUE(nearly(asNumber(wf.at(0)), 0.0));
  EXPECT_TRUE(nearly(asNumber(wf.at(1)), kClockPeriodNs / 2));
  EXPECT_FALSE(clk->at("is_generated").as_bool());
  EXPECT_FALSE(clk->at("is_virtual").as_bool());
  EXPECT_TRUE(clk->at("master_clock").is_null());
  EXPECT_EQ(clk->at("comment").as_string(), "primary");
}

TEST_F(SdcHandlerWithStaTest, ClocksGeneratedExposesDivideByAndMaster)
{
  auto val = parsePayload(
      handler_->handleSdcClocks(makeReq(1, WebSocketRequest::kSdcClocks)));
  const auto* div2
      = findByName(val.as_object().at("clocks").as_array(), "name", "clk_div2");
  ASSERT_NE(div2, nullptr);
  EXPECT_TRUE(div2->at("is_generated").as_bool());
  ASSERT_TRUE(div2->at("divide_by").is_number());
  EXPECT_EQ(asNumber(div2->at("divide_by")), 2.0);
  ASSERT_TRUE(div2->at("master_clock").is_string());
  EXPECT_EQ(div2->at("master_clock").as_string(), "clk");
  EXPECT_TRUE(nearly(asNumber(div2->at("period")), kClockPeriodNs * 2.0));
}

TEST_F(SdcHandlerWithStaTest, ClocksClockTreeRootsByMasterChain)
{
  auto val = parsePayload(
      handler_->handleSdcClocks(makeReq(1, WebSocketRequest::kSdcClocks)));
  const auto& tree = val.as_object().at("clock_tree").as_array();
  // Both `clk` and `clk_virt` are roots (neither has a master); the
  // generated clocks `clk_div2` and `clk_x2` hang off `clk` as
  // children.
  const auto* clk_root = findByName(tree, "name", "clk");
  ASSERT_NE(clk_root, nullptr);
  const auto& clk_children = clk_root->at("children").as_array();
  EXPECT_EQ(clk_children.size(), 2u);
  EXPECT_NE(findByName(clk_children, "name", "clk_div2"), nullptr);
  EXPECT_NE(findByName(clk_children, "name", "clk_x2"), nullptr);
  // Virtual clock is its own root with no children.
  const auto* virt_root = findByName(tree, "name", "clk_virt");
  ASSERT_NE(virt_root, nullptr);
  EXPECT_TRUE(virt_root->at("children").as_array().empty());
}

TEST_F(SdcHandlerWithStaTest, ClocksUncertaintyEmittedAsSetupAndHold)
{
  auto val = parsePayload(
      handler_->handleSdcClocks(makeReq(1, WebSocketRequest::kSdcClocks)));
  const auto* clk
      = findByName(val.as_object().at("clocks").as_array(), "name", "clk");
  ASSERT_NE(clk, nullptr);
  ASSERT_TRUE(clk->at("uncertainty_setup").is_number());
  EXPECT_TRUE(nearly(asNumber(clk->at("uncertainty_setup")), kSetupUnc));
  ASSERT_TRUE(clk->at("uncertainty_hold").is_number());
  EXPECT_TRUE(nearly(asNumber(clk->at("uncertainty_hold")), kHoldUnc));
}

// ─── Modes / case analysis ──────────────────────────────────────────────────

TEST_F(SdcHandlerWithStaTest, ModesReportsCaseAndLogicSeparately)
{
  auto val = parsePayload(handler_->handleSdcClockModes(
      makeReq(1, WebSocketRequest::kSdcClockModes)));
  const auto& obj = val.as_object();
  // Fixture has set_case_analysis 0 on ff1/D.
  const auto& case_arr = obj.at("case_analysis").as_array();
  ASSERT_EQ(case_arr.size(), 1u);
  EXPECT_EQ(case_arr.at(0).as_object().at("value").as_string(), "0");
  EXPECT_NE(case_arr.at(0).as_object().at("pin").as_string().find("ff1"),
            std::string::npos);

  // Fixture has both set_logic_one (ff2/D) and set_logic_zero (in2).
  const auto& logic_arr = obj.at("logic_values").as_array();
  EXPECT_EQ(logic_arr.size(), 2u);
  bool found_one = false, found_zero = false;
  for (const auto& e : logic_arr) {
    const auto& o = e.as_object();
    const auto& v = o.at("value").as_string();
    const auto& p = o.at("pin").as_string();
    if (v == "1" && p.find("ff2") != std::string::npos) {
      found_one = true;
    } else if (v == "0" && p.find("in2") != std::string::npos) {
      found_zero = true;
    }
  }
  EXPECT_TRUE(found_one) << "set_logic_one on ff2/D not surfaced";
  EXPECT_TRUE(found_zero) << "set_logic_zero on in2 not surfaced";
}

// ─── Port delays ────────────────────────────────────────────────────────────

TEST_F(SdcHandlerWithStaTest, PortDelaysReportsInputAndOutputs)
{
  auto val = parsePayload(handler_->handleSdcPortDelays(
      makeReq(1, WebSocketRequest::kSdcPortDelays)));
  const auto& obj = val.as_object();
  const auto& arr = obj.at("port_delays").as_array();
  EXPECT_EQ(asNumber(obj.at("total")), static_cast<double>(arr.size()));
  EXPECT_GE(arr.size(), 4u);
  EXPECT_NE(findByName(arr, "port", "in1"), nullptr);
  EXPECT_NE(findByName(arr, "port", "in2"), nullptr);
  EXPECT_NE(findByName(arr, "port", "out1"), nullptr);
  EXPECT_NE(findByName(arr, "port", "out2"), nullptr);
}

TEST_F(SdcHandlerWithStaTest, PortDelaysCarryCorrectDelayValues)
{
  auto val = parsePayload(handler_->handleSdcPortDelays(
      makeReq(1, WebSocketRequest::kSdcPortDelays)));
  const auto& arr = val.as_object().at("port_delays").as_array();
  const auto* in1 = findByName(arr, "port", "in1");
  ASSERT_NE(in1, nullptr);
  EXPECT_EQ(in1->at("clock").as_string(), "clk");
  ASSERT_TRUE(in1->at("rise_max").is_number());
  EXPECT_TRUE(nearly(asNumber(in1->at("rise_max")), kInputDelayNs));

  const auto* out1 = findByName(arr, "port", "out1");
  ASSERT_NE(out1, nullptr);
  ASSERT_TRUE(out1->at("rise_max").is_number());
  EXPECT_TRUE(nearly(asNumber(out1->at("rise_max")), kOutputDelayNs));
}

TEST_F(SdcHandlerWithStaTest, PortDelaysPaginationLimitsResults)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcPortDelays);
  req.json = makeJson({}, {{"offset", 0}, {"limit", 2}});
  auto val = parsePayload(handler_->handleSdcPortDelays(req));
  EXPECT_LE(val.as_object().at("port_delays").as_array().size(), 2u);
  EXPECT_GE(asNumber(val.as_object().at("total")), 4.0);
}

// ─── Exceptions ─────────────────────────────────────────────────────────────

TEST_F(SdcHandlerWithStaTest, ExceptionsReportsAllThreeTypes)
{
  auto val = parsePayload(handler_->handleSdcExceptions(
      makeReq(1, WebSocketRequest::kSdcExceptions)));
  const auto& arr = val.as_object().at("exceptions").as_array();

  bool has_false = false;
  bool has_mcp = false;
  bool has_pd = false;
  for (const auto& e : arr) {
    const auto& obj = e.as_object();
    const auto& t = obj.at("type").as_string();
    if (t == "false_path") {
      has_false = true;
    } else if (t == "multi_cycle") {
      has_mcp = true;
      EXPECT_EQ(asNumber(obj.at("multiplier")),
                static_cast<double>(kMcpMultiplier));
    } else if (t == "path_delay") {
      has_pd = true;
      EXPECT_TRUE(nearly(asNumber(obj.at("delay")), kPathDelayNs));
    }
  }
  EXPECT_TRUE(has_false) << "false_path not emitted";
  EXPECT_TRUE(has_mcp) << "multi_cycle not emitted";
  EXPECT_TRUE(has_pd) << "path_delay not emitted";
}

// ─── Limits ─────────────────────────────────────────────────────────────────

TEST_F(SdcHandlerWithStaTest, LimitsReportsClockLatencyAndInsertion)
{
  auto val = parsePayload(
      handler_->handleSdcLimits(makeReq(1, WebSocketRequest::kSdcLimits)));
  const auto& obj = val.as_object();

  const auto& lats = obj.at("clock_latencies").as_array();
  ASSERT_EQ(lats.size(), 1u);
  const auto& lat = lats.at(0).as_object();
  EXPECT_EQ(lat.at("clock").as_string(), "clk");
  EXPECT_TRUE(nearly(asNumber(lat.at("rise_max")), kClockLatency));

  const auto& inss = obj.at("clock_insertions").as_array();
  ASSERT_EQ(inss.size(), 1u);
  const auto& ins = inss.at(0).as_object();
  EXPECT_EQ(ins.at("clock").as_string(), "clk");
  // At least one of the 8 cells (rise/fall × min/max × early/late) carries
  // the value we set.
  bool found = false;
  for (const auto& [_, v] : ins) {
    if (v.is_number() && nearly(asNumber(v), kClockInsert)) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected " << kClockInsert
                     << " somewhere in clock_insertion entry";
}

TEST_F(SdcHandlerWithStaTest, LimitsReportsClockUncertainty)
{
  auto val = parsePayload(
      handler_->handleSdcLimits(makeReq(1, WebSocketRequest::kSdcLimits)));
  const auto& arr = val.as_object().at("clock_uncertainties").as_array();
  ASSERT_GE(arr.size(), 1u);
  bool found_clk = false;
  for (const auto& e : arr) {
    const auto& o = e.as_object();
    if (o.at("clock").as_string() == "clk") {
      found_clk = true;
      EXPECT_TRUE(nearly(asNumber(o.at("setup")), kSetupUnc));
      EXPECT_TRUE(nearly(asNumber(o.at("hold")), kHoldUnc));
    }
  }
  EXPECT_TRUE(found_clk);
}

TEST_F(SdcHandlerWithStaTest, LimitsReportsDisabledTiming)
{
  // We disabled ff1/QN in applySdc — the disabled_timing array should
  // surface a pin-scope entry naming that exact pin.
  auto val = parsePayload(
      handler_->handleSdcLimits(makeReq(1, WebSocketRequest::kSdcLimits)));
  const auto& arr = val.as_object().at("disabled_timing").as_array();
  ASSERT_GE(arr.size(), 1u);
  bool found_ff1_qn = false;
  for (const auto& e : arr) {
    const auto& o = e.as_object();
    if (o.contains("scope") && o.at("scope").as_string() == "pin"
        && o.contains("name") && o.at("name").as_string() == "ff1/QN") {
      found_ff1_qn = true;
      break;
    }
  }
  EXPECT_TRUE(found_ff1_qn) << "expected disabled_timing to contain "
                               "{scope:pin, name:ff1/QN}";
}

// ─── Clock groups ───────────────────────────────────────────────────────────

TEST_F(SdcHandlerWithStaTest, ClockGroupsReportsEveryType)
{
  // Three groups in the fixture: async (clk vs clk_div2),
  // logically_exclusive (clk vs clk_x2), and physically_exclusive +
  // allow_paths (clk_div2 vs clk_x2).
  auto val = parsePayload(handler_->handleSdcClockGroups(
      makeReq(1, WebSocketRequest::kSdcClockGroups)));
  const auto& arr = val.as_object().at("groups").as_array();
  ASSERT_EQ(arr.size(), 3u);

  bool has_async = false, has_le = false, has_pe = false;
  for (const auto& g : arr) {
    const auto& o = g.as_object();
    const std::string t = std::string(o.at("type").as_string());
    EXPECT_EQ(o.at("clk_sets").as_array().size(), 2u);
    if (t == "asynchronous") {
      has_async = true;
    }
    if (t == "logically_exclusive") {
      has_le = true;
    }
    if (t == "physically_exclusive") {
      has_pe = true;
      EXPECT_TRUE(o.at("allow_paths").as_bool())
          << "pe group set with -allow_paths";
    }
  }
  EXPECT_TRUE(has_async);
  EXPECT_TRUE(has_le);
  EXPECT_TRUE(has_pe);
}

// ─── Endpoint queries ───────────────────────────────────────────────────────

TEST_F(SdcHandlerWithStaTest, EndpointResolvesPinAndReportsClockDomain)
{
  // Query out1 — a top-level output port driven by ff2/Q. It's an
  // unconstrained timing endpoint in clk's domain (we deliberately don't
  // pick a flop-D pin here because both D pins carry set_case_analysis /
  // set_logic_one in the fixture, which disables timing through them).
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpoint);
  req.json = makeJson({{"pin", "out1"}});
  auto val = parsePayload(handler_->handleSdcEndpoint(req));
  const auto& obj = val.as_object();
  EXPECT_TRUE(obj.at("found").as_bool());
  const auto& pins = obj.at("pins").as_array();
  ASSERT_EQ(pins.size(), 1u);
  const auto& pin = pins.at(0).as_object();
  EXPECT_EQ(pin.at("name").as_string(), "out1");
  EXPECT_TRUE(pin.at("is_port").as_bool()) << "out1 is a top-level port";
  // It must report the clock domain that captures it.
  ASSERT_TRUE(pin.contains("clocks"));
  const auto& clocks = pin.at("clocks").as_array();
  bool found_clk = false;
  for (const auto& c : clocks) {
    if (c.as_object().at("name").as_string() == "clk") {
      found_clk = true;
      break;
    }
  }
  EXPECT_TRUE(found_clk) << "out1 should be in clk's domain";
}

TEST_F(SdcHandlerWithStaTest, EndpointReturnsFalseForMissingPin)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpoint);
  req.json = makeJson({{"pin", "definitely_not_a_pin"}});
  auto val = parsePayload(handler_->handleSdcEndpoint(req));
  EXPECT_FALSE(val.as_object().at("found").as_bool());
}

TEST_F(SdcHandlerWithStaTest, EndpointCountsReportsPerClockTotals)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointCounts);
  auto val = parsePayload(handler_->handleSdcEndpointCounts(req));
  const auto& obj = val.as_object();
  ASSERT_TRUE(obj.contains("clocks_total"));
  const auto& counts = obj.at("clocks_total").as_object();
  // The fixture creates a clock named "clk" driving DFF_X1 endpoints,
  // so that clock must surface at least one endpoint pin in the count.
  ASSERT_TRUE(counts.contains("clk"));
  EXPECT_GE(asNumber(counts.at("clk")), 1.0);
}

TEST_F(SdcHandlerWithStaTest, EndpointListEnumeratesEndpoints)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req.json = makeJson({}, {{"offset", 0}, {"limit", 100}});
  auto val = parsePayload(handler_->handleSdcEndpointList(req));
  const auto& obj = val.as_object();
  EXPECT_GT(asNumber(obj.at("total")), 0.0);

  // The design has two DFF_X1 instances (ff1, ff2) — at least one of their
  // D pins must be classified as a flipflop endpoint.
  ASSERT_TRUE(obj.contains("kinds_total"));
  const auto& kinds = obj.at("kinds_total").as_object();
  EXPECT_GE(asNumber(kinds.at("flipflop")), 1.0)
      << "expected the flop chain to surface at least one flipflop endpoint";

  // And the endpoints array should carry an entry whose kind is "flipflop".
  bool found_flop = false;
  for (const auto& e : obj.at("endpoints").as_array()) {
    if (e.as_object().at("kind").as_string() == "flipflop") {
      found_flop = true;
      break;
    }
  }
  EXPECT_TRUE(found_flop)
      << "endpoints[] should contain at least one flipflop entry";
}

// ─── Mode list / set ────────────────────────────────────────────────────────

TEST_F(SdcHandlerWithStaTest, ListModesReportsCurrentMode)
{
  auto val = parsePayload(handler_->handleSdcListModes(
      makeReq(1, WebSocketRequest::kSdcListModes)));
  const auto& obj = val.as_object();
  EXPECT_FALSE(obj.at("current").as_string().empty());
  EXPECT_GE(obj.at("modes").as_array().size(), 1u);
}

TEST_F(SdcHandlerWithStaTest, SetModeRejectsUnknownMode)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcSetMode);
  req.json = makeJson({{"mode", "no_such_mode_xyz"}});
  auto val = parsePayload(handler_->handleSdcSetMode(req));
  const auto& obj = val.as_object();
  EXPECT_FALSE(obj.at("ok").as_bool());
  EXPECT_FALSE(obj.at("error").as_string().empty());
}

// ─── New round-2 coverage ──────────────────────────────────────────────────

// Virtual clocks don't have source pins. Surface that on the Clocks tab.
TEST_F(SdcHandlerWithStaTest, ClocksVirtualClockHasNoSources)
{
  auto val = parsePayload(
      handler_->handleSdcClocks(makeReq(1, WebSocketRequest::kSdcClocks)));
  const auto* virt
      = findByName(val.as_object().at("clocks").as_array(), "name", "clk_virt");
  ASSERT_NE(virt, nullptr);
  EXPECT_TRUE(virt->at("is_virtual").as_bool());
  EXPECT_FALSE(virt->at("is_generated").as_bool());
  EXPECT_TRUE(virt->at("master_clock").is_null());
  // Virtual clocks have an empty `sources` array — no real launch pin.
  if (virt->contains("sources")) {
    EXPECT_TRUE(virt->at("sources").as_array().empty());
  }
  EXPECT_TRUE(nearly(asNumber(virt->at("period")), kVirtClockNs));
}

// Multiplied + inverted + combinational generated clock. Each flag
// surfaces in the JSON so the Clocks tab can render the correct
// badges.
TEST_F(SdcHandlerWithStaTest, ClocksGeneratedX2ExposesEveryFlag)
{
  auto val = parsePayload(
      handler_->handleSdcClocks(makeReq(1, WebSocketRequest::kSdcClocks)));
  const auto* x2
      = findByName(val.as_object().at("clocks").as_array(), "name", "clk_x2");
  ASSERT_NE(x2, nullptr);
  EXPECT_TRUE(x2->at("is_generated").as_bool());
  EXPECT_EQ(x2->at("master_clock").as_string(), "clk");
  ASSERT_TRUE(x2->at("multiply_by").is_number());
  EXPECT_EQ(asNumber(x2->at("multiply_by")), 2.0);
  EXPECT_TRUE(x2->at("invert").as_bool());
  EXPECT_TRUE(x2->at("combinational").as_bool());
  EXPECT_EQ(x2->at("comment").as_string(), "doubled+inverted");
}

// `set_propagated_clock` flips a per-clock flag. We propagate clk_virt
// so the existing tests against `clk` (latency, etc.) keep working.
TEST_F(SdcHandlerWithStaTest, ClocksReflectsPropagatedFlag)
{
  auto val = parsePayload(
      handler_->handleSdcClocks(makeReq(1, WebSocketRequest::kSdcClocks)));
  const auto& clocks = val.as_object().at("clocks").as_array();
  // clk_virt — propagated.
  const auto* virt = findByName(clocks, "name", "clk_virt");
  ASSERT_NE(virt, nullptr);
  EXPECT_TRUE(virt->at("is_propagated").as_bool());
  // clk — NOT propagated (default ideal).
  const auto* clk = findByName(clocks, "name", "clk");
  ASSERT_NE(clk, nullptr);
  EXPECT_FALSE(clk->at("is_propagated").as_bool());
}

// Per-pin clock uncertainty surfaces as a separate Limits-tab entry.
TEST_F(SdcHandlerWithStaTest, LimitsReportsPinAnchoredUncertainty)
{
  auto val = parsePayload(
      handler_->handleSdcLimits(makeReq(1, WebSocketRequest::kSdcLimits)));
  const auto& obj = val.as_object();
  // The fixture set set_clock_uncertainty -to ff2/D. The Limits tab
  // emits this through a probe gated on the timing graph being built —
  // the fixture's ensureLevelized() in applySdc() takes care of that.
  ASSERT_TRUE(obj.contains("pin_clock_uncertainties"));
  const auto& arr = obj.at("pin_clock_uncertainties").as_array();
  bool found = false;
  for (const auto& e : arr) {
    const auto& o = e.as_object();
    if (o.at("pin").as_string().find("ff2") != std::string::npos
        && nearly(asNumber(o.at("setup")), kPinUncSetup)) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected per-pin uncertainty on ff2/D";
}

// Group path lands in the same `exceptions[]` array, alongside
// false_path / multi_cycle / path_delay.
TEST_F(SdcHandlerWithStaTest, ExceptionsReportsGroupPath)
{
  auto val = parsePayload(handler_->handleSdcExceptions(
      makeReq(1, WebSocketRequest::kSdcExceptions)));
  const auto& arr = val.as_object().at("exceptions").as_array();
  bool found_group = false;
  for (const auto& e : arr) {
    const auto& o = e.as_object();
    if (o.at("type").as_string() == "group_path") {
      found_group = true;
      EXPECT_EQ(o.at("name").as_string(), "g1");
      // -from in2 -to out1 — verify both ends made it through.
      EXPECT_TRUE(o.contains("from_pins"));
      EXPECT_TRUE(o.contains("to_pins"));
      break;
    }
  }
  EXPECT_TRUE(found_group) << "set_group_path 'g1' missing from exceptions[]";
}

// Port-scope set_max_transition.
TEST_F(SdcHandlerWithStaTest, LimitsReportsPortSlewLimit)
{
  auto val = parsePayload(
      handler_->handleSdcLimits(makeReq(1, WebSocketRequest::kSdcLimits)));
  const auto& obj = val.as_object();
  // Port-scope limits live under a "port_limits" / "max_transition_*"
  // section depending on emission shape — accept either of the
  // shapes the handler currently uses.
  bool found = false;
  for (const auto& [k, v] : obj) {
    if (!v.is_array()) {
      continue;
    }
    for (const auto& e : v.as_array()) {
      if (!e.is_object()) {
        continue;
      }
      const auto& o = e.as_object();
      auto name_it = o.find("port");
      if (name_it == o.end()) {
        name_it = o.find("name");
      }
      if (name_it == o.end()) {
        continue;
      }
      if (name_it->value().as_string() != "out1") {
        continue;
      }
      // Search every numeric child for the value we set.
      for (const auto& [_, val] : o) {
        if (val.is_number() && nearly(asNumber(val), kPortSlewLim)) {
          found = true;
          break;
        }
      }
      if (found) {
        break;
      }
    }
    if (found) {
      break;
    }
  }
  EXPECT_TRUE(found) << "expected port slew limit " << kPortSlewLim
                     << " on out1 in some Limits sub-array";
}

// Design-wide max_capacitance / max_transition / max_fanout: emitted
// under a "design_*"-style section.
TEST_F(SdcHandlerWithStaTest, LimitsReportsDesignWideLimits)
{
  auto val = parsePayload(
      handler_->handleSdcLimits(makeReq(1, WebSocketRequest::kSdcLimits)));
  // Walk every numeric in the response — at least one of the values
  // we configured (cap=0.05, slew=0.4, fanout=32) must be findable.
  // The exact section name varies by emission style; this lets the
  // test survive minor schema renames.
  std::vector<double> wanted{kDesignCapLim, kDesignSlewLim, kPortFanoutLim};
  std::vector<bool> matched(wanted.size(), false);
  std::function<void(const bj::value&)> walk = [&](const bj::value& v) {
    if (v.is_object()) {
      for (const auto& [_, x] : v.as_object()) {
        walk(x);
      }
    } else if (v.is_array()) {
      for (const auto& x : v.as_array()) {
        walk(x);
      }
    } else if (v.is_number()) {
      double n = asNumber(v);
      for (size_t i = 0; i < wanted.size(); ++i) {
        if (!matched[i] && nearly(n, wanted[i])) {
          matched[i] = true;
        }
      }
    }
  };
  walk(val);
  for (size_t i = 0; i < wanted.size(); ++i) {
    EXPECT_TRUE(matched[i])
        << "expected design-wide limit value " << wanted[i] << " in JSON";
  }
}

// Endpoint list with a `kind` filter should restrict the result set.
TEST_F(SdcHandlerWithStaTest, EndpointListKindFilterRestrictsResults)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req.json
      = makeJson({{"kind", "flipflop"}}, {{"offset", 0}, {"limit", 100}});
  auto val = parsePayload(handler_->handleSdcEndpointList(req));
  const auto& eps = val.as_object().at("endpoints").as_array();
  ASSERT_FALSE(eps.empty());
  for (const auto& e : eps) {
    EXPECT_EQ(e.as_object().at("kind").as_string(), "flipflop")
        << "kind filter must drop non-flipflop entries";
  }
}

// Endpoint list with a glob pattern should only return matching pin
// paths — verify by setting a pattern that excludes everything.
TEST_F(SdcHandlerWithStaTest, EndpointListGlobPatternFiltersResults)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req.json = makeJson({{"pattern", "no_match_anywhere_*"}},
                          {{"offset", 0}, {"limit", 100}});
  auto val = parsePayload(handler_->handleSdcEndpointList(req));
  EXPECT_EQ(asNumber(val.as_object().at("total")), 0.0);
  EXPECT_TRUE(val.as_object().at("endpoints").as_array().empty());
}

// Multi-result endpoint match: querying a pattern that hits multiple
// pins. handleSdcEndpoint reports `multi: true` and emits every
// matching pin in `pins[]`.
TEST_F(SdcHandlerWithStaTest, EndpointMultiResultEmitsAllMatches)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpoint);
  req.json = makeJson({{"pin", "ff*/Q"}});
  auto val = parsePayload(handler_->handleSdcEndpoint(req));
  const auto& obj = val.as_object();
  EXPECT_TRUE(obj.at("found").as_bool());
  // ff1/Q feeds n_q1 (an internal net) and ff2/Q drives out1 — pattern
  // `ff*/Q` should hit at least both flop output pins.
  EXPECT_GE(obj.at("pins").as_array().size(), 2u);
  EXPECT_TRUE(obj.at("multi").as_bool());
}

// Resolve generated clocks: the success path. We have two generated
// clocks (clk_div2 + clk_x2) — `updateGeneratedClks` was called in
// applySdc, so the response should show ok:true.
TEST_F(SdcHandlerWithStaTest, ResolveGenClocksSucceedsWhenSdcIsValid)
{
  auto val = parsePayload(handler_->handleSdcResolveGenClocks(
      makeReq(1, WebSocketRequest::kSdcResolveGenClocks)));
  const auto& obj = val.as_object();
  EXPECT_TRUE(obj.at("ok").as_bool());
  ASSERT_TRUE(obj.contains("resolved"));
}

// SetMode round-trip on the existing mode should succeed and echo
// the mode back. The fixture currently has the default mode only.
TEST_F(SdcHandlerWithStaTest, SetModeAcceptsCurrentMode)
{
  // List modes first to discover the current name.
  auto list = parsePayload(handler_->handleSdcListModes(
      makeReq(1, WebSocketRequest::kSdcListModes)));
  const std::string current
      = std::string(list.as_object().at("current").as_string());
  ASSERT_FALSE(current.empty());

  WebSocketRequest req = makeReq(2, WebSocketRequest::kSdcSetMode);
  req.json = makeJson({{"mode", current}});
  auto val = parsePayload(handler_->handleSdcSetMode(req));
  const auto& obj = val.as_object();
  EXPECT_TRUE(obj.at("ok").as_bool()) << "switching to current mode is a no-op";
  EXPECT_EQ(obj.at("current").as_string(), current);
}

// The fixture defines clock_div2 generated from ff1/Q. The Endpoint
// query for that source pin should report it carrying clk_div2 in
// its clocks list (clk_div2's launch domain).
TEST_F(SdcHandlerWithStaTest, EndpointResolvesGeneratedClockSource)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpoint);
  req.json = makeJson({{"pin", "ff1/Q"}});
  auto val = parsePayload(handler_->handleSdcEndpoint(req));
  ASSERT_TRUE(val.as_object().at("found").as_bool());
  const auto& pins = val.as_object().at("pins").as_array();
  ASSERT_EQ(pins.size(), 1u);
  bool found_div2 = false;
  for (const auto& c : pins.at(0).as_object().at("clocks").as_array()) {
    if (c.as_object().at("name").as_string() == "clk_div2") {
      found_div2 = true;
      break;
    }
  }
  EXPECT_TRUE(found_div2) << "ff1/Q should carry clk_div2 (its generated)";
}

// kinds_total should never overcount the total — every kind's count
// is bounded by `total` and the sum doesn't exceed it. We're lenient
// because the backend's "total" (pin count) and "kinds_total" (which
// classifies by enclosing instance) can legitimately differ when
// ports are tracked separately or when one instance has multiple
// endpoint pins.
TEST_F(SdcHandlerWithStaTest, EndpointListKindsTotalIsConsistent)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req.json = makeJson({}, {{"offset", 0}, {"limit", 1000}});
  auto val = parsePayload(handler_->handleSdcEndpointList(req));
  const auto& obj = val.as_object();
  const double total = asNumber(obj.at("total"));
  EXPECT_GT(total, 0.0);
  const auto& kinds = obj.at("kinds_total").as_object();
  for (const auto& [k, v] : kinds) {
    if (v.is_number()) {
      EXPECT_LE(asNumber(v), total)
          << "kinds_total[" << k << "] should never exceed total";
    }
  }
}

// `kinds_total` and `clocks_total` should reflect the active pattern
// filter — the frontend's chip counts are reactive to the search box,
// so the user can see "switching to this kind would give N matches"
// given their current glob. Pre-fix the tally was design-wide and
// never changed when the user typed a search.
TEST_F(SdcHandlerWithStaTest, EndpointListChipCountsApplyPattern)
{
  // Baseline: no pattern → counts reflect the whole design.
  auto baseVal = parsePayload(handler_->handleSdcEndpointList(
      makeReq(1, WebSocketRequest::kSdcEndpointList)));
  const auto& baseKinds = baseVal.as_object().at("kinds_total").as_object();
  const auto& baseClocks = baseVal.as_object().at("clocks_total").as_object();
  double baseFlop = baseKinds.contains("flipflop")
                        ? asNumber(baseKinds.at("flipflop"))
                        : 0.0;
  double baseClkSum = 0.0;
  for (const auto& [_, v] : baseClocks) {
    baseClkSum += asNumber(v);
  }
  ASSERT_GT(baseFlop, 0.0);
  ASSERT_GT(baseClkSum, 0.0);

  // Pattern that's guaranteed not to match any endpoint in the
  // fixture. Counts should collapse to 0 across the board.
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req.json = makeJson({{"pattern", "definitely_no_such_pin_xyz_*"}});
  auto val = parsePayload(handler_->handleSdcEndpointList(req));
  const auto& obj = val.as_object();
  const auto& kinds = obj.at("kinds_total").as_object();
  const auto& clocks = obj.at("clocks_total").as_object();
  double pruneFlop = kinds.contains("flipflop")
                         ? asNumber(kinds.at("flipflop"))
                         : 0.0;
  double pruneClkSum = 0.0;
  for (const auto& [_, v] : clocks) {
    pruneClkSum += asNumber(v);
  }
  EXPECT_EQ(pruneFlop, 0.0)
      << "kinds_total[flipflop] should reflect pattern filter";
  EXPECT_EQ(pruneClkSum, 0.0)
      << "clocks_total entries should reflect pattern filter";
  EXPECT_EQ(asNumber(obj.at("total")), 0.0);
}

// Pagination doesn't necessarily preserve a stable total ordering
// across calls (the underlying endpoint set is hash-ordered), but the
// total count, kinds_total, and page-size handling must be coherent.
TEST_F(SdcHandlerWithStaTest, EndpointListPaginationLimitsPageSize)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req.json = makeJson({}, {{"offset", 0}, {"limit", 1}});
  auto val = parsePayload(handler_->handleSdcEndpointList(req));
  const auto& obj = val.as_object();
  EXPECT_LE(obj.at("endpoints").as_array().size(), 1u);
  EXPECT_GE(asNumber(obj.at("total")), 1.0);
}

// Port Delays for an inout port (we don't have one in the fixture
// today, so this just sanity-checks that the key exists in the
// envelope schema). When an inout port is added later, the same
// test should naturally extend.
TEST_F(SdcHandlerWithStaTest, PortDelaysCarryDirectionField)
{
  auto val = parsePayload(handler_->handleSdcPortDelays(
      makeReq(1, WebSocketRequest::kSdcPortDelays)));
  const auto& arr = val.as_object().at("port_delays").as_array();
  // Every emitted port carries its direction so the frontend can
  // dispatch on input / output / inout.
  for (const auto& e : arr) {
    EXPECT_TRUE(e.as_object().contains("direction"))
        << "every Port Delays entry must carry `direction`";
  }
}

// ── Integrated clock-gating cell endpoints (kind=clock_gate) ──────────

TEST_F(SdcHandlerWithStaTest, EndpointListReportsClockGateKind)
{
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req.json = makeJson({}, {{"offset", 0}, {"limit", 100}});
  auto val = parsePayload(handler_->handleSdcEndpointList(req));
  const auto& obj = val.as_object();
  ASSERT_TRUE(obj.contains("kinds_total"));
  const auto& kinds = obj.at("kinds_total").as_object();
  ASSERT_TRUE(kinds.contains("clock_gate"))
      << "kinds_total must include clock_gate bucket";
  // Fixture instantiates one CLKGATE_X1 (icg1) — count is 1 (instance-
  // deduped) regardless of how many endpoint pins (CK + E) the cell
  // contributes.
  EXPECT_GE(asNumber(kinds.at("clock_gate")), 1.0);
}

// kinds_total must be stable across calls — the per-kind counts are
// computed from the full endpoint walk before the kind filter is
// applied, so swapping the filter from "all" to "clock_gate" must
// not change the reported kinds_total. A bug here was reported by
// the user as "the Clock Gate filter button shows 3, then becomes 2
// after I click it" — root-cause was subtle endpoint-set drift
// across sequential calls.
TEST_F(SdcHandlerWithStaTest, KindsTotalIsStableAcrossKindFilter)
{
  WebSocketRequest req_all = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req_all.json
      = makeJson({{"kind", "all"}}, {{"offset", 0}, {"limit", 100}});
  auto val_all = parsePayload(handler_->handleSdcEndpointList(req_all));
  const auto& kinds_all = val_all.as_object().at("kinds_total").as_object();
  const double cg_all = asNumber(kinds_all.at("clock_gate"));

  WebSocketRequest req_cg = makeReq(2, WebSocketRequest::kSdcEndpointList);
  req_cg.json
      = makeJson({{"kind", "clock_gate"}}, {{"offset", 0}, {"limit", 100}});
  auto val_cg = parsePayload(handler_->handleSdcEndpointList(req_cg));
  const auto& kinds_cg = val_cg.as_object().at("kinds_total").as_object();
  const double cg_filtered = asNumber(kinds_cg.at("clock_gate"));

  EXPECT_EQ(cg_all, cg_filtered)
      << "kinds_total.clock_gate must be identical regardless of the "
      << "request's kind filter; the per-kind tally happens before "
      << "filtering. all=" << cg_all << " clock_gate=" << cg_filtered;
  // Fixture instantiates 3 CLKGATE_X1 ICGs (icg1/icg2/icg3) each with
  // a `set_clock_gating_check` SDC override on its E pin — the
  // tally must surface all three.
  EXPECT_EQ(cg_all, 3.0)
      << "expected 3 ICGs with E-pin SDC overrides to all be tallied";
}

TEST_F(SdcHandlerWithStaTest, ClockGateEndpointEmitsFlavorAndPinRoles)
{
  // Filter the endpoint list to just clock_gate kind.
  WebSocketRequest req = makeReq(1, WebSocketRequest::kSdcEndpointList);
  req.json
      = makeJson({{"kind", "clock_gate"}}, {{"offset", 0}, {"limit", 100}});
  auto val = parsePayload(handler_->handleSdcEndpointList(req));
  const auto& eps = val.as_object().at("endpoints").as_array();
  ASSERT_FALSE(eps.empty()) << "kind=clock_gate filter should hit icg1";

  // Find the entry corresponding to our icg1 instance.
  const bj::object* icg = nullptr;
  for (const auto& e : eps) {
    const auto& o = e.as_object();
    if (o.at("kind").as_string() == "clock_gate") {
      icg = &o;
      break;
    }
  }
  ASSERT_NE(icg, nullptr);
  EXPECT_EQ(icg->at("instance").as_string(), "icg1");
  EXPECT_EQ(icg->at("cell").as_string(), "CLKGATE_X1");
  // Flavor — Nangate45's CLKGATE_X1 is latch_posedge.
  ASSERT_TRUE(icg->contains("clock_gate_flavor"));
  EXPECT_EQ(icg->at("clock_gate_flavor").as_string(), "latch_posedge");
  // Pin roles — CK / E / GCK from the cell's Liberty are mapped to
  // canonical role names so the frontend can find them by role.
  ASSERT_TRUE(icg->contains("clock_gate_ck_pin"));
  ASSERT_TRUE(icg->contains("clock_gate_en_pin"));
  ASSERT_TRUE(icg->contains("clock_gate_out_pin"));
  EXPECT_NE(icg->at("clock_gate_ck_pin").as_string().find("icg1"),
            std::string::npos);
  EXPECT_NE(icg->at("clock_gate_en_pin").as_string().find("icg1"),
            std::string::npos);
  EXPECT_NE(icg->at("clock_gate_out_pin").as_string().find("icg1"),
            std::string::npos);
}

}  // namespace
}  // namespace web
