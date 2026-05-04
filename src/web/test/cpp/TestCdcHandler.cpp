// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors
//
// Tests for the CDC web handler. Three test groupings, sharing one
// rich fixture that covers every shape of CDC path the visualiser
// surfaces:
//
//   * NullStaTest    — handlers return well-formed empty envelopes
//                      when STA is not loaded. Also exercises the
//                      whitelist round-trip + CSV edge cases (the
//                      whitelist state lives on the handler, no
//                      design needed).
//
//   * CdcWithStaTest — small Nangate45 design with three clock
//                      domains and six endpoint patterns:
//                        1. 2FF synced chain   (sync_b1 → sync_b2)
//                        2. Single-flop unsync (ff_unsynced)
//                        3. Async-excluded     (ff_excluded, clk_c)
//                        4. Comb back-walk     (AND→OR→ff_comb_b)
//                        5. Domain-mix gate    ((q_a, q_b) → AND →
//                                              ff_mix_b)
//                        6. Forward-buf sync   (buf_sync1 → BUF →
//                                              buf_sync2)
//                      Asserts the matrix totals, per-category
//                      filtering, path-detail schema for every shape,
//                      whitelist re-classification (instance + master),
//                      and pair-cache invalidation on whitelist change.

#include <algorithm>
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
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/NetworkClass.hh"
#include "sta/Sdc.hh"
#include "sta/SdcClass.hh"
#include "sta/Sequential.hh"
#include "sta/Transition.hh"
#include "sta/Units.hh"
#include "tile_generator.h"
#include "tst/nangate45_fixture.h"

namespace web {
namespace {

namespace bj = boost::json;

// ─── Shared helpers ─────────────────────────────────────────────────────────

static std::string payloadStr(const WebSocketResponse& resp)
{
  return std::string(resp.payload.begin(), resp.payload.end());
}

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

static WebSocketRequest makeReq(uint32_t id, WebSocketRequest::Type type)
{
  WebSocketRequest req;
  req.id = id;
  req.type = type;
  // Empty payload is the common default — handlers that read fields
  // assign their own makeJson() result on top.
  return req;
}

// Inline JSON-object helper. Returns boost::json::object directly so
// callers assign straight to `WebSocketRequest::json`.
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

// Parse a JSON-string literal into boost::json::object. For tests that
// build a request body with embedded escapes / dynamic substitutions
// rather than using makeJson.
static bj::object parseObj(const std::string& s)
{
  return bj::parse(s).as_object();
}

// Resolve a (launch, capture) cell out of an overview response. The
// overview is keyed by mode name; we look up whichever mode the
// backend reported as current.
static const bj::object& matrixCell(const bj::value& overview,
                                    const std::string& launch,
                                    const std::string& capture)
{
  const auto& o = overview.as_object();
  const std::string current = std::string(o.at("current_mode").as_string());
  return o.at("modes")
      .as_object()
      .at(current)
      .as_object()
      .at("matrix")
      .as_object()
      .at(launch)
      .as_object()
      .at(capture)
      .as_object();
}

// Find the first row in a `paths[]` array whose `capture_inst` matches.
// Returns nullptr when no match — callers ASSERT_NE on the result.
static const bj::object* findPath(const bj::array& paths,
                                  const std::string& capture_inst)
{
  for (const auto& p : paths) {
    const auto& o = p.as_object();
    if (o.contains("capture_inst")
        && o.at("capture_inst").as_string() == capture_inst) {
      return &o;
    }
  }
  return nullptr;
}

// Find the first stage in a `stages[]` array whose `instance` matches.
static const bj::object* findStage(const bj::array& stages,
                                   const std::string& instance)
{
  for (const auto& s : stages) {
    const auto& o = s.as_object();
    if (o.contains("instance") && o.at("instance").as_string() == instance) {
      return &o;
    }
  }
  return nullptr;
}

// ─── NullStaTest ────────────────────────────────────────────────────────────

class NullStaTest : public tst::Nangate45Fixture
{
 protected:
  void SetUp() override
  {
    block_->setDieArea(odb::Rect(0, 0, 100000, 100000));
    gen_ = std::make_shared<TileGenerator>(
        getDb(), /*sta=*/nullptr, getLogger());
    handler_ = std::make_unique<CdcHandler>(gen_);
  }
  std::shared_ptr<TileGenerator> gen_;
  std::unique_ptr<CdcHandler> handler_;
};

TEST_F(NullStaTest, OverviewReturnsEmptyEnvelope)
{
  auto resp
      = handler_->handleCdcOverview(makeReq(1, WebSocketRequest::kCdcOverview));
  EXPECT_EQ(resp.id, 1u);
  EXPECT_EQ(resp.type, WebSocketResponse::kJson);
  auto v = parsePayload(resp);
  ASSERT_TRUE(v.is_object());
  const auto& o = v.as_object();
  EXPECT_TRUE(o.contains("modes"));
  EXPECT_TRUE(o.contains("current_mode"));
  EXPECT_TRUE(o.contains("time_unit"));
  EXPECT_TRUE(o.at("modes").as_object().empty());
}

TEST_F(NullStaTest, PathsReturnsEmptyEnvelope)
{
  auto req = makeReq(2, WebSocketRequest::kCdcPaths);
  req.json
      = makeJson({{"launch_clock", "clk_a"}, {"capture_clock", "clk_b"}});
  auto resp = handler_->handleCdcPaths(req);
  auto v = parsePayload(resp);
  ASSERT_TRUE(v.is_object());
  const auto& o = v.as_object();
  EXPECT_TRUE(o.contains("paths"));
  EXPECT_TRUE(o.contains("category_total"));
  EXPECT_TRUE(o.at("paths").as_array().empty());
  EXPECT_EQ(o.at("total").as_int64(), 0);
}

TEST_F(NullStaTest, PathDetailReturnsEmptyEnvelope)
{
  auto resp = handler_->handleCdcPathDetail(
      makeReq(3, WebSocketRequest::kCdcPathDetail));
  auto v = parsePayload(resp);
  ASSERT_TRUE(v.is_object());
  const auto& o = v.as_object();
  EXPECT_TRUE(o.contains("stages"));
  EXPECT_TRUE(o.contains("sync_chain"));
  EXPECT_TRUE(o.at("stages").as_array().empty());
}

// `cdc_pin_fan_in` returns an empty `stages` array when STA is
// not loaded — the same shape the frontend can render either as
// "no further fan-in" or as "no data" without special-casing.
TEST_F(NullStaTest, PinFanInReturnsEmptyEnvelope)
{
  auto req = makeReq(4, WebSocketRequest::kCdcPinFanIn);
  req.json = makeJson(
      {{"pin_odb_type", "iterm"}, {"clock", "clk_a"}},
      {{"pin_odb_id", 0}});
  auto resp = handler_->handleCdcPinFanIn(req);
  auto v = parsePayload(resp);
  ASSERT_TRUE(v.is_object());
  const auto& o = v.as_object();
  EXPECT_TRUE(o.contains("stages"));
  EXPECT_TRUE(o.at("stages").as_array().empty());
}

// `cdc_clock_mix_trace` returns a null `tree` field when STA is not
// loaded — same end shape (no stages to render) as the fan-in
// envelope, just expressed via a single root reference.
TEST_F(NullStaTest, ClockMixTraceReturnsEmptyEnvelope)
{
  auto req = makeReq(7, WebSocketRequest::kCdcClockMixTrace);
  req.json = makeJson({{"pin_odb_type", "iterm"}},
                          {{"pin_odb_id", 0}});
  auto v = parsePayload(handler_->handleCdcClockMixTrace(req));
  ASSERT_TRUE(v.is_object());
  EXPECT_TRUE(v.as_object().at("tree").is_null());
}

// ─── Whitelist round-trip + CSV edge cases ─────────────────────────────────

TEST_F(NullStaTest, WhitelistGetEmptyByDefault)
{
  auto resp = handler_->handleCdcGetWhitelist(
      makeReq(4, WebSocketRequest::kCdcGetWhitelist));
  auto v = parsePayload(resp);
  ASSERT_TRUE(v.is_object());
  const auto& o = v.as_object();
  ASSERT_TRUE(o.contains("instance_patterns"));
  ASSERT_TRUE(o.contains("master_patterns"));
  EXPECT_TRUE(o.at("instance_patterns").as_array().empty());
  EXPECT_TRUE(o.at("master_patterns").as_array().empty());
}

TEST_F(NullStaTest, WhitelistSetThenGetRoundtrips)
{
  auto setReq = makeReq(5, WebSocketRequest::kCdcSetWhitelist);
  setReq.json = makeJson({{"instance_patterns", "u_top/u_sync,*flop*"},
                              {"master_patterns", "SYNC_FF,*MUX*"}});
  auto setResp = handler_->handleCdcSetWhitelist(setReq);
  auto sv = parsePayload(setResp);
  ASSERT_TRUE(sv.is_object());
  const auto& so = sv.as_object();
  // The set response echoes the new state.
  ASSERT_TRUE(so.contains("instance_patterns"));
  const auto& iarr = so.at("instance_patterns").as_array();
  ASSERT_EQ(iarr.size(), 2u);
  EXPECT_EQ(iarr[0].as_string(), "u_top/u_sync");
  EXPECT_EQ(iarr[1].as_string(), "*flop*");
  const auto& marr = so.at("master_patterns").as_array();
  ASSERT_EQ(marr.size(), 2u);
  EXPECT_EQ(marr[0].as_string(), "SYNC_FF");
  EXPECT_EQ(marr[1].as_string(), "*MUX*");

  // Round-trip via get.
  auto getResp = handler_->handleCdcGetWhitelist(
      makeReq(6, WebSocketRequest::kCdcGetWhitelist));
  auto gv = parsePayload(getResp);
  ASSERT_TRUE(gv.is_object());
  EXPECT_EQ(gv.as_object().at("instance_patterns").as_array().size(), 2u);
  EXPECT_EQ(gv.as_object().at("master_patterns").as_array().size(), 2u);
}

TEST_F(NullStaTest, WhitelistEmptyValuesClearTheList)
{
  // Seed the lists.
  auto setReq = makeReq(1, WebSocketRequest::kCdcSetWhitelist);
  setReq.json
      = makeJson({{"instance_patterns", "*"}, {"master_patterns", "DFF*"}});
  handler_->handleCdcSetWhitelist(setReq);

  // Empty values clear both lists.
  auto clearReq = makeReq(2, WebSocketRequest::kCdcSetWhitelist);
  clearReq.json
      = makeJson({{"instance_patterns", ""}, {"master_patterns", ""}});
  auto clearResp = handler_->handleCdcSetWhitelist(clearReq);
  auto cv = parsePayload(clearResp);
  EXPECT_TRUE(cv.as_object().at("instance_patterns").as_array().empty());
  EXPECT_TRUE(cv.as_object().at("master_patterns").as_array().empty());
}

TEST_F(NullStaTest, WhitelistCsvHandlesWhitespace)
{
  // Spaces and tabs around tokens should be trimmed.
  auto req = makeReq(1, WebSocketRequest::kCdcSetWhitelist);
  req.json = makeJson({{"instance_patterns", "  *flop* , u_top/u_sync\t"},
                           {"master_patterns", "  SYNC_FF "}});
  auto v = parsePayload(handler_->handleCdcSetWhitelist(req));
  const auto& iarr = v.as_object().at("instance_patterns").as_array();
  ASSERT_EQ(iarr.size(), 2u);
  EXPECT_EQ(iarr[0].as_string(), "*flop*");
  EXPECT_EQ(iarr[1].as_string(), "u_top/u_sync");
  const auto& marr = v.as_object().at("master_patterns").as_array();
  ASSERT_EQ(marr.size(), 1u);
  EXPECT_EQ(marr[0].as_string(), "SYNC_FF");
}

TEST_F(NullStaTest, WhitelistCsvIgnoresEmptyEntries)
{
  // Empty tokens between commas should be silently dropped.
  auto req = makeReq(1, WebSocketRequest::kCdcSetWhitelist);
  req.json = makeJson({{"instance_patterns", ",,*flop*,, ,u_top/u_sync,,"},
                           {"master_patterns", ""}});
  auto v = parsePayload(handler_->handleCdcSetWhitelist(req));
  const auto& iarr = v.as_object().at("instance_patterns").as_array();
  ASSERT_EQ(iarr.size(), 2u);
  EXPECT_EQ(iarr[0].as_string(), "*flop*");
  EXPECT_EQ(iarr[1].as_string(), "u_top/u_sync");
}

TEST_F(NullStaTest, WhitelistOnlyInstanceClearsMaster)
{
  // Seed.
  auto seed = makeReq(1, WebSocketRequest::kCdcSetWhitelist);
  seed.json
      = makeJson({{"instance_patterns", "*"}, {"master_patterns", "DFF*"}});
  handler_->handleCdcSetWhitelist(seed);
  // Sending a payload with only `instance_patterns` should overwrite
  // both lists — empty is the explicit "clear" signal for master.
  auto req = makeReq(2, WebSocketRequest::kCdcSetWhitelist);
  req.json
      = makeJson({{"instance_patterns", "*new*"}, {"master_patterns", ""}});
  auto v = parsePayload(handler_->handleCdcSetWhitelist(req));
  EXPECT_EQ(v.as_object().at("instance_patterns").as_array().size(), 1u);
  EXPECT_TRUE(v.as_object().at("master_patterns").as_array().empty());
}

// ─── Rich-design fixture: every CDC shape we render ────────────────────────

class CdcWithStaTest : public tst::Nangate45Fixture
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
    handler_ = std::make_unique<CdcHandler>(gen_);
  }

  // Build the netlist. Six endpoint patterns share three clock
  // domains so the matrix lights up with synced/unsynced/excluded
  // crossings, and the path-detail walks have something interesting
  // to enumerate:
  //
  //   1) sync_b1 → sync_b2 → ff_synced_dst    SYNCED (2FF chain)
  //   2) ff_unsynced                          UNSYNCED (single flop)
  //   3) ff_excluded                          EXCLUDED (clk_c, async)
  //   4) AND→OR→ff_comb_b                     UNSYNCED, comb back-walk
  //   5) (q_a,q_b)→AND→ff_mix_b               UNSYNCED, domain mix
  //   6) buf_sync1 → BUF → buf_sync2          SYNCED (forward-buf chain)
  void buildDesign()
  {
    const char* layer = "metal1";
    auto pinRect = [](int x) { return odb::Rect(x, 0, x + 10, 10); };
    int x = 0;
    auto bterm = [&](const char* name,
                     odb::dbIoType iot = odb::dbIoType::INPUT) {
      x += 20;
      return makeBTerm(block_,
                       name,
                       {.io_type = iot,
                        .bpins = {{.layer_name = layer, .rect = pinRect(x)}}});
    };

    clk_a_ = bterm("clk_a");
    clk_b_ = bterm("clk_b");
    clk_c_ = bterm("clk_c");
    in_a_ = bterm("in_a");
    in_b_ = bterm("in_b");
    out_synced_ = bterm("out_synced_b", odb::dbIoType::OUTPUT);
    out_unsynced_ = bterm("out_unsynced_b", odb::dbIoType::OUTPUT);
    out_excluded_ = bterm("out_excluded_c", odb::dbIoType::OUTPUT);
    out_comb_ = bterm("out_comb_b", odb::dbIoType::OUTPUT);
    out_mix_ = bterm("out_mix_b", odb::dbIoType::OUTPUT);
    out_buf_synced_ = bterm("out_buf_synced_b", odb::dbIoType::OUTPUT);

    odb::dbMaster* dff = db_->findMaster("DFF_X1");
    odb::dbMaster* and2 = db_->findMaster("AND2_X1");
    odb::dbMaster* or2 = db_->findMaster("OR2_X1");
    odb::dbMaster* buf = db_->findMaster("BUF_X1");
    ASSERT_NE(dff, nullptr);
    ASSERT_NE(and2, nullptr);
    ASSERT_NE(or2, nullptr);
    ASSERT_NE(buf, nullptr);

    int y = 0;
    auto place = [&]() {
      y += 100;
      return odb::Point(100, y);
    };
    auto inst = [&](odb::dbMaster* master,
                    const char* name,
                    std::vector<tst::InstOptions::ITermInfo> iterms) {
      return tst::Fixture::makeInst(block_,
                                    master,
                                    name,
                                    {.location = place(),
                                     .status = odb::dbPlacementStatus::PLACED,
                                     .iterms = std::move(iterms)});
    };

    // Source flops, one per launch domain.
    ff_src_a_ = inst(dff,
                     "ff_src_a",
                     {{.net_name = "in_a", .term_name = "D"},
                      {.net_name = "clk_a", .term_name = "CK"},
                      {.net_name = "q_a", .term_name = "Q"}});
    ff_src_b_ = inst(dff,
                     "ff_src_b",
                     {{.net_name = "in_b", .term_name = "D"},
                      {.net_name = "clk_b", .term_name = "CK"},
                      {.net_name = "q_b", .term_name = "Q"}});

    // (1) 2FF synced chain.
    sync_b1_ = inst(dff,
                    "sync_b1",
                    {{.net_name = "q_a", .term_name = "D"},
                     {.net_name = "clk_b", .term_name = "CK"},
                     {.net_name = "q_sync_b1", .term_name = "Q"}});
    sync_b2_ = inst(dff,
                    "sync_b2",
                    {{.net_name = "q_sync_b1", .term_name = "D"},
                     {.net_name = "clk_b", .term_name = "CK"},
                     {.net_name = "q_sync_b2", .term_name = "Q"}});
    ff_synced_dst_ = inst(dff,
                          "ff_synced_dst",
                          {{.net_name = "q_sync_b2", .term_name = "D"},
                           {.net_name = "clk_b", .term_name = "CK"},
                           {.net_name = "out_synced_b", .term_name = "Q"}});

    // (2) Single capture flop, no chain — the bug pattern.
    ff_unsynced_ = inst(dff,
                        "ff_unsynced",
                        {{.net_name = "q_a", .term_name = "D"},
                         {.net_name = "clk_b", .term_name = "CK"},
                         {.net_name = "out_unsynced_b", .term_name = "Q"}});

    // (3) Excluded by clock_groups (clk_c is async with clk_a).
    ff_excluded_ = inst(dff,
                        "ff_excluded",
                        {{.net_name = "q_a", .term_name = "D"},
                         {.net_name = "clk_c", .term_name = "CK"},
                         {.net_name = "out_excluded_c", .term_name = "Q"}});

    // (4) Comb back-walk: q_a → AND → OR → ff_comb_b.D
    inst_and_back_ = inst(and2,
                          "comb_back_and",
                          {{.net_name = "q_a", .term_name = "A1"},
                           {.net_name = "in_a", .term_name = "A2"},
                           {.net_name = "comb_back_t1", .term_name = "ZN"}});
    inst_or_back_ = inst(or2,
                         "comb_back_or",
                         {{.net_name = "comb_back_t1", .term_name = "A1"},
                          {.net_name = "q_a", .term_name = "A2"},
                          {.net_name = "comb_back_t2", .term_name = "ZN"}});
    ff_comb_b_ = inst(dff,
                      "ff_comb_b",
                      {{.net_name = "comb_back_t2", .term_name = "D"},
                       {.net_name = "clk_b", .term_name = "CK"},
                       {.net_name = "out_comb_b", .term_name = "Q"}});

    // (5) Domain mix: AND fed by clk_a launch (q_a) AND clk_b launch
    // (q_b). The actual cross-domain mix happens AT the gate.
    inst_and_mix_ = inst(and2,
                         "mix_and",
                         {{.net_name = "q_a", .term_name = "A1"},
                          {.net_name = "q_b", .term_name = "A2"},
                          {.net_name = "mix_y", .term_name = "ZN"}});
    ff_mix_b_ = inst(dff,
                     "ff_mix_b",
                     {{.net_name = "mix_y", .term_name = "D"},
                      {.net_name = "clk_b", .term_name = "CK"},
                      {.net_name = "out_mix_b", .term_name = "Q"}});

    // (6) Forward-buf sync chain: BUF between sync flops. The CDC
    // crossover is at buf_sync1.D; the forward walk silently
    // collapses sync_buf and surfaces it via passthroughs_after.
    buf_sync1_ = inst(dff,
                      "buf_sync1",
                      {{.net_name = "q_a", .term_name = "D"},
                       {.net_name = "clk_b", .term_name = "CK"},
                       {.net_name = "q_buf_inter", .term_name = "Q"}});
    sync_buf_ = inst(buf,
                     "sync_buf",
                     {{.net_name = "q_buf_inter", .term_name = "A"},
                      {.net_name = "q_buf_post", .term_name = "Z"}});
    buf_sync2_ = inst(dff,
                      "buf_sync2",
                      {{.net_name = "q_buf_post", .term_name = "D"},
                       {.net_name = "clk_b", .term_name = "CK"},
                       {.net_name = "out_buf_synced_b", .term_name = "Q"}});
  }

  void applySdc()
  {
    sta::Sdc* sdc = sta_->cmdMode()->sdc();
    auto* mode = sta_->cmdMode();
    sta::Network* net = sta_->network();

    sta::Pin* clk_a_pin = db_network_->dbToSta(clk_a_);
    sta::Pin* clk_b_pin = db_network_->dbToSta(clk_b_);
    sta::Pin* clk_c_pin = db_network_->dbToSta(clk_c_);
    sta::Pin* in_a_pin = db_network_->dbToSta(in_a_);
    sta::Pin* in_b_pin = db_network_->dbToSta(in_b_);
    ASSERT_NE(clk_a_pin, nullptr);
    ASSERT_NE(clk_b_pin, nullptr);
    ASSERT_NE(clk_c_pin, nullptr);

    const float pa = sta_->units()->timeUnit()->userToSta(10.0f);
    const float pb = sta_->units()->timeUnit()->userToSta(7.0f);
    const float pc = sta_->units()->timeUnit()->userToSta(13.0f);

    sta::PinSet pa_pins(net);
    pa_pins.insert(clk_a_pin);
    sta_->makeClock(
        "clk_a", pa_pins, false, pa, sta::FloatSeq{0.0f, pa / 2.0f}, "", mode);
    sta::PinSet pb_pins(net);
    pb_pins.insert(clk_b_pin);
    sta_->makeClock(
        "clk_b", pb_pins, false, pb, sta::FloatSeq{0.0f, pb / 2.0f}, "", mode);
    sta::PinSet pc_pins(net);
    pc_pins.insert(clk_c_pin);
    sta_->makeClock(
        "clk_c", pc_pins, false, pc, sta::FloatSeq{0.0f, pc / 2.0f}, "", mode);

    // Input delays so STA accepts the primary inputs as anchored to
    // their clocks. Used by clockDomains() during the CDC walk.
    const sta::RiseFallBoth* rf = sta::RiseFallBoth::riseFall();
    const sta::RiseFall* rise = sta::RiseFall::rise();
    const sta::MinMaxAll* mm = sta::MinMaxAll::all();
    const float oneNs = sta_->units()->timeUnit()->userToSta(1.0f);
    sta_->setInputDelay(in_a_pin,
                        rf,
                        sdc->findClock("clk_a"),
                        rise,
                        nullptr,
                        false,
                        false,
                        mm,
                        false,
                        oneNs,
                        sdc);
    if (in_b_pin) {
      sta_->setInputDelay(in_b_pin,
                          rf,
                          sdc->findClock("clk_b"),
                          rise,
                          nullptr,
                          false,
                          false,
                          mm,
                          false,
                          oneNs,
                          sdc);
    }

    // set_clock_groups -asynchronous { clk_a } { clk_c } so the clk_a
    // → clk_c row classifies as `excluded` (not unsynced).
    sta::ClockGroups* groups
        = sta_->makeClockGroups("async_a_c",
                                /*logically_exclusive=*/false,
                                /*physically_exclusive=*/false,
                                /*asynchronous=*/true,
                                /*allow_paths=*/false,
                                /*comment=*/"",
                                sdc);
    {
      auto* g_a = new sta::ClockSet;
      g_a->insert(sdc->findClock("clk_a"));
      sta_->makeClockGroup(groups, g_a, sdc);
      auto* g_c = new sta::ClockSet;
      g_c->insert(sdc->findClock("clk_c"));
      sta_->makeClockGroup(groups, g_c, sdc);
    }

    sta_->ensureGraph();
    sta_->ensureLevelized();
  }

  // Convenience: get the {synced, unsynced, excluded, paths} cell for
  // a given (launch, capture) pair from a fresh overview.
  bj::value getOverview()
  {
    return parsePayload(handler_->handleCdcOverview(
        makeReq(100, WebSocketRequest::kCdcOverview)));
  }
  bj::value getPaths(const std::string& launch,
                     const std::string& capture,
                     const std::string& category = "all")
  {
    auto req = makeReq(101, WebSocketRequest::kCdcPaths);
    req.json = makeJson({{"launch_clock", launch},
                             {"capture_clock", capture},
                             {"category", category}});
    return parsePayload(handler_->handleCdcPaths(req));
  }
  // Resolve `instance/term` to the dbITerm's ODB id so the path-detail
  // request can find it.
  int itermId(odb::dbInst* inst, const char* term)
  {
    auto* it = inst->findITerm(term);
    return it ? static_cast<int>(it->getId()) : -1;
  }
  bj::value getPathDetail(odb::dbInst* capture_inst,
                          const char* d_term,
                          const std::string& launch_clock)
  {
    auto req = makeReq(102, WebSocketRequest::kCdcPathDetail);
    req.json = makeJson(
        {{"capture_odb_type", "iterm"}, {"launch_clock", launch_clock}},
        {{"capture_odb_id", itermId(capture_inst, d_term)}});
    return parsePayload(handler_->handleCdcPathDetail(req));
  }

  sta::LibertyLibrary* library_{nullptr};
  sta::dbNetwork* db_network_{nullptr};
  odb::dbBTerm* clk_a_{nullptr};
  odb::dbBTerm* clk_b_{nullptr};
  odb::dbBTerm* clk_c_{nullptr};
  odb::dbBTerm* in_a_{nullptr};
  odb::dbBTerm* in_b_{nullptr};
  odb::dbBTerm* out_synced_{nullptr};
  odb::dbBTerm* out_unsynced_{nullptr};
  odb::dbBTerm* out_excluded_{nullptr};
  odb::dbBTerm* out_comb_{nullptr};
  odb::dbBTerm* out_mix_{nullptr};
  odb::dbBTerm* out_buf_synced_{nullptr};
  odb::dbInst* ff_src_a_{nullptr};
  odb::dbInst* ff_src_b_{nullptr};
  odb::dbInst* sync_b1_{nullptr};
  odb::dbInst* sync_b2_{nullptr};
  odb::dbInst* ff_synced_dst_{nullptr};
  odb::dbInst* ff_unsynced_{nullptr};
  odb::dbInst* ff_excluded_{nullptr};
  odb::dbInst* inst_and_back_{nullptr};
  odb::dbInst* inst_or_back_{nullptr};
  odb::dbInst* ff_comb_b_{nullptr};
  odb::dbInst* inst_and_mix_{nullptr};
  odb::dbInst* ff_mix_b_{nullptr};
  odb::dbInst* buf_sync1_{nullptr};
  odb::dbInst* sync_buf_{nullptr};
  odb::dbInst* buf_sync2_{nullptr};

  std::shared_ptr<TileGenerator> gen_;
  std::unique_ptr<CdcHandler> handler_;
};

// ─── Overview matrix ───────────────────────────────────────────────────────

// clk_a → clk_b cell should hold every CDC pair landing on a clk_b
// flop with a clk_a launch: 2 synced (sync_b1, buf_sync1), 3 unsynced
// (ff_unsynced, ff_comb_b, ff_mix_b). 5 total.
TEST_F(CdcWithStaTest, OverviewClkAtoClkBCellTotals)
{
  auto v = getOverview();
  const auto& cell = matrixCell(v, "clk_a", "clk_b");
  EXPECT_EQ(cell.at("paths").as_int64(), 5);
  EXPECT_EQ(cell.at("synced").as_int64(), 2);
  EXPECT_EQ(cell.at("unsynced").as_int64(), 3);
  EXPECT_EQ(cell.at("excluded").as_int64(), 0);
}

// clk_a → clk_c cell should hold the 1 excluded pair (ff_excluded).
TEST_F(CdcWithStaTest, OverviewExcludedReflectsClockGroups)
{
  auto v = getOverview();
  const auto& cell = matrixCell(v, "clk_a", "clk_c");
  EXPECT_EQ(cell.at("paths").as_int64(), 1);
  EXPECT_EQ(cell.at("synced").as_int64(), 0);
  EXPECT_EQ(cell.at("unsynced").as_int64(), 0);
  EXPECT_EQ(cell.at("excluded").as_int64(), 1);
}

// clk_b → clk_a row has no flops launching from clk_b targeting clk_a.
// Either the cell is missing (sparse storage) or zeroed out.
TEST_F(CdcWithStaTest, OverviewNoReversePairs)
{
  auto v = getOverview();
  const std::string current
      = std::string(v.as_object().at("current_mode").as_string());
  const auto& matrix = v.as_object()
                           .at("modes")
                           .as_object()
                           .at(current)
                           .as_object()
                           .at("matrix")
                           .as_object();
  if (matrix.contains("clk_b")
      && matrix.at("clk_b").as_object().contains("clk_a")) {
    const auto& cell = matrix.at("clk_b").as_object().at("clk_a").as_object();
    EXPECT_EQ(cell.at("paths").as_int64(), 0);
  }
}

TEST_F(CdcWithStaTest, OverviewListsAllThreeClocks)
{
  auto v = getOverview();
  const std::string current
      = std::string(v.as_object().at("current_mode").as_string());
  const auto& clocks = v.as_object()
                           .at("modes")
                           .as_object()
                           .at(current)
                           .as_object()
                           .at("clocks")
                           .as_array();
  // Every clock with at least one endpoint pin in its domain is in
  // the list — clk_a, clk_b, clk_c all have flops.
  std::vector<std::string> names;
  for (const auto& c : clocks) {
    names.emplace_back(c.as_string());
  }
  EXPECT_NE(std::find(names.begin(), names.end(), "clk_a"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "clk_b"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "clk_c"), names.end());
}

// ─── Path-list filtering and pagination ────────────────────────────────────

TEST_F(CdcWithStaTest, PathsAllReturnsEverythingForCell)
{
  auto v = getPaths("clk_a", "clk_b", "all");
  EXPECT_EQ(v.as_object().at("total").as_int64(), 5);
  EXPECT_EQ(v.as_object().at("paths").as_array().size(), 5u);
  // category_total reports per-bucket counts.
  const auto& ct = v.as_object().at("category_total").as_object();
  EXPECT_EQ(ct.at("synced").as_int64(), 2);
  EXPECT_EQ(ct.at("unsynced").as_int64(), 3);
  EXPECT_EQ(ct.at("excluded").as_int64(), 0);
}

TEST_F(CdcWithStaTest, PathsSyncedFilterReturnsOnlySyncedRows)
{
  auto v = getPaths("clk_a", "clk_b", "synchronized");
  const auto& paths = v.as_object().at("paths").as_array();
  ASSERT_EQ(paths.size(), 2u);
  for (const auto& p : paths) {
    EXPECT_EQ(p.as_object().at("category").as_string(), "synchronized");
  }
  // sync_b1 and buf_sync1 are the two synced captures.
  EXPECT_NE(findPath(paths, "sync_b1"), nullptr);
  EXPECT_NE(findPath(paths, "buf_sync1"), nullptr);
}

TEST_F(CdcWithStaTest, PathsUnsyncedFilterReturnsOnlyBugRows)
{
  auto v = getPaths("clk_a", "clk_b", "unsynchronized");
  const auto& paths = v.as_object().at("paths").as_array();
  ASSERT_EQ(paths.size(), 3u);
  for (const auto& p : paths) {
    EXPECT_EQ(p.as_object().at("category").as_string(), "unsynchronized");
    EXPECT_EQ(p.as_object().at("sync_chain_kind").as_string(), "none");
  }
  EXPECT_NE(findPath(paths, "ff_unsynced"), nullptr);
  EXPECT_NE(findPath(paths, "ff_comb_b"), nullptr);
  EXPECT_NE(findPath(paths, "ff_mix_b"), nullptr);
}

TEST_F(CdcWithStaTest, PathsExcludedFilterReturnsAsyncGroupedRows)
{
  auto v = getPaths("clk_a", "clk_c", "excluded");
  const auto& paths = v.as_object().at("paths").as_array();
  ASSERT_EQ(paths.size(), 1u);
  EXPECT_EQ(paths[0].as_object().at("category").as_string(), "excluded");
  EXPECT_EQ(paths[0].as_object().at("capture_inst").as_string(), "ff_excluded");
}

TEST_F(CdcWithStaTest, PathsRowSchemaCarriesCaptureOdbAndClocks)
{
  auto v = getPaths("clk_a", "clk_b", "synchronized");
  const auto& paths = v.as_object().at("paths").as_array();
  ASSERT_FALSE(paths.empty());
  const auto* p = findPath(paths, "sync_b1");
  ASSERT_NE(p, nullptr);
  EXPECT_EQ(p->at("odb_type").as_string(), "iterm");
  EXPECT_GE(p->at("odb_id").as_int64(), 0);
  EXPECT_EQ(p->at("launch_clock").as_string(), "clk_a");
  EXPECT_EQ(p->at("capture_clock").as_string(), "clk_b");
  EXPECT_EQ(p->at("sync_chain_kind").as_string(), "ff_chain");
  EXPECT_GE(p->at("sync_chain_depth").as_int64(), 2);
}

TEST_F(CdcWithStaTest, PathsPaginationOffsetLimitTrimsResults)
{
  // Limit 2 — first batch only returns 2 rows; offset 2 returns the rest.
  auto req1 = makeReq(1, WebSocketRequest::kCdcPaths);
  req1.json = makeJson({{"launch_clock", "clk_a"},
                            {"capture_clock", "clk_b"},
                            {"category", "all"}},
                           {{"offset", 0}, {"limit", 2}});
  auto v1 = parsePayload(handler_->handleCdcPaths(req1));
  EXPECT_EQ(v1.as_object().at("total").as_int64(), 5);
  EXPECT_EQ(v1.as_object().at("paths").as_array().size(), 2u);

  auto req2 = makeReq(2, WebSocketRequest::kCdcPaths);
  req2.json = makeJson({{"launch_clock", "clk_a"},
                            {"capture_clock", "clk_b"},
                            {"category", "all"}},
                           {{"offset", 2}, {"limit", 100}});
  auto v2 = parsePayload(handler_->handleCdcPaths(req2));
  EXPECT_EQ(v2.as_object().at("paths").as_array().size(), 3u);
}

TEST_F(CdcWithStaTest, PathsForUnknownPairReturnsEmpty)
{
  // clk_b → clk_a has no CDC rows — request returns total=0 and an
  // empty array (not a missing-pair error).
  auto v = getPaths("clk_b", "clk_a", "all");
  EXPECT_EQ(v.as_object().at("total").as_int64(), 0);
  EXPECT_TRUE(v.as_object().at("paths").as_array().empty());
}

// `pattern` glob narrows the result set to rows whose capture-pin
// path matches the pattern. category_total still reflects the
// FULL bucket counts (deliberate — see the comment in
// handleCdcPaths) so the user sees "after my filter, 1 of N rows".
TEST_F(CdcWithStaTest, PathsPatternFiltersByCapturePin)
{
  auto req = makeReq(1, WebSocketRequest::kCdcPaths);
  // Only the unsynced sink ff_unsynced has "unsynced" in its path —
  // the synced and combinational variants don't.
  req.json = makeJson({{"launch_clock", "clk_a"},
                           {"capture_clock", "clk_b"},
                           {"category", "all"},
                           {"pattern", "*unsynced*"}});
  auto v = parsePayload(handler_->handleCdcPaths(req));
  const auto& paths = v.as_object().at("paths").as_array();
  EXPECT_EQ(v.as_object().at("total").as_int64(),
            static_cast<int64_t>(paths.size()));
  EXPECT_FALSE(paths.empty()) << "expected at least one '*unsynced*' row";
  for (const auto& p : paths) {
    const auto& name = p.as_object().at("capture_pin").as_string();
    EXPECT_NE(std::string(name).find("unsynced"), std::string::npos)
        << "every returned row's capture_pin should match the glob: " << name;
  }
  // category_total uses the pre-pattern population so the user can
  // see how aggressive their pattern is.
  const auto& ct = v.as_object().at("category_total").as_object();
  EXPECT_EQ(ct.at("synced").as_int64(), 2)
      << "category_total tally must be the full clk_a → clk_b cell, "
      << "not narrowed by the pattern";
  EXPECT_EQ(ct.at("unsynced").as_int64(), 3);
}

// Pattern that matches nothing → empty result set, category_total
// still surfaces the full population so the user sees their filter
// is too tight rather than that there's nothing to match.
TEST_F(CdcWithStaTest, PathsPatternEmptyResultStillReportsCategoryTotal)
{
  auto req = makeReq(1, WebSocketRequest::kCdcPaths);
  req.json = makeJson({{"launch_clock", "clk_a"},
                           {"capture_clock", "clk_b"},
                           {"category", "all"},
                           {"pattern", "*no_such_pin*"}});
  auto v = parsePayload(handler_->handleCdcPaths(req));
  EXPECT_EQ(v.as_object().at("total").as_int64(), 0);
  EXPECT_TRUE(v.as_object().at("paths").as_array().empty());
  const auto& ct = v.as_object().at("category_total").as_object();
  EXPECT_GT(ct.at("synced").as_int64() + ct.at("unsynced").as_int64()
                + ct.at("excluded").as_int64(),
            0);
}

// ─── Per-pin fan-in expansion ──────────────────────────────────────────────

// `cdc_pin_fan_in` walks back from a pin in the fan-in cone and
// returns the ancestor stages. Used by the path-detail UI when a
// user clicks a clock chip on a pin to trace where THAT clock's
// signal arrives from. This test pins down the contract: the
// trace from the domain-mix gate's clk_b-side input lands on the
// clk_b source flop (ff_src_b) as a launch-tagged register stage.
TEST_F(CdcWithStaTest, PinFanInFromMixGateInputLandsOnSourceFlop)
{
  // Resolve the A2 input of the mix AND gate — the clk_b-tied side.
  odb::dbITerm* a2 = inst_and_mix_->findITerm("A2");
  ASSERT_NE(a2, nullptr);
  auto req = makeReq(1, WebSocketRequest::kCdcPinFanIn);
  req.json = makeJson({{"pin_odb_type", "iterm"},
                           {"clock", "clk_b"}},
                          {{"pin_odb_id", static_cast<int>(a2->getId())}});
  auto v = parsePayload(handler_->handleCdcPinFanIn(req));
  ASSERT_TRUE(v.is_object());
  ASSERT_TRUE(v.as_object().contains("stages"));
  const auto& stages = v.as_object().at("stages").as_array();
  ASSERT_FALSE(stages.empty()) << "fan-in trace must surface at least "
                               << "the source flop";
  // The terminal stage should be the source flop, marked as a
  // launch register. Direct net (q_b) has no buf/inv between
  // ff_src_b/Q and mix_and/A2, so it's a single-stage trace.
  const auto& last = stages.back().as_object();
  EXPECT_EQ(last.at("instance").as_string(), "ff_src_b");
  EXPECT_EQ(last.at("kind").as_string(), "register");
  EXPECT_TRUE(last.at("is_launch").as_bool());
  EXPECT_EQ(last.at("clock").as_string(), "clk_b");
}

// Missing pin_odb_id (defaults to -1 in the handler) returns an
// empty stages array — the frontend renders that as "no further
// fan-in" rather than erroring out, so the contract has to be a
// well-formed empty envelope. Out-of-range ODB ids are not
// covered here because `dbITerm::getITerm` doesn't bounds-check
// in all builds; same caveat applies to the path-detail handler.
TEST_F(CdcWithStaTest, PinFanInMissingIdReturnsEmpty)
{
  auto req = makeReq(1, WebSocketRequest::kCdcPinFanIn);
  // No pin_odb_id field — extract_int_or returns -1.
  req.json = makeJson({{"pin_odb_type", "iterm"},
                           {"clock", "clk_a"}});
  auto v = parsePayload(handler_->handleCdcPinFanIn(req));
  ASSERT_TRUE(v.is_object());
  EXPECT_TRUE(v.as_object().at("stages").as_array().empty());
}

// Strict-clock-match guard: if the user asks for a clock that
// doesn't propagate through any input of a multi-input gate, the
// walk MUST stop rather than fall back to input 0. The
// path-detail walker uses fall-back behaviour to keep diagrams
// non-empty; the fan-in RPC uses strict matching so a request
// for clk_C on a gate whose inputs are clk_A / clk_B doesn't
// silently report ff_src_a as "the source for clk_C". User
// repro: clicking four bogus clock chips on the same gate
// produced four expansions all converging on the same input-0
// register.
TEST_F(CdcWithStaTest, PinFanInStopsWhenRequestedClockNotPresent)
{
  odb::dbITerm* d = ff_mix_b_->findITerm("D");
  ASSERT_NE(d, nullptr);
  auto req = makeReq(1, WebSocketRequest::kCdcPinFanIn);
  // mix_and inputs are clk_a (q_a) and clk_b (q_b). Request clk_c
  // — neither input carries it, so the walk must produce no
  // stages (or stop at the first non-matching gate without
  // reporting a flop in the wrong domain).
  req.json = makeJson({{"pin_odb_type", "iterm"},
                           {"clock", "clk_c"}},
                          {{"pin_odb_id", static_cast<int>(d->getId())}});
  auto v = parsePayload(handler_->handleCdcPinFanIn(req));
  ASSERT_TRUE(v.is_object());
  const auto& stages = v.as_object().at("stages").as_array();
  for (const auto& st : stages) {
    const auto& o = st.as_object();
    if (o.at("kind").as_string() == "register") {
      // The only acceptable register stage would be one in clk_c
      // — but in this fixture there's no clk_c source feeding
      // mix_and, so the walk shouldn't surface ff_src_a or
      // ff_src_b as "the clk_c source".
      const std::string inst = std::string(o.at("instance").as_string());
      EXPECT_NE(inst, "ff_src_a")
          << "must not pick a clk_a register when clk_c was "
          << "requested";
      EXPECT_NE(inst, "ff_src_b")
          << "must not pick a clk_b register when clk_c was "
          << "requested";
    }
  }
}

// Walking back from the *capture* flop's D pin should produce
// the same launch-side back-chain that `cdc_path_detail` already
// emits — i.e. the new RPC is consistent with the existing
// path-detail walker. This test pins down that consistency so
// future refactors keep the two views in sync.
TEST_F(CdcWithStaTest, PinFanInMatchesPathDetailLaunchSide)
{
  odb::dbITerm* d = ff_mix_b_->findITerm("D");
  ASSERT_NE(d, nullptr);
  auto req = makeReq(1, WebSocketRequest::kCdcPinFanIn);
  req.json = makeJson({{"pin_odb_type", "iterm"},
                           {"clock", "clk_a"}},
                          {{"pin_odb_id", static_cast<int>(d->getId())}});
  auto fan_in = parsePayload(handler_->handleCdcPinFanIn(req))
                    .as_object().at("stages").as_array();
  // The path-detail back-chain for the same crossing should be
  // a prefix of the same set: in this fixture the mix gate sits
  // directly between source flops and the capture flop, so the
  // fan-in trace from D walks: mix_and (comb) → ff_src_a
  // (when clock=clk_a) since the mix gate's clk_a-side input
  // matches the requested clock for input picking.
  ASSERT_GE(fan_in.size(), 2u);
  // Walk emits stages in temporal order: stages[0] is the deepest
  // ancestor (the source flop), the final entry is the closest-
  // to-the-cursor (the gate just upstream of the queried pin).
  const auto& launch = fan_in.front().as_object();
  const auto& comb = fan_in.back().as_object();
  EXPECT_EQ(launch.at("instance").as_string(), "ff_src_a");
  EXPECT_EQ(launch.at("kind").as_string(), "register");
  EXPECT_EQ(comb.at("instance").as_string(), "mix_and");
  EXPECT_EQ(comb.at("kind").as_string(), "comb");
}

// ─── Clock-mix tracer ──────────────────────────────────────────────────────

// Helper: recursively count nodes of `kind` anywhere in a tree
// rooted at `n`. Used to assert "the walk found exactly one mixer"
// or "no mixer was emitted on a single-clock walk".
static int countNodes(const bj::value& n, const std::string& kind)
{
  if (!n.is_object()) {
    return 0;
  }
  const auto& o = n.as_object();
  int total = 0;
  if (o.contains("kind") && o.at("kind").as_string() == kind) {
    total += 1;
  }
  if (o.contains("branches")) {
    for (const auto& br : o.at("branches").as_array()) {
      if (br.as_object().contains("subtree")) {
        total += countNodes(br.as_object().at("subtree"), kind);
      }
    }
  }
  if (o.contains("child")) {
    total += countNodes(o.at("child"), kind);
  }
  return total;
}

// Helper: locate first mixer node in a tree rooted at `n`. Returns
// nullptr when the tree contains no mixer (e.g. single-clock walk).
static const bj::object* findFirstMixer(const bj::value& n)
{
  if (!n.is_object()) {
    return nullptr;
  }
  const auto& o = n.as_object();
  if (o.contains("kind") && o.at("kind").as_string() == "mixer") {
    return &o;
  }
  if (o.contains("branches")) {
    for (const auto& br : o.at("branches").as_array()) {
      if (br.as_object().contains("subtree")) {
        if (auto* found
            = findFirstMixer(br.as_object().at("subtree"))) {
          return found;
        }
      }
    }
  }
  if (o.contains("child")) {
    if (auto* found = findFirstMixer(o.at("child"))) {
      return found;
    }
  }
  return nullptr;
}

// `cdc_clock_mix_trace` walks upstream from a multi-clock pin and
// returns a TREE rooted at the originating pin's first stage. The
// fixture's mix_and AND2 has clk_a on A1 and clk_b on A2, so a walk
// from ff_mix_b/D (which carries both via mix_and's output) must:
//
//   - emit a mixer node at mix_and (≥2 contributing inputs),
//   - hang one branch per contributing input off that mixer.
//
// BFS replaces the earlier greedy single-path walker so muxes that
// live on non-followed input paths show up automatically (user
// feedback 2026-05-02 round 4).
TEST_F(CdcWithStaTest, ClockMixFromMixedDPinFindsMixerGate)
{
  odb::dbITerm* d = ff_mix_b_->findITerm("D");
  ASSERT_NE(d, nullptr);
  auto req = makeReq(1, WebSocketRequest::kCdcClockMixTrace);
  req.json = makeJson({{"pin_odb_type", "iterm"}},
                          {{"pin_odb_id", static_cast<int>(d->getId())}});
  auto v = parsePayload(handler_->handleCdcClockMixTrace(req));
  ASSERT_TRUE(v.is_object());
  ASSERT_TRUE(v.as_object().contains("tree"));
  const auto& tree = v.as_object().at("tree");
  ASSERT_TRUE(tree.is_object()) << "non-empty walk returns a tree";
  const auto* mixer = findFirstMixer(tree);
  ASSERT_NE(mixer, nullptr) << "BFS reaches mix_and as a mixer";
  EXPECT_EQ(mixer->at("instance").as_string(), "mix_and");
  EXPECT_EQ(mixer->at("kind").as_string(), "mixer");
  EXPECT_FALSE(mixer->at("on_clock_path").as_bool())
      << "walk from D pin starts on the data path";
  // Two branches, one carrying clk_a (A1), the other clk_b (A2).
  const auto& branches = mixer->at("branches").as_array();
  ASSERT_EQ(branches.size(), 2u);
  std::set<std::string> seen_clocks;
  for (const auto& br : branches) {
    const auto& cks = br.as_object().at("clocks").as_array();
    ASSERT_EQ(cks.size(), 1u);
    seen_clocks.insert(std::string(cks[0].as_string()));
  }
  EXPECT_TRUE(seen_clocks.count("clk_a") > 0);
  EXPECT_TRUE(seen_clocks.count("clk_b") > 0);
}

// Single-clock start pin: there is no mix to find. The walker still
// builds a tree (back-walking through transits / registers / port),
// but no node anywhere in the tree should be of kind="mixer".
TEST_F(CdcWithStaTest, ClockMixOnSingleClockPinHasNoMixerStages)
{
  odb::dbITerm* a1 = inst_and_mix_->findITerm("A1");
  ASSERT_NE(a1, nullptr);
  auto req = makeReq(1, WebSocketRequest::kCdcClockMixTrace);
  req.json = makeJson({{"pin_odb_type", "iterm"}},
                          {{"pin_odb_id", static_cast<int>(a1->getId())}});
  auto v = parsePayload(handler_->handleCdcClockMixTrace(req));
  ASSERT_TRUE(v.is_object());
  const auto& tree = v.as_object().at("tree");
  EXPECT_EQ(countNodes(tree, "mixer"), 0)
      << "a single-clock walk should never emit a mixer node";
}

// Regression: a multi-input cell whose inputs ALL carry the same
// (super-)set of cur_clocks is a pass-through, not a mixer. The
// fixture's comb_back_or has both inputs in {clk_a}; walking with
// {clk_a} must traverse it as a comb_transit, not stop and call it
// a "2-clock mixer" (user-reported regression 2026-05-02 round 2;
// pinned here against future regressions).
TEST_F(CdcWithStaTest, ClockMixTreatsRedundantClockRouteAsTransit)
{
  odb::dbITerm* d = ff_comb_b_->findITerm("D");
  ASSERT_NE(d, nullptr);
  auto req = makeReq(1, WebSocketRequest::kCdcClockMixTrace);
  req.json = makeJson({{"pin_odb_type", "iterm"}},
                          {{"pin_odb_id", static_cast<int>(d->getId())}});
  auto v = parsePayload(handler_->handleCdcClockMixTrace(req));
  ASSERT_TRUE(v.is_object());
  const auto& tree = v.as_object().at("tree");
  EXPECT_EQ(countNodes(tree, "mixer"), 0)
      << "redundant-clock-route cells must traverse as transit";
  // Both comb_back_or and comb_back_and should appear as
  // comb_transit nodes.
  std::function<std::set<std::string>(const bj::value&)> transitInsts;
  transitInsts = [&](const bj::value& n) -> std::set<std::string> {
    std::set<std::string> out;
    if (!n.is_object()) {
      return out;
    }
    const auto& o = n.as_object();
    if (o.contains("kind")
        && o.at("kind").as_string() == "comb_transit"
        && o.contains("instance") && o.at("instance").is_string()) {
      out.insert(std::string(o.at("instance").as_string()));
    }
    if (o.contains("branches")) {
      for (const auto& br : o.at("branches").as_array()) {
        if (br.as_object().contains("subtree")) {
          auto sub = transitInsts(br.as_object().at("subtree"));
          out.insert(sub.begin(), sub.end());
        }
      }
    }
    if (o.contains("child")) {
      auto sub = transitInsts(o.at("child"));
      out.insert(sub.begin(), sub.end());
    }
    return out;
  };
  auto names = transitInsts(tree);
  EXPECT_TRUE(names.count("comb_back_or") > 0)
      << "comb_back_or rendered as transit";
  EXPECT_TRUE(names.count("comb_back_and") > 0)
      << "comb_back_and rendered as transit";
}

// Missing pin id — handler returns the canonical null-tree
// envelope that the frontend treats as "no upstream driver".
TEST_F(CdcWithStaTest, ClockMixMissingIdReturnsEmpty)
{
  auto req = makeReq(1, WebSocketRequest::kCdcClockMixTrace);
  req.json = makeJson({{"pin_odb_type", "iterm"}});
  auto v = parsePayload(handler_->handleCdcClockMixTrace(req));
  ASSERT_TRUE(v.is_object());
  EXPECT_TRUE(v.as_object().at("tree").is_null());
}

// ─── Path detail (per-shape coverage) ──────────────────────────────────────

// Bug pattern: capture flop only, no sync chain. The back-walk hits
// ff_src_a directly (no comb). Stages: launch + capture.
TEST_F(CdcWithStaTest, PathDetailUnsyncedHasLaunchAndCrossover)
{
  auto v = getPathDetail(ff_unsynced_, "D", "clk_a");
  const auto& sc = v.as_object().at("sync_chain").as_object();
  EXPECT_EQ(sc.at("kind").as_string(), "none");

  const auto& stages = v.as_object().at("stages").as_array();
  ASSERT_GE(stages.size(), 2u);
  // First stage is the launch flop.
  EXPECT_EQ(stages[0].as_object().at("kind").as_string(), "register");
  EXPECT_EQ(stages[0].as_object().at("is_launch").as_bool(), true);
  EXPECT_EQ(stages[0].as_object().at("instance").as_string(), "ff_src_a");
  // The crossover flop is somewhere in the array.
  const auto* cap = findStage(stages, "ff_unsynced");
  ASSERT_NE(cap, nullptr);
  EXPECT_EQ(cap->at("is_capture").as_bool(), true);
  EXPECT_EQ(cap->at("kind").as_string(), "register");
}

// Every register stage carries `ck_pin` + `ck_pin_clocks`.
// Single-clock CK (the common case) emits a one-element array;
// multi-clock CK (clock-mux'd flops) emits N — that's the
// affordance the frontend uses to render the "⚠ multi-clock CK"
// warning. Pinning the schema here so a backend refactor can't
// silently drop the field.
TEST_F(CdcWithStaTest, PathDetailRegisterStagesCarryCkPinClocks)
{
  auto v = getPathDetail(ff_unsynced_, "D", "clk_a");
  const auto& stages = v.as_object().at("stages").as_array();
  ASSERT_GE(stages.size(), 2u);
  // Launch flop: clk_a only.
  const auto& launch = stages[0].as_object();
  ASSERT_TRUE(launch.contains("ck_pin"));
  ASSERT_TRUE(launch.contains("ck_pin_clocks"));
  EXPECT_FALSE(launch.at("ck_pin").is_null());
  const auto& launch_cks = launch.at("ck_pin_clocks").as_array();
  EXPECT_EQ(launch_cks.size(), 1u);
  EXPECT_EQ(launch_cks[0].as_string(), "clk_a");
  // Capture flop: clk_b only.
  const auto* cap = findStage(stages, "ff_unsynced");
  ASSERT_NE(cap, nullptr);
  ASSERT_TRUE(cap->contains("ck_pin_clocks"));
  const auto& cap_cks = cap->at("ck_pin_clocks").as_array();
  EXPECT_EQ(cap_cks.size(), 1u);
  EXPECT_EQ(cap_cks[0].as_string(), "clk_b");
}

// Synced 2FF: stages = launch + capture + sync2 (+ ff_synced_dst, the
// downstream flop in the same domain).
TEST_F(CdcWithStaTest, PathDetailSyncedIncludesSyncStages)
{
  auto v = getPathDetail(sync_b1_, "D", "clk_a");
  const auto& sc = v.as_object().at("sync_chain").as_object();
  EXPECT_EQ(sc.at("kind").as_string(), "ff_chain");
  EXPECT_GE(sc.at("depth").as_int64(), 2);

  const auto& stages = v.as_object().at("stages").as_array();
  EXPECT_NE(findStage(stages, "ff_src_a"), nullptr);
  EXPECT_NE(findStage(stages, "sync_b1"), nullptr);
  EXPECT_NE(findStage(stages, "sync_b2"), nullptr);
  // The capture flop is sync_b1; sync_b2 is the next sync stage.
  const auto* cap = findStage(stages, "sync_b1");
  ASSERT_NE(cap, nullptr);
  EXPECT_TRUE(cap->at("is_capture").as_bool());
  const auto* s2 = findStage(stages, "sync_b2");
  ASSERT_NE(s2, nullptr);
  EXPECT_TRUE(s2->at("is_sync_stage").as_bool());
}

// Comb back-walk: AND→OR between launch flop and capture flop. Stages
// in temporal order: launch(register) · comb(AND) · comb(OR) · capture.
TEST_F(CdcWithStaTest, PathDetailBackWalkEmitsCombStages)
{
  auto v = getPathDetail(ff_comb_b_, "D", "clk_a");
  const auto& stages = v.as_object().at("stages").as_array();
  EXPECT_NE(findStage(stages, "ff_src_a"), nullptr);
  const auto* g_or = findStage(stages, "comb_back_or");
  ASSERT_NE(g_or, nullptr);
  EXPECT_EQ(g_or->at("kind").as_string(), "comb");
  const auto* g_and = findStage(stages, "comb_back_and");
  ASSERT_NE(g_and, nullptr);
  EXPECT_EQ(g_and->at("kind").as_string(), "comb");
  // Comb stages have in_pin, out_pin, and aux_in_pins arrays.
  EXPECT_TRUE(g_and->contains("in_pin"));
  EXPECT_TRUE(g_and->contains("out_pin"));
  EXPECT_TRUE(g_and->contains("aux_in_pins"));
  // Both comb stages should be in the launch domain (clk_a).
  EXPECT_EQ(g_and->at("clock").as_string(), "clk_a");
  EXPECT_EQ(g_or->at("clock").as_string(), "clk_a");
}

TEST_F(CdcWithStaTest, PathDetailDomainMixGateFlagged)
{
  auto v = getPathDetail(ff_mix_b_, "D", "clk_a");
  const auto& stages = v.as_object().at("stages").as_array();
  const auto* mix = findStage(stages, "mix_and");
  ASSERT_NE(mix, nullptr);
  EXPECT_EQ(mix->at("kind").as_string(), "comb");
  // Domain-mix flag is set: inputs span clk_a and clk_b.
  EXPECT_TRUE(mix->at("is_domain_mix").as_bool());
  // aux_in_pins should list q_b's input pin with a clk_b clock list,
  // since the back-walk followed q_a (the launch input).
  const auto& aux = mix->at("aux_in_pins").as_array();
  ASSERT_EQ(aux.size(), 1u);
  const auto& aux_clocks = aux[0].as_object().at("clocks").as_array();
  ASSERT_EQ(aux_clocks.size(), 1u);
  EXPECT_EQ(aux_clocks[0].as_string(), "clk_b");
}

TEST_F(CdcWithStaTest, PathDetailForwardChainTracksBufPassthroughs)
{
  auto v = getPathDetail(buf_sync1_, "D", "clk_a");
  const auto& stages = v.as_object().at("stages").as_array();
  const auto* cap = findStage(stages, "buf_sync1");
  ASSERT_NE(cap, nullptr);
  // The capture flop's passthroughs_after should carry the BUF the
  // forward walk silently traversed on the way to buf_sync2.
  ASSERT_TRUE(cap->contains("passthroughs_after"));
  const auto& pts = cap->at("passthroughs_after").as_array();
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_EQ(pts[0].as_object().at("instance").as_string(), "sync_buf");
  EXPECT_EQ(pts[0].as_object().at("cell").as_string(), "BUF_X1");
  // Pass-through carries its own out_net so the expanded UI can show
  // the chain of nets between sync stages.
  EXPECT_TRUE(pts[0].as_object().contains("out_net"));
  // buf_sync2 follows as the next sync stage.
  EXPECT_NE(findStage(stages, "buf_sync2"), nullptr);
}

TEST_F(CdcWithStaTest, PathDetailEachStageHasOutNet)
{
  auto v = getPathDetail(sync_b1_, "D", "clk_a");
  const auto& stages = v.as_object().at("stages").as_array();
  ASSERT_FALSE(stages.empty());
  // Every emitted stage carries an `out_net` field — null for the
  // very last stage in the chain (no downstream net), an object
  // {name, odb_type:"net", odb_id} otherwise.
  for (const auto& s : stages) {
    ASSERT_TRUE(s.as_object().contains("out_net"));
    if (s.as_object().at("out_net").is_object()) {
      const auto& n = s.as_object().at("out_net").as_object();
      EXPECT_EQ(n.at("odb_type").as_string(), "net");
      EXPECT_GE(n.at("odb_id").as_int64(), 0);
      EXPECT_FALSE(n.at("name").as_string().empty());
    }
  }
}

TEST_F(CdcWithStaTest, PathDetailMissingPinReturnsEmpty)
{
  // Send a request with no capture_odb_id — handler returns the
  // empty envelope, not an error.
  auto resp = handler_->handleCdcPathDetail(
      makeReq(1, WebSocketRequest::kCdcPathDetail));
  auto v = parsePayload(resp);
  EXPECT_TRUE(v.as_object().at("stages").as_array().empty());
  EXPECT_EQ(v.as_object().at("sync_chain").as_object().at("kind").as_string(),
            "none");
}

TEST_F(CdcWithStaTest, PathDetailUnsupportedOdbTypeReturnsEmpty)
{
  auto req = makeReq(1, WebSocketRequest::kCdcPathDetail);
  req.json
      = makeJson({{"capture_odb_type", "bterm"}}, {{"capture_odb_id", 1}});
  auto v = parsePayload(handler_->handleCdcPathDetail(req));
  EXPECT_TRUE(v.as_object().at("stages").as_array().empty());
}

// ─── Whitelist re-classification ───────────────────────────────────────────

TEST_F(CdcWithStaTest, InstanceWhitelistReclassifiesAsSynchronized)
{
  // Pre: 3 unsynced rows in the clk_a → clk_b cell.
  EXPECT_EQ(
      matrixCell(getOverview(), "clk_a", "clk_b").at("unsynced").as_int64(), 3);

  // Whitelist ff_unsynced specifically.
  auto req = makeReq(1, WebSocketRequest::kCdcSetWhitelist);
  req.json = makeJson(
      {{"instance_patterns", "ff_unsynced"}, {"master_patterns", ""}});
  handler_->handleCdcSetWhitelist(req);

  // Post: that one flop reclassifies to synced; the count drops by 1.
  // Bind to a local bj::value first — `matrixCell` returns a reference
  // into the value's interior, and the value would otherwise be a
  // temporary destroyed at end-of-statement, leaving `cell` dangling.
  auto v_post = getOverview();
  const auto& cell = matrixCell(v_post, "clk_a", "clk_b");
  EXPECT_EQ(cell.at("unsynced").as_int64(), 2);
  EXPECT_EQ(cell.at("synced").as_int64(), 3);
  EXPECT_EQ(cell.at("paths").as_int64(), 5);

  // Drill into the row — it should now report sync_chain_kind=whitelisted.
  auto paths = getPaths("clk_a", "clk_b", "synchronized");
  const auto* row
      = findPath(paths.as_object().at("paths").as_array(), "ff_unsynced");
  ASSERT_NE(row, nullptr);
  EXPECT_EQ(row->at("sync_chain_kind").as_string(), "whitelisted");
  EXPECT_EQ(row->at("whitelist_match").as_string(), "instance");
  EXPECT_EQ(row->at("whitelist_pattern").as_string(), "ff_unsynced");
}

TEST_F(CdcWithStaTest, MasterWhitelistReclassifiesEveryDff)
{
  // Pre: 3 unsynced.
  EXPECT_EQ(
      matrixCell(getOverview(), "clk_a", "clk_b").at("unsynced").as_int64(), 3);

  // Whitelist ALL DFF_X1 cells via master pattern.
  auto req = makeReq(1, WebSocketRequest::kCdcSetWhitelist);
  req.json
      = makeJson({{"instance_patterns", ""}, {"master_patterns", "DFF_X1"}});
  handler_->handleCdcSetWhitelist(req);

  // Every CDC capture is on a DFF_X1 — they all flip to synced.
  auto v_post = getOverview();
  const auto& cell = matrixCell(v_post, "clk_a", "clk_b");
  EXPECT_EQ(cell.at("unsynced").as_int64(), 0);
  EXPECT_EQ(cell.at("synced").as_int64(), 5);

  // Sample row: ff_comb_b's whitelist_match should now be "master".
  auto paths = getPaths("clk_a", "clk_b", "synchronized");
  const auto* row
      = findPath(paths.as_object().at("paths").as_array(), "ff_comb_b");
  ASSERT_NE(row, nullptr);
  EXPECT_EQ(row->at("sync_chain_kind").as_string(), "whitelisted");
  EXPECT_EQ(row->at("whitelist_match").as_string(), "master");
}

TEST_F(CdcWithStaTest, WhitelistGlobMatchesMultipleInstances)
{
  // Glob matches every comb/mix capture flop (ff_comb_b, ff_mix_b).
  // Doesn't match ff_unsynced — it stays in the unsynced bucket.
  auto req = makeReq(1, WebSocketRequest::kCdcSetWhitelist);
  req.json
      = makeJson({{"instance_patterns", "ff_*_b"}, {"master_patterns", ""}});
  handler_->handleCdcSetWhitelist(req);

  auto v = getOverview();
  const auto& cell = matrixCell(v, "clk_a", "clk_b");
  EXPECT_EQ(cell.at("unsynced").as_int64(), 1);  // only ff_unsynced left
  EXPECT_EQ(cell.at("synced").as_int64(), 4);
}

TEST_F(CdcWithStaTest, WhitelistChangeInvalidatesPairCache)
{
  // First overview populates the pair cache.
  EXPECT_EQ(
      matrixCell(getOverview(), "clk_a", "clk_b").at("unsynced").as_int64(), 3);

  // Apply a whitelist; cache should be cleared so the NEXT overview
  // re-walks and reflects the new patterns.
  auto setReq = makeReq(1, WebSocketRequest::kCdcSetWhitelist);
  setReq.json = makeJson(
      {{"instance_patterns", "ff_unsynced"}, {"master_patterns", ""}});
  handler_->handleCdcSetWhitelist(setReq);
  EXPECT_EQ(
      matrixCell(getOverview(), "clk_a", "clk_b").at("unsynced").as_int64(), 2);

  // Clear the whitelist; counts must return to the original — proves
  // the cache didn't pin the previous classification.
  auto clearReq = makeReq(2, WebSocketRequest::kCdcSetWhitelist);
  clearReq.json
      = makeJson({{"instance_patterns", ""}, {"master_patterns", ""}});
  handler_->handleCdcSetWhitelist(clearReq);
  EXPECT_EQ(
      matrixCell(getOverview(), "clk_a", "clk_b").at("unsynced").as_int64(), 3);
}

// ─── Liberty statetable depth extraction ───────────────────────────────────
//
// `liberty_sync` Tier-1 detection reports the cell's built-in
// synchronizer depth by reading `Statetable::internalPorts().size()`.
// The classifier then combines this with any same-domain FF chain
// stages walked forward from the cell's Q. These tests cover the
// extraction half of that contract directly — Nangate45 has no
// statetable cells with >1 internal node, so we load a synthetic
// Liberty file (`sync_test.lib`) carrying SYNC2_X1 (2 internal
// nodes) and SYNC3_X1 (3 internal nodes).

class StatetableLibraryTest : public tst::Nangate45Fixture
{
 protected:
  void SetUp() override
  {
    library_ = readLiberty("_main/src/web/test/cpp/data/sync_test.lib");
    ASSERT_NE(library_, nullptr);
  }
  sta::LibertyLibrary* library_{nullptr};
};

TEST_F(StatetableLibraryTest, Sync2CellExposesTwoInternalPorts)
{
  sta::LibertyCell* sync2 = library_->findLibertyCell("SYNC2_X1");
  ASSERT_NE(sync2, nullptr);
  const sta::Statetable* st = sync2->statetable();
  ASSERT_NE(st, nullptr) << "SYNC2_X1 should carry a statetable";
  EXPECT_EQ(st->internalPorts().size(), 2u)
      << "internalPorts() should mirror the table's internal-pin count";
}

TEST_F(StatetableLibraryTest, Sync3CellExposesThreeInternalPorts)
{
  sta::LibertyCell* sync3 = library_->findLibertyCell("SYNC3_X1");
  ASSERT_NE(sync3, nullptr);
  const sta::Statetable* st = sync3->statetable();
  ASSERT_NE(st, nullptr) << "SYNC3_X1 should carry a statetable";
  EXPECT_EQ(st->internalPorts().size(), 3u)
      << "deeper synchronizer cells should report deeper internal counts";
}

TEST_F(StatetableLibraryTest, NonStatetableCellReturnsNullStatetable)
{
  // Nangate45 DFF_X1 uses an `ff` group, not a statetable. The
  // classifier falls through to Tier 3 (FF→FF chain) on these.
  sta::LibertyLibrary* nan
      = readLiberty("_main/test/Nangate45/Nangate45_typ.lib");
  ASSERT_NE(nan, nullptr);
  sta::LibertyCell* dff = nan->findLibertyCell("DFF_X1");
  ASSERT_NE(dff, nullptr);
  EXPECT_EQ(dff->statetable(), nullptr)
      << "ff-group cells must not register as liberty_sync";
}

// Integrated clock-gating cells share the statetable shape with
// vendor synchronizers (both encode their behaviour with internal
// node pins) but are NOT synchronizers — an ICG samples its enable
// through a latch and gates the clock; data flowing past it never
// re-samples through it, so no metastability margin is added. The
// CDC chain walker exits early when `isClockGate()` is true, AND
// `cellDepth` returns 0, so an ICG can never bump a chain into the
// `liberty_sync` / `composite` bucket or inflate `sync_chain_depth`.
// This test pins down the Liberty-side invariant the classifier
// relies on: Nangate45's CLKGATE_X1 must report BOTH a statetable
// (so the old, buggier "any statetable cell is a synchronizer" path
// is the path we have to guard against) AND `isClockGate()` (so the
// new guard fires).
TEST_F(StatetableLibraryTest, ClockGateIsStatetableButFlaggedAsClockGate)
{
  sta::LibertyLibrary* nan
      = readLiberty("_main/test/Nangate45/Nangate45_typ.lib");
  ASSERT_NE(nan, nullptr);
  sta::LibertyCell* icg = nan->findLibertyCell("CLKGATE_X1");
  ASSERT_NE(icg, nullptr) << "Nangate45 ships CLKGATE_X1";
  EXPECT_NE(icg->statetable(), nullptr)
      << "CLKGATE_X1 is statetable-bodied — the very ambiguity the "
         "CDC walker has to disambiguate";
  EXPECT_TRUE(icg->isClockGate())
      << "CLKGATE_X1 must report `isClockGate()` so the CDC walker "
         "can exclude it from the sync-chain classification";
}

}  // namespace
}  // namespace web
