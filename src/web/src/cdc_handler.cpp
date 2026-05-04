// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/json.hpp>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "request_dispatcher.h"
#include "request_handler.h"
#include "sta/Clock.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PatternMatch.hh"
#include "sta/PortDirection.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/Sequential.hh"
#include "sta/Units.hh"
#include "tile_generator.h"

namespace web {

// ── helpers ─────────────────────────────────────────────────────────────────

static WebSocketResponse jsonResponse(uint32_t id, const std::string& json)
{
  WebSocketResponse resp;
  resp.id = id;
  resp.type = WebSocketResponse::kJson;
  resp.payload.assign(json.begin(), json.end());
  return resp;
}

static WebSocketResponse emptyOverview(uint32_t id)
{
  return jsonResponse(id, R"({"time_unit":"ns","current_mode":"","modes":{}})");
}

static WebSocketResponse emptyPaths(uint32_t id)
{
  return jsonResponse(
      id,
      R"({"time_unit":"ns","total":0,"offset":0,)"
      R"("category_total":{"synced":0,"excluded":0,"unsynced":0},"paths":[]})");
}

// Glue: resolve a sta::Pin into {odb_type, odb_id} and emit the bare
// fields for the array-of-objects form used by Level 2 / Level 3.
static bool resolvePinOdb(const sta::Pin* pin,
                          sta::dbNetwork* db_network,
                          const char*& out_type,
                          int& out_id)
{
  if (!pin || !db_network) {
    return false;
  }
  odb::dbITerm* iterm = nullptr;
  odb::dbBTerm* bterm = nullptr;
  odb::dbModITerm* moditerm = nullptr;
  db_network->staToDb(pin, iterm, bterm, moditerm);
  if (iterm) {
    out_type = "iterm";
    out_id = static_cast<int>(iterm->getId());
    return true;
  }
  if (bterm) {
    out_type = "bterm";
    out_id = static_cast<int>(bterm->getId());
    return true;
  }
  if (moditerm) {
    out_type = "moditerm";
    out_id = static_cast<int>(moditerm->getId());
    return true;
  }
  return false;
}

static void emitPinOdbBare(boost::json::object& obj,
                           const sta::Pin* pin,
                           sta::dbNetwork* db_network)
{
  const char* type = nullptr;
  int id = 0;
  if (resolvePinOdb(pin, db_network, type, id)) {
    obj["odb_type"] = std::string(type);
    obj["odb_id"] = id;
  }
}

// Prefix variant: emits "<prefix>_odb_type" / "<prefix>_odb_id" so a
// pin can carry inline ODB refs alongside other fields on the same
// object (e.g. d_pin / q_pin / out_pin on a stage card).
static void emitPinOdbPrefixed(boost::json::object& obj,
                               const std::string& prefix,
                               const sta::Pin* pin,
                               sta::dbNetwork* db_network)
{
  const char* type = nullptr;
  int id = 0;
  if (resolvePinOdb(pin, db_network, type, id)) {
    obj[prefix + "_odb_type"] = std::string(type);
    obj[prefix + "_odb_id"] = id;
  }
}

static void emitInstanceOdbBare(boost::json::object& obj,
                                const sta::Instance* inst,
                                sta::dbNetwork* db_network)
{
  if (!inst || !db_network) {
    return;
  }
  odb::dbInst* di = nullptr;
  odb::dbModInst* dmi = nullptr;
  db_network->staToDb(inst, di, dmi);
  if (di) {
    obj["odb_type"] = std::string("inst");
    obj["odb_id"] = static_cast<int>(di->getId());
  } else if (dmi) {
    obj["odb_type"] = std::string("modinst");
    obj["odb_id"] = static_cast<int>(dmi->getId());
  }
}

// Bundle of pointers the multi-mode walk needs. Unlike SdcContext, this
// carries no specific mode/sdc/scene — the overview handler iterates
// every mode and resolves each one's Sdc on the fly via Mode::sdc().
struct CdcContext
{
  sta::dbSta* sta;
  sta::Network* network;
  sta::dbNetwork* db_network;
  std::string time_suffix;
};

static std::optional<CdcContext> makeCdcContext(
    const std::shared_ptr<TileGenerator>& gen)
{
  sta::dbSta* sta = gen ? gen->getSta() : nullptr;
  if (!sta) {
    return std::nullopt;
  }
  sta::Network* network = sta->network();
  if (!network || !network->isLinked()) {
    return std::nullopt;
  }
  const sta::Unit* tu = sta->units()->timeUnit();
  return CdcContext{sta,
                    network,
                    sta->getDbNetwork(),
                    tu ? std::string(tu->scaleAbbrevSuffix()) : "ns"};
}

// Resolve the Sdc that backs a given mode. Each Mode owns its own Sdc
// (clocks, clock_groups, exceptions are per-mode), so per-mode CDC
// classification has to read from the right one — *not* the
// scene-pinned cmdScene()->sdc().
static sta::Sdc* sdcForMode(sta::Mode* mode)
{
  if (!mode) {
    return nullptr;
  }
  return mode->sdc();
}

// ── Sync chain detection ────────────────────────────────────────────────────

// One-shot result of the synchronizer classifier.
struct SyncClass
{
  // "ff_chain"     — chain made up entirely of ff-group flops
  // "liberty_sync" — chain entirely on a single statetable sync cell
  // "composite"    — chain mixes ff-group flops and statetable cell(s)
  // "whitelisted"  — user pattern match (overrides chain detection)
  // "none"         — no chain (single capture flop, no follow-on)
  std::string kind;
  // Total bits-of-margin in the chain. Each ff-group flop contributes
  // 1; each statetable cell contributes `internalPorts().size()`. So a
  // SYNC2_X1 (2 internal nodes) followed by one DFF reports depth 3.
  int depth = 0;
  // For "whitelisted": which list matched, plus the matching pattern, so
  // the UI can explain *why* the row was reclassified.
  std::string whitelist_match;  // "instance" | "master" | ""
  std::string whitelist_pattern;
};

// Cell helpers --------------------------------------------------------------

static bool isPassThroughCell(const sta::LibertyCell* lc)
{
  return lc && (lc->isBuffer() || lc->isInverter());
}

static bool hasRegisterSequential(const sta::LibertyCell* lc)
{
  if (!lc || !lc->hasSequentials()) {
    return false;
  }
  for (const sta::Sequential& seq : lc->sequentials()) {
    if (seq.isRegister()) {
      return true;
    }
  }
  return false;
}

// Returns the first output pin of a leaf instance, or nullptr if none.
// For buffers/inverters the only output is unambiguous; for sequential cells
// this returns the Q (or first output) pin which is what the chain walk
// follows.
static const sta::Pin* findInstanceOutputPin(const sta::Instance* inst,
                                             const sta::Network* network)
{
  if (!inst || !network) {
    return nullptr;
  }
  std::unique_ptr<sta::InstancePinIterator> it(network->pinIterator(inst));
  while (it->hasNext()) {
    const sta::Pin* p = it->next();
    if (!p) {
      continue;
    }
    sta::PortDirection* dir = network->direction(p);
    if (dir && (dir->isOutput() || dir->isTristate() || dir->isBidirect())) {
      return p;
    }
  }
  return nullptr;
}

// First non-clock data input pin of a leaf instance — typically the D
// pin of a flop. Skips register-clock pins (CK), and stops at the first
// match (good enough for the CDC stage diagram, where flops have a
// single primary data input). Returns nullptr if none.
static const sta::Pin* findRegisterDataInput(const sta::Instance* inst,
                                             const sta::Network* network)
{
  if (!inst || !network) {
    return nullptr;
  }
  std::unique_ptr<sta::InstancePinIterator> it(network->pinIterator(inst));
  while (it->hasNext()) {
    const sta::Pin* p = it->next();
    if (!p) {
      continue;
    }
    if (network->isRegClkPin(p)) {
      continue;
    }
    sta::PortDirection* dir = network->direction(p);
    if (dir && dir->isInput()) {
      return p;
    }
  }
  return nullptr;
}

// First register-clock pin of a leaf instance — the CK pin in
// flop-like cells. Mirrors `findRegisterDataInput` but in reverse:
// keeps register-clock pins, skips data inputs. Returns nullptr if
// the instance isn't a sequential cell (no `isRegClkPin` matches).
static const sta::Pin* findRegisterClockInput(const sta::Instance* inst,
                                              const sta::Network* network)
{
  if (!inst || !network) {
    return nullptr;
  }
  std::unique_ptr<sta::InstancePinIterator> it(network->pinIterator(inst));
  while (it->hasNext()) {
    const sta::Pin* p = it->next();
    if (p && network->isRegClkPin(p)) {
      return p;
    }
  }
  return nullptr;
}

// Glob match against the user whitelist patterns (sta::PatternMatch uses
// unix-glob semantics, same as `get_*` SDC commands).
static bool matchAnyPattern(const std::string& subject,
                            const std::vector<std::string>& patterns,
                            std::string* matched_pattern)
{
  for (const std::string& p : patterns) {
    sta::PatternMatch pm(p);
    if (pm.match(subject.c_str())) {
      if (matched_pattern) {
        *matched_pattern = p;
      }
      return true;
    }
  }
  return false;
}

// Forward declaration so walkToNextSyncFlop can reference the pass-
// through type defined inside LaunchStage further down. (We only use
// the type in a pointer parameter, so no full definition needed yet.)
struct LaunchStage;

// Walk forward from a driver pin, recursing through buffers/inverters, until
// we hit a single load that's a sequential-cell input pin in the same capture
// domain. Returns the next sync flop instance, or nullptr.
//
// Constraints (any miss disqualifies the chain):
//   - Net has exactly one load (multi-fanout disqualifies — not a
//     dedicated sync chain).
//   - The load instance is a leaf, sequential cell.
//   - clockDomains(load_pin) contains capture_clk.
//
// `out_passthroughs` is optional: when non-null, every buf/inv the walk
// silently traverses is appended (in data-flow order) so the path-detail
// handler can surface them as `+ N hidden cells` expanders between
// sync stages, matching the back-walk's UX.
struct ForwardPassThrough
{
  const sta::Instance* inst = nullptr;
  const sta::Pin* in_pin = nullptr;
  const sta::Pin* out_pin = nullptr;
};

static const sta::Instance* walkToNextSyncFlop(
    const sta::Pin* driver_pin,
    const sta::Clock* capture_clk,
    sta::dbSta* sta,
    sta::Network* network,
    sta::Mode* mode,
    int max_depth,
    std::vector<ForwardPassThrough>* out_passthroughs = nullptr)
{
  if (!driver_pin || !capture_clk || max_depth <= 0) {
    return nullptr;
  }

  // Find the single load on this net.
  std::unique_ptr<sta::PinConnectedPinIterator> it(
      network->connectedPinIterator(driver_pin));
  const sta::Pin* sole_load = nullptr;
  int load_count = 0;
  while (it->hasNext()) {
    const sta::Pin* p = it->next();
    if (!p || p == driver_pin) {
      continue;
    }
    if (network->isHierarchical(p)) {
      continue;  // skip hier ports
    }
    sta::PortDirection* dir = network->direction(p);
    if (!dir) {
      continue;
    }
    if (dir->isOutput()) {
      continue;  // only loads
    }
    sole_load = p;
    if (++load_count > 1) {
      return nullptr;
    }
  }
  if (load_count != 1 || !sole_load) {
    return nullptr;
  }

  sta::Instance* load_inst = network->instance(sole_load);
  if (!load_inst) {
    return nullptr;
  }
  sta::LibertyCell* load_lc = network->libertyCell(load_inst);
  if (!load_lc) {
    return nullptr;
  }

  // Buffer/inverter: skip through it. Track for the path-detail UI
  // when out_passthroughs is provided (data-flow order: this cell is
  // closer to the upstream driver than later ones).
  if (isPassThroughCell(load_lc)) {
    const sta::Pin* out = findInstanceOutputPin(load_inst, network);
    if (out_passthroughs) {
      ForwardPassThrough pt;
      pt.inst = load_inst;
      pt.in_pin = sole_load;
      pt.out_pin = out;
      out_passthroughs->push_back(pt);
    }
    return walkToNextSyncFlop(
        out, capture_clk, sta, network, mode, max_depth - 1, out_passthroughs);
  }

  // Sequential cell — check the load pin is a *data* input (skip
  // clk/reset). Both ff-group registers and statetable cells qualify
  // as valid sync-chain stages: an ff_group cell stores 1 bit of
  // metastability margin, a statetable cell stores `internalPorts`
  // bits. The composite chain (ff → statetable, or statetable → ff)
  // gets surfaced via SyncClass below.
  //
  // Integrated clock-gating cells (Liberty
  // `clock_gating_integrated_cell`) are *also* statetable-bodied, but
  // they are NOT synchronizers: an ICG samples its enable through a
  // latch and ANDs the result with CK, so the gated clock — not the
  // data — passes through it. Data flowing past the ICG never
  // re-samples through it, so the ICG contributes zero metastability
  // margin. Counting ICGs as sync stages would also double-count the
  // crossing (the ICG's own E pin is already classified as a CDC
  // endpoint when the enable is cross-domain), so we exclude them
  // explicitly from chain detection.
  const bool is_clock_gate = load_lc->isClockGate();
  const bool is_register = hasRegisterSequential(load_lc);
  const bool is_statetable
      = (load_lc->statetable() != nullptr) && !is_clock_gate;
  if (!is_register && !is_statetable) {
    return nullptr;
  }
  if (network->isRegClkPin(sole_load)) {
    return nullptr;
  }

  // Same-domain check: the load's clockDomains must contain capture_clk.
  if (!mode) {
    return nullptr;
  }
  sta::ClockSet load_clks = sta->clockDomains(sole_load, mode);
  for (const sta::Clock* c : load_clks) {
    if (c == capture_clk) {
      return load_inst;
    }
  }
  return nullptr;
}

// ── Launch-side back-walk ─────────────────────────────────────────────────
//
// Phase 3 (launch-side enumeration): walk back from the capture flop's D
// pin through pass-through cells and combinational gates to the launch
// register. Each combinational gate becomes its own stage in the path
// diagram so users can see the comb logic between launch.Q and capture.D.
//
// Pass-through cells (buffer / inverter) collapse silently — they're
// noise to the user. Multi-input combinational gates emit a stage with
// `aux_in_pins` listing the other inputs; the walk continues back
// through the first input only.

struct LaunchStage
{
  enum class Kind
  {
    Register,  // launch flop / latch — terminator
    Comb,      // combinational gate — emits a stage
    Port       // top-level input port — terminator
  };
  // One entry per combinational input pin: the pin and the clock-domain
  // names visible there. On a domain-mix gate, the followed input and
  // each aux pin will carry different clocks, and the frontend renders
  // them with their actual domain badges instead of one stage-wide tag.
  struct PinInfo
  {
    const sta::Pin* pin = nullptr;
    std::vector<std::string> clocks;
  };
  // A buffer/inverter the back-walk silently traversed between this
  // stage and the next one downstream. Surfaced to the frontend so a
  // user can expand the inter-stage gap and see what was collapsed.
  struct PassThrough
  {
    const sta::Instance* inst = nullptr;
    const sta::Pin* in_pin = nullptr;
    const sta::Pin* out_pin = nullptr;
  };
  Kind kind;
  const sta::Instance* inst = nullptr;  // null for Port
  PinInfo in_pin;                       // followed input (for Comb)
  const sta::Pin* out_pin = nullptr;    // driver pin
  std::vector<PinInfo> aux_in_pins;     // other inputs of multi-input gates
  // Pass-through cells that sit between this stage's output and the
  // NEXT temporal stage's input (data-flow order: closest-to-this-stage
  // first). Empty when no buf/inv was traversed.
  std::vector<PassThrough> passthroughs_after;
  // Set on Comb stages whose inputs span multiple distinct clock
  // domains AND include the launch clock. This is where the actual
  // cross-domain mix happens — upstream of the capture flop, which
  // only re-samples the already-mixed signal.
  bool is_domain_mix = false;
  // Set when the strict-clock-match walk hit a comb gate whose
  // inputs none carry the requested clock. Without this flag we
  // would have either (a) silently stopped (yielding an empty
  // fan-in expansion — opaque to the user) or (b) followed a
  // wrong input (silently misattributing the launch source).
  // We instead emit the gate as a normal comb stage and tag it
  // here so the frontend can render a distinct "trace stopped:
  // requested clock not on any input" banner. The user sees
  // exactly where the walk halted and can audit the inputs.
  bool stuck_clock_not_in_inputs = false;
};

// Single-driver lookup on the net carrying `load_pin`. Returns the unique
// driver pin (an internal output, or a top-level INPUT port that drives
// the internal net), or nullptr if the net has no driver / multiple
// drivers / hierarchical drivers.
static const sta::Pin* findSingleNetDriver(const sta::Pin* load_pin,
                                           sta::Network* network)
{
  if (!load_pin || !network) {
    return nullptr;
  }
  std::unique_ptr<sta::PinConnectedPinIterator> it(
      network->connectedPinIterator(load_pin));
  const sta::Pin* sole_driver = nullptr;
  int driver_count = 0;
  bool saw_hier_pin = false;
  while (it->hasNext()) {
    const sta::Pin* p = it->next();
    if (!p || p == load_pin) {
      continue;
    }
    if (network->isHierarchical(p)) {
      // Local-net iterators hand back hierarchical port-of-module
      // pins — the boundary between the current scope and its
      // parent. We can't follow them via the local iterator;
      // instead we'll fall back to the flat net below.
      saw_hier_pin = true;
      continue;
    }
    sta::PortDirection* dir = network->direction(p);
    if (!dir) {
      continue;
    }
    bool is_driver = false;
    if (dir->isOutput() || dir->isTristate() || dir->isBidirect()) {
      // Internal cell driving this net.
      is_driver = true;
    } else if (dir->isInput() && network->isTopLevelPort(p)) {
      // Top-level INPUT port — a primary input drives internal nets.
      is_driver = true;
    }
    if (!is_driver) {
      continue;
    }
    sole_driver = p;
    if (++driver_count > 1) {
      return nullptr;
    }
  }

  if (driver_count == 1) {
    return sole_driver;
  }

  // No driver found at the leaf level. If we saw hierarchical
  // pins on the way, the actual driver is upstream of one of
  // those boundaries — i.e. on a sibling/parent net of the
  // FLAT (hierarchy-traversed) net. Re-scan via the dbNet's
  // flat ITerms/BTerms to locate it. This is the common case
  // for clock nets in hierarchical designs: the create_clock
  // top-level port drives the flat clock net, but each leaf
  // module sees only its local segment with a hierarchical
  // input pin as the "driver" stand-in.
  if (!saw_hier_pin) {
    return nullptr;
  }
  sta::dbNetwork* db_net = dynamic_cast<sta::dbNetwork*>(network);
  if (!db_net) {
    return nullptr;
  }
  odb::dbNet* flat = db_net->findFlatDbNet(load_pin);
  if (!flat) {
    return nullptr;
  }
  driver_count = 0;
  for (odb::dbBTerm* bt : flat->getBTerms()) {
    odb::dbIoType io = bt->getIoType();
    if (io == odb::dbIoType::INPUT || io == odb::dbIoType::INOUT) {
      sta::Pin* p = db_net->dbToSta(bt);
      if (p) {
        sole_driver = p;
        if (++driver_count > 1) {
          return nullptr;
        }
      }
    }
  }
  for (odb::dbITerm* iterm : flat->getITerms()) {
    odb::dbIoType io = iterm->getIoType();
    if (io == odb::dbIoType::OUTPUT || io == odb::dbIoType::INOUT) {
      sta::Pin* p = db_net->dbToSta(iterm);
      if (p) {
        sole_driver = p;
        if (++driver_count > 1) {
          return nullptr;
        }
      }
    }
  }
  return driver_count == 1 ? sole_driver : nullptr;
}

// All drivers of the FLAT (hierarchy-traversed) net containing
// `load_pin`. Unlike `findSingleNetDriver`, this returns every
// driver — used by the clock-mix walker, where a multi-driven
// clock net IS the convergence point (instead of an error). When
// multiple drivers exist, the walker treats the NET ITSELF as a
// mixer: each driver becomes a contributor with its own clock
// subset, and the user can recurse into any branch.
//
// Walks the FLAT net directly (skips the leaf-level
// `connectedPinIterator` step) because clock-mix tracing wants
// to cross hierarchy boundaries automatically — at the leaf
// scope the driver is often a hierarchical port, and the
// convergence is one level out. The flat net flattens the
// boundary so we see the actual driving cells directly.
static std::vector<const sta::Pin*> collectFlatNetDrivers(
    const sta::Pin* load_pin, sta::Network* network)
{
  std::vector<const sta::Pin*> drivers;
  if (!load_pin || !network) {
    return drivers;
  }
  sta::dbNetwork* db_net = dynamic_cast<sta::dbNetwork*>(network);
  if (!db_net) {
    return drivers;
  }
  odb::dbNet* flat = db_net->findFlatDbNet(load_pin);
  if (!flat) {
    return drivers;
  }
  // Top-level INPUT / INOUT BTerms drive internal nets (a primary
  // clock port is the canonical case).
  for (odb::dbBTerm* bt : flat->getBTerms()) {
    odb::dbIoType io = bt->getIoType();
    if (io == odb::dbIoType::INPUT || io == odb::dbIoType::INOUT) {
      if (sta::Pin* p = db_net->dbToSta(bt)) {
        drivers.push_back(p);
      }
    }
  }
  // OUTPUT / INOUT ITerms = cell outputs driving the net.
  for (odb::dbITerm* iterm : flat->getITerms()) {
    odb::dbIoType io = iterm->getIoType();
    if (io == odb::dbIoType::OUTPUT || io == odb::dbIoType::INOUT) {
      if (sta::Pin* p = db_net->dbToSta(iterm)) {
        drivers.push_back(p);
      }
    }
  }
  return drivers;
}

// All input pins of a leaf instance (skips outputs).
//
// Register-clock pins (CK) are normally skipped — for a regular flop
// in a DATA-path back-walk, going through CK would land in a different
// (clock-source) domain, which is meaningless. EXCEPT for integrated
// clock-gating cells (ICGs): an ICG's CK is the upstream clock signal,
// and on a CLOCK-path back-walk we DO want to follow it. The walk's
// per-input clockDomains check picks the right input naturally — when
// the requested clock is on CK, we follow CK; when the requested clock
// (e.g., the gating-control's launch domain) is on E, we follow E. So
// for ICGs we include all input pins, including CK, and let
// `strict_clock_match` + the per-input clock list disambiguate.
//
// Without this carve-out, a clock-path fan-in walk asking for clk_b
// from a downstream pin would hit the ICG, see only E in its inputs
// (because CK was filtered out), find no match for clk_b on E, and
// — under strict-match — terminate with zero stages. That was the
// "no fanin clocks" empty-expansion bug reported on capture-side CK
// clicks for designs with ICGs in their clock trees.
static std::vector<const sta::Pin*> findCombInputPins(const sta::Instance* inst,
                                                      sta::Network* network)
{
  std::vector<const sta::Pin*> out;
  if (!inst || !network) {
    return out;
  }
  sta::LibertyCell* lc = network->libertyCell(inst);
  const bool include_clk_pins = lc && lc->isClockGate();
  std::unique_ptr<sta::InstancePinIterator> it(network->pinIterator(inst));
  while (it->hasNext()) {
    const sta::Pin* p = it->next();
    if (!p) {
      continue;
    }
    sta::PortDirection* dir = network->direction(p);
    if (!dir || !dir->isInput()) {
      continue;
    }
    if (network->isRegClkPin(p) && !include_clk_pins) {
      continue;
    }
    out.push_back(p);
  }
  return out;
}

// Walk backward from a load pin (typically the capture flop's D) through
// pass-throughs and combinational gates until we hit a register output,
// a top-level port, or an unresolved fanin. Returns stages in TEMPORAL
// order: stages[0] is the launch terminator (register or port) and
// successive entries are combinational gates ending at the gate that
// directly drives the original load pin.
//
// `launch_clk` is used to disambiguate multi-input combinational gates:
// the back-walk follows the input whose clockDomains() includes the
// launch clock when there's a choice, so a gate fed by q_a (clk_a) and
// q_b (clk_b) traces back to the launch flop on the user-selected side
// rather than picking the first input blindly. When inputs span MORE
// than one distinct clock domain (and the launch clock is present),
// the gate is flagged with `is_domain_mix = true` — that's the actual
// crossover point, upstream of the capture flop.
// `strict_clock_match` controls behaviour at multi-input
// combinational gates when no input has clocks matching
// `launch_clk`:
//   - false (default, used by path-detail's launch-side walk):
//     fall back to the first input. This keeps the diagram
//     producing SOMETHING even when the requested launch clock
//     can't be matched cleanly upstream.
//   - true (used by `cdc_pin_fan_in`): STOP the walk. Picking
//     a non-matching input would silently report a register in
//     a different clock domain as "the source of clk_X", which
//     was reported as a bug ("4 clocks all converge on the
//     same register"). When the user explicitly asks for one
//     clock's fan-in, dropping out at the first non-matching
//     gate is the honest answer.
static std::vector<LaunchStage> walkLaunchSide(const sta::Pin* capture_d_pin,
                                               sta::dbSta* sta_eng,
                                               sta::Network* network,
                                               sta::Mode* mode,
                                               const sta::Clock* launch_clk,
                                               int max_depth,
                                               bool strict_clock_match = false)
{
  std::vector<LaunchStage> stages;
  if (!capture_d_pin || !network) {
    return stages;
  }

  std::unordered_set<const sta::Instance*> seen;
  if (sta::Instance* cap_inst = network->instance(capture_d_pin)) {
    seen.insert(cap_inst);  // don't walk back into the capture flop
  }

  // Helper: collect the set of clock-domain names visible on a pin.
  // Returns empty when sta/mode are unavailable.
  auto pinClockNames = [&](const sta::Pin* p) -> std::vector<std::string> {
    std::vector<std::string> names;
    if (!p || !sta_eng || !mode) {
      return names;
    }
    sta::ClockSet cks = sta_eng->clockDomains(p, mode);
    for (const sta::Clock* c : cks) {
      if (c) {
        names.emplace_back(c->name());
      }
    }
    return names;
  };
  const std::string launch_name = launch_clk ? launch_clk->name() : "";

  // Pass-throughs (buf/inv) walked back since the last emitted stage.
  // Walk order is reverse of data flow, so we accumulate in walk order
  // and reverse at attach-time to land on data-flow order. Attached to
  // the *just-emitted* stage's `passthroughs_after` because that stage
  // sits upstream (in data flow) of these pass-throughs.
  std::vector<LaunchStage::PassThrough> pending;
  auto attachPending = [&](LaunchStage& s) {
    std::reverse(pending.begin(), pending.end());
    s.passthroughs_after = std::move(pending);
    pending.clear();
  };

  const sta::Pin* cur_load = capture_d_pin;
  for (int step = 0; step < max_depth; ++step) {
    const sta::Pin* driver = findSingleNetDriver(cur_load, network);
    if (!driver) {
      break;
    }

    // Top-level port hits are terminators — we can't walk past the
    // module boundary in this widget.
    if (network->isTopLevelPort(driver)) {
      LaunchStage s;
      s.kind = LaunchStage::Kind::Port;
      s.out_pin = driver;
      attachPending(s);
      stages.push_back(s);
      break;
    }

    sta::Instance* drv_inst = network->instance(driver);
    if (!drv_inst) {
      break;
    }
    if (seen.count(drv_inst)) {
      break;  // cycle guard
    }
    seen.insert(drv_inst);

    sta::LibertyCell* lc = network->libertyCell(drv_inst);
    if (!lc) {
      break;
    }

    // Buffer / inverter — silent skip from the user's POV but we
    // record the cell in `pending` so the frontend can offer a
    // "+ N pass-through cells" expander between stage cards.
    if (isPassThroughCell(lc)) {
      auto inputs = findCombInputPins(drv_inst, network);
      if (inputs.empty()) {
        break;
      }
      LaunchStage::PassThrough pt;
      pt.inst = drv_inst;
      pt.in_pin = inputs.front();
      pt.out_pin = driver;
      pending.push_back(pt);
      cur_load = inputs.front();
      continue;
    }

    // Sequential cell (register or latch) — this is the launch flop.
    if (lc->hasSequentials()) {
      LaunchStage s;
      s.kind = LaunchStage::Kind::Register;
      s.inst = drv_inst;
      s.out_pin = driver;
      attachPending(s);
      stages.push_back(s);
      break;
    }

    // Combinational gate — pick the input to follow back, surface the
    // others as aux_in_pins, and detect domain-mix cases.
    auto inputs = findCombInputPins(drv_inst, network);
    if (inputs.empty()) {
      break;
    }

    // Per-input clock-domain enumeration. Used in three ways:
    //   1) Picking which input the back-walk follows (prefer one whose
    //      clocks include the launch clock so domain-mix gates trace
    //      back to the right launch flop).
    //   2) Detecting genuine domain-mix gates (>1 distinct clock + the
    //      launch clock present).
    //   3) Per-input badges in the rendered stage card so the user can
    //      see WHICH input is in WHICH domain.
    std::vector<std::vector<std::string>> per_input_clocks;
    per_input_clocks.reserve(inputs.size());
    size_t pick_idx = 0;
    bool pick_found = false;
    std::set<std::string> domain_union;
    for (size_t i = 0; i < inputs.size(); ++i) {
      per_input_clocks.push_back(pinClockNames(inputs[i]));
      for (const std::string& n : per_input_clocks.back()) {
        domain_union.insert(n);
      }
      if (!pick_found && !launch_name.empty()) {
        for (const std::string& n : per_input_clocks.back()) {
          if (n == launch_name) {
            pick_idx = i;
            pick_found = true;
            break;
          }
        }
      }
    }
    // Strict-match mode: when the caller provided a clock and it
    // doesn't propagate through any input of this gate, we MUST
    // stop the walk (silently picking input 0 would misattribute
    // the launch source). But emitting nothing leaves the user
    // with an empty fan-in expansion and no idea where the trace
    // halted. Compromise: emit this gate as a comb stage with
    // `stuck_clock_not_in_inputs=true` set, then break. The
    // frontend renders the gate with a "trace stopped here"
    // banner so the user sees the last cell encountered + each
    // input's clock domain (none of which matches the requested
    // clock) — actionable diagnostic info instead of an empty
    // box.
    if (strict_clock_match && !launch_name.empty() && !pick_found) {
      LaunchStage stuck;
      stuck.kind = LaunchStage::Kind::Comb;
      stuck.inst = drv_inst;
      stuck.out_pin = driver;
      stuck.stuck_clock_not_in_inputs = true;
      // Surface every input + its clocks — that's the diagnostic
      // payload the user reads to figure out why the walk
      // stopped. Pin 0 becomes `in_pin` (the rendered card's IN
      // row), the rest become aux pins (+IN rows).
      if (!inputs.empty()) {
        stuck.in_pin.pin = inputs[0];
        stuck.in_pin.clocks = per_input_clocks[0];
        for (size_t i = 1; i < inputs.size(); ++i) {
          LaunchStage::PinInfo aux;
          aux.pin = inputs[i];
          aux.clocks = per_input_clocks[i];
          stuck.aux_in_pins.push_back(std::move(aux));
        }
      }
      attachPending(stuck);
      stages.push_back(stuck);
      break;
    }

    LaunchStage s;
    s.kind = LaunchStage::Kind::Comb;
    s.inst = drv_inst;
    s.out_pin = driver;
    s.in_pin.pin = inputs[pick_idx];
    s.in_pin.clocks = per_input_clocks[pick_idx];
    for (size_t i = 0; i < inputs.size(); ++i) {
      if (i == pick_idx) {
        continue;
      }
      LaunchStage::PinInfo aux;
      aux.pin = inputs[i];
      aux.clocks = per_input_clocks[i];
      s.aux_in_pins.push_back(std::move(aux));
    }
    // Multi-domain inputs that include the launch clock signal a
    // genuine domain mix at this gate — i.e. the cross-domain bug
    // originates here, not at the capture flop.
    if (domain_union.size() > 1
        && (launch_name.empty() || domain_union.count(launch_name) > 0)) {
      s.is_domain_mix = true;
    }
    attachPending(s);
    stages.push_back(s);
    cur_load = s.in_pin.pin;
  }

  // Walk discovered stages in reverse temporal order (closest-to-capture
  // first, launch register last). Reverse to put the launch register at
  // index 0 so the rendered diagram reads top-to-bottom temporally.
  std::reverse(stages.begin(), stages.end());
  return stages;
}

// Classify a CDC endpoint. The capture flop is the instance owning the
// endpoint pin. The chain walk starts at the capture flop's Q.
//
// Tier 1 (Liberty statetable) and Tier 2 (whitelist) both extend, rather
// than replace, the Tier 3 chain depth — a whitelisted single-flop is still
// considered "synchronized" and the chain depth includes the capture flop
// itself plus any further sync stages found by the FF→FF walk.
static SyncClass classifyCdcEndpoint(
    const sta::Instance* capture_flop,
    const sta::Clock* capture_clk,
    sta::dbSta* sta,
    sta::Network* network,
    sta::Mode* mode,
    const std::vector<std::string>& instance_patterns,
    const std::vector<std::string>& master_patterns)
{
  SyncClass sc;
  if (!capture_flop) {
    return sc;
  }

  sta::LibertyCell* lc = network->libertyCell(capture_flop);

  // Per-cell depth contribution: each ff-group flop is worth 1, each
  // *vendor sync* statetable cell is worth its
  // `internalPorts().size()`. ICGs share the statetable shape but
  // encode a latch-on-enable rather than a depth-N sync ladder, so
  // they contribute 0 (and `walkToNextSyncFlop` already filters them
  // out, so reaching this with an ICG would only happen if the
  // capture flop itself were an ICG — the depth still shouldn't
  // count toward sync margin). Helper returns 0 for unknown cells
  // (defensive — caller filters).
  auto cellDepth = [](const sta::LibertyCell* c) -> int {
    if (!c) {
      return 0;
    }
    if (c->isClockGate()) {
      return 0;
    }
    if (const sta::Statetable* st = c->statetable()) {
      const int n = static_cast<int>(st->internalPorts().size());
      return n > 0 ? n : 1;
    }
    return 1;  // ff-group flop / latch
  };
  // Statetable detection for sync-chain classification — ICGs are
  // statetable-bodied but not synchronizers (see cellDepth for the
  // rationale), so they don't qualify the chain as `liberty_sync` or
  // `composite`.
  auto isStatetable = [](const sta::LibertyCell* c) {
    return c && c->statetable() != nullptr && !c->isClockGate();
  };

  // Tier 2: user whitelist. Either list match wins.
  std::string ws_match;
  std::string ws_pattern;
  if (!instance_patterns.empty()) {
    const std::string inst_path = network->pathName(capture_flop);
    if (matchAnyPattern(inst_path, instance_patterns, &ws_pattern)) {
      ws_match = "instance";
    }
  }
  if (ws_match.empty() && lc && !master_patterns.empty()) {
    const std::string master_name = lc->name();
    if (matchAnyPattern(master_name, master_patterns, &ws_pattern)) {
      ws_match = "master";
    }
  }

  // Walk the chain forward from the capture flop's Q. Each step
  // accumulates depth (per-cell contribution) and tracks whether the
  // chain saw an ff-group flop, a statetable cell, or both — that
  // determines `kind` at the end.
  int total_depth = cellDepth(lc);
  bool saw_ff = lc && !isStatetable(lc);
  bool saw_statetable = isStatetable(lc);
  int stage_count = 1;  // safety cap: count by INSTANCES, not depth

  const sta::Pin* q = findInstanceOutputPin(capture_flop, network);
  const sta::Pin* cur_q = q;
  std::unordered_set<const sta::Instance*> seen;
  seen.insert(capture_flop);
  while (cur_q) {
    const sta::Instance* next = walkToNextSyncFlop(
        cur_q, capture_clk, sta, network, mode, /*max_depth=*/5);
    if (!next || seen.count(next)) {
      break;
    }
    seen.insert(next);
    sta::LibertyCell* next_lc = network->libertyCell(next);
    total_depth += cellDepth(next_lc);
    if (isStatetable(next_lc)) {
      saw_statetable = true;
    } else {
      saw_ff = true;
    }
    ++stage_count;
    cur_q = findInstanceOutputPin(next, network);
    if (stage_count >= 8) {
      break;  // hard safety cap on chain length
    }
  }

  // Combine tiers — first match wins. Whitelist overrides chain
  // detection because the user has explicitly told us this is a sync.
  if (!ws_match.empty()) {
    sc.kind = "whitelisted";
    sc.depth = total_depth;
    sc.whitelist_match = ws_match;
    sc.whitelist_pattern = ws_pattern;
  } else if (saw_statetable && saw_ff) {
    // Mixed: ff-group flops AND statetable cells in the same chain
    // (e.g. a regular DFF capture flop followed by a vendor SYNC2,
    // or a SYNC2 capture followed by a downstream DFF). The chain
    // is real and the depth is meaningful, but the user should
    // know it's heterogeneous so they can audit each cell type.
    sc.kind = "composite";
    sc.depth = total_depth;
  } else if (saw_statetable) {
    sc.kind = "liberty_sync";
    sc.depth = total_depth;
  } else if (total_depth >= 2) {
    sc.kind = "ff_chain";
    sc.depth = total_depth;
  } else {
    sc.kind = "none";
    sc.depth = total_depth;  // 1 or 0
  }
  return sc;
}

// ── Per-endpoint CDC pair extraction ────────────────────────────────────────

struct CdcPair
{
  const sta::Pin* endpoint_pin;  // capture flop's data pin
  const sta::Instance* capture_inst;
  const sta::Clock* launch_clk;
  const sta::Clock* capture_clk;
  std::string launch_name;
  std::string capture_name;
  std::string category;  // "synchronized" | "excluded" | "unsynchronized"
  SyncClass sync;
};

// Restrict CDC analysis to register endpoints. Output ports and
// stdcell-stranded endpoints get their launch clock from input/output delay
// constraints, not from a launching flop, and aren't a useful CDC concern.
static bool isRegisterEndpoint(const sta::Pin* pin, sta::Network* network)
{
  if (!pin) {
    return false;
  }
  if (network->isTopLevelPort(pin)) {
    return false;
  }
  sta::Instance* inst = network->instance(pin);
  if (!inst) {
    return false;
  }
  sta::LibertyCell* lc = network->libertyCell(inst);
  if (!lc) {
    return false;
  }
  return lc->hasSequentials();  // covers both flops and latches
}

// Walk every register endpoint and emit one CdcPair per (launch, capture)
// crossing. Same-domain endpoints are skipped. Each pair is fully
// classified including sync detection — this is the heaviest function in
// the handler. `mode` selects which scenario's clockDomains map to
// evaluate against, and `sdc` must be that mode's Sdc (Sdc::sameClockGroup
// reads per-mode clock_groups).
static std::vector<CdcPair> collectCdcPairs(
    CdcContext& ctx,
    sta::Mode* mode,
    sta::Sdc* sdc,
    const std::vector<std::string>& instance_patterns,
    const std::vector<std::string>& master_patterns)
{
  std::vector<CdcPair> out;
  sta::dbSta* sta = ctx.sta;
  sta::Network* network = ctx.network;
  if (!mode || !sdc) {
    return out;
  }

  // Single-clock fast-out: a CDC pair needs two distinct clocks. With
  // only one clock defined for this mode, every endpoint's
  // launch/capture pair is same-domain by construction, so the walk
  // would yield zero pairs after the same-clock skip below — skip
  // the walk entirely. Critical on big designs: ensureGraph + the
  // endpoint iteration is the main cost of the scan.
  {
    int nontrivial = 0;
    for (const sta::Clock* c : sdc->clocks()) {
      if (c) {
        ++nontrivial;
        if (nontrivial >= 2) {
          break;
        }
      }
    }
    if (nontrivial < 2) {
      return out;
    }
  }

  sta->ensureGraph();
  sta::Search* search = sta->search();
  if (!search) {
    return out;
  }

  sta::VertexSet& endpoints = search->endpoints();
  out.reserve(endpoints.size() / 4);
  for (const sta::Vertex* v : endpoints) {
    if (!v) {
      continue;
    }
    const sta::Pin* pin = v->pin();
    if (!isRegisterEndpoint(pin, network)) {
      continue;
    }

    sta::Instance* inst = network->instance(pin);
    if (!inst) {
      continue;
    }

    // Capture clock(s): clocks reaching the capture flop's CK pin.
    // Most flops have a single capture clock; multi-clock check pins are
    // rare but possible (mux-clocked flops). For each capture clock we
    // emit one CdcPair per launch-clock that differs.
    sta::ClockSet capture_clks;
    {
      // Find the clock pin via the LibertyCell's first register seq.
      sta::LibertyCell* lc = network->libertyCell(inst);
      if (!lc) {
        continue;
      }
      const sta::LibertyPort* clk_port = nullptr;
      for (const sta::Sequential& seq : lc->sequentials()) {
        if (!seq.isRegister()) {
          continue;
        }
        // The clock FuncExpr usually references one port; the simplest
        // approach is to scan instance pins for one flagged isRegClkPin.
        break;  // unused in this loop, see scan below
      }
      (void) clk_port;
      // Scan pins — pick the one flagged as a register-clock pin.
      std::unique_ptr<sta::InstancePinIterator> pi(network->pinIterator(inst));
      while (pi->hasNext()) {
        const sta::Pin* p = pi->next();
        if (!p) {
          continue;
        }
        if (network->isRegClkPin(p)) {
          sta::ClockSet cks = sta->clockDomains(p, mode);
          for (sta::Clock* c : cks) {
            capture_clks.insert(c);
          }
        }
      }
    }
    if (capture_clks.empty()) {
      continue;
    }

    // Launch clock(s): clockDomains(D-pin) — clocks that propagate to
    // this endpoint. For a typical flop fed by a single launching flop,
    // this is one entry; for paths converging from multiple domains it
    // can be multiple, in which case each is its own CDC pair.
    sta::ClockSet launch_clks = sta->clockDomains(pin, mode);
    if (launch_clks.empty()) {
      continue;
    }

    for (sta::Clock* lc : launch_clks) {
      if (!lc) {
        continue;
      }
      for (sta::Clock* cc : capture_clks) {
        if (!cc || lc == cc) {
          continue;  // same clock — not CDC
        }
        CdcPair pair;
        pair.endpoint_pin = pin;
        pair.capture_inst = inst;
        pair.launch_clk = lc;
        pair.capture_clk = cc;
        pair.launch_name = lc->name();
        pair.capture_name = cc->name();

        // Categorise.
        // sameClockGroup() returns false when the two clocks are in
        // explicitly disjoint async / exclusive groups — that's our
        // "excluded" bucket.
        const bool excluded = !sdc->sameClockGroup(lc, cc);
        if (excluded) {
          pair.category = "excluded";
          // Still classify so the UI shows whether a chain happened to
          // exist anyway (cheap and informative).
          pair.sync = classifyCdcEndpoint(
              inst, cc, sta, network, mode, instance_patterns, master_patterns);
        } else {
          pair.sync = classifyCdcEndpoint(
              inst, cc, sta, network, mode, instance_patterns, master_patterns);
          if (pair.sync.kind == "ff_chain" || pair.sync.kind == "liberty_sync"
              || pair.sync.kind == "composite"
              || pair.sync.kind == "whitelisted") {
            pair.category = "synchronized";
          } else {
            pair.category = "unsynchronized";
          }
        }
        out.push_back(std::move(pair));
      }
    }
  }
  return out;
}

// ── CdcHandler ──────────────────────────────────────────────────────────────

// Cache of `collectCdcPairs` output, keyed by mode name. The full walk
// is the dominant cost of cdc_overview / cdc_paths (per-endpoint
// `clockDomains` + sync-chain walk for every register endpoint), so
// memoising it lets per-cell drill-in clicks return in microseconds
// instead of seconds. Invalidated whenever the whitelist changes
// (sync classification depends on the patterns) and re-populated by
// the next cdc_overview call.
//
// pImpl-style: forward-declared in request_handler.h so we don't need
// to leak CdcPair / sta:: types into the public header.
struct CdcHandler::PairCache
{
  std::mutex mutex;
  std::map<std::string, std::vector<CdcPair>> by_mode;
};

CdcHandler::CdcHandler(std::shared_ptr<TileGenerator> gen)
    : gen_(std::move(gen)), pair_cache_(std::make_unique<PairCache>())
{
}

// Out-of-line destructor needed because PairCache is incomplete in the
// header (pImpl): the unique_ptr's deleter must see the full type.
CdcHandler::~CdcHandler() = default;

WebSocketResponse CdcHandler::handleCdcOverview(const WebSocketRequest& req)
{
  auto ctx = makeCdcContext(gen_);
  if (!ctx) {
    return emptyOverview(req.id);
  }

  // Snapshot the whitelist once for the whole multi-mode walk so each
  // mode's classification uses the same patterns and a concurrent
  // cdc_set_whitelist call can't tear results across modes.
  std::vector<std::string> insts, masters;
  {
    std::lock_guard<std::mutex> lock(whitelist_mutex_);
    insts = instance_patterns_;
    masters = master_patterns_;
  }

  // Collect every mode the design defines. We always include the
  // current cmdMode (it's the "you are here" anchor) plus every other
  // mode registered with STA. clockDomains takes mode explicitly so we
  // don't need to mutate cmdMode/cmdScene during the walk.
  sta::ModeSeq& modes = ctx->sta->modes();

  struct CellTally
  {
    int paths = 0;
    int synced = 0;
    int excluded = 0;
    int unsynced = 0;
  };
  struct PerMode
  {
    int endpoint_count = 0;
    std::set<std::string> clocks;
    std::map<std::string, std::map<std::string, CellTally>> matrix;
  };
  // Wipe any stale per-mode cache (whitelist may have changed since last
  // overview, or the user clicked Refresh) and re-populate as we walk.
  // Subsequent cdc_paths drill-ins read from this same cache instead of
  // re-walking every endpoint per click.
  std::map<std::string, std::vector<CdcPair>> fresh_cache;
  std::map<std::string, PerMode> by_mode;
  for (sta::Mode* mode : modes) {
    if (!mode) {
      continue;
    }
    sta::Sdc* sdc = sdcForMode(mode);
    if (!sdc) {
      continue;
    }
    PerMode& pm = by_mode[mode->name()];
    auto pairs = collectCdcPairs(*ctx, mode, sdc, insts, masters);
    pm.endpoint_count = static_cast<int>(pairs.size());
    for (const CdcPair& p : pairs) {
      auto& cell = pm.matrix[p.launch_name][p.capture_name];
      ++cell.paths;
      if (p.category == "synchronized") {
        ++cell.synced;
      } else if (p.category == "excluded") {
        ++cell.excluded;
      } else {
        ++cell.unsynced;
      }
      pm.clocks.insert(p.launch_name);
      pm.clocks.insert(p.capture_name);
    }
    // Pull in every clock the mode declares so empty rows/columns
    // render (a mode where one clock has no crossings still belongs
    // on the matrix axes).
    for (sta::Clock* c : sdc->clocks()) {
      if (c) {
        pm.clocks.insert(c->name());
      }
    }
    fresh_cache[mode->name()] = std::move(pairs);
  }
  {
    std::lock_guard<std::mutex> lock(pair_cache_->mutex);
    pair_cache_->by_mode = std::move(fresh_cache);
  }

  sta::Mode* current = ctx->sta->cmdMode();
  const std::string current_name = current ? current->name() : "";

  boost::json::object root;
  root["time_unit"] = ctx->time_suffix;
  root["current_mode"] = current_name;
  boost::json::object modes_obj;
  for (const auto& [name, pm] : by_mode) {
    boost::json::object mode_obj;
    mode_obj["endpoint_count"] = pm.endpoint_count;
    boost::json::array clocks_arr;
    for (const std::string& cn : pm.clocks) {
      clocks_arr.push_back(boost::json::value(cn));
    }
    mode_obj["clocks"] = std::move(clocks_arr);
    boost::json::object matrix_obj;
    for (const auto& [launch, row] : pm.matrix) {
      boost::json::object launch_obj;
      for (const auto& [capture, cell] : row) {
        boost::json::object cell_obj;
        cell_obj["paths"] = cell.paths;
        cell_obj["synced"] = cell.synced;
        cell_obj["excluded"] = cell.excluded;
        cell_obj["unsynced"] = cell.unsynced;
        launch_obj[capture] = std::move(cell_obj);
      }
      matrix_obj[launch] = std::move(launch_obj);
    }
    mode_obj["matrix"] = std::move(matrix_obj);
    modes_obj[name] = std::move(mode_obj);
  }
  root["modes"] = std::move(modes_obj);
  return jsonResponse(req.id, boost::json::serialize(root));
}

WebSocketResponse CdcHandler::handleCdcPaths(const WebSocketRequest& req)
{
  auto ctx = makeCdcContext(gen_);
  if (!ctx) {
    return emptyPaths(req.id);
  }

  const std::string launch_filter
      = extract_string(req.json, "launch_clock");
  const std::string capture_filter
      = extract_string(req.json, "capture_clock");
  const std::string category_filter = extract_string(req.json, "category");
  // Optional capture-pin glob — mirrors `sdc_endpoint_list`'s
  // `pattern` affordance. Empty / absent means no name narrowing.
  // Unix-glob semantics via sta::PatternMatch: `*` matches any
  // sequence, `?` matches one character, otherwise exact match.
  // (For substring-style "type a bare word" matching, the frontend
  // wraps the user's input in `*…*` before sending.)
  const std::string pin_pattern = extract_string(req.json, "pattern");
  std::unique_ptr<sta::PatternMatch> pin_pat;
  if (!pin_pattern.empty()) {
    pin_pat = std::make_unique<sta::PatternMatch>(pin_pattern);
  }
  if (launch_filter.empty() || capture_filter.empty()) {
    return emptyPaths(req.id);
  }

  // Optional mode field. Empty / absent → current cmdMode (matches the
  // legacy behaviour). When the frontend caches a multi-mode overview
  // it threads the active matrix's mode name through here so per-cell
  // drill-in is consistent with the matrix the user clicked.
  const std::string mode_name = extract_string(req.json, "mode");
  sta::Mode* mode = nullptr;
  if (!mode_name.empty()) {
    mode = ctx->sta->findMode(mode_name);
  }
  if (!mode) {
    mode = ctx->sta->cmdMode();
  }
  sta::Sdc* sdc = sdcForMode(mode);
  if (!mode || !sdc) {
    return emptyPaths(req.id);
  }

  std::vector<std::string> insts, masters;
  {
    std::lock_guard<std::mutex> lock(whitelist_mutex_);
    insts = instance_patterns_;
    masters = master_patterns_;
  }

  // Read pairs from the cache populated by cdc_overview. On a cache
  // miss (e.g. user requested a mode the overview didn't see, or
  // overview hasn't run yet) we fall back to a fresh walk and
  // populate the cache so subsequent clicks for the same mode are
  // fast. A const reference into the cache keeps the work O(filter +
  // page size) on the hot path.
  const std::vector<CdcPair>* pairs = nullptr;
  std::vector<CdcPair> fresh;
  {
    std::lock_guard<std::mutex> lock(pair_cache_->mutex);
    auto it = pair_cache_->by_mode.find(mode->name());
    if (it != pair_cache_->by_mode.end()) {
      pairs = &it->second;
    }
  }
  if (!pairs) {
    fresh = collectCdcPairs(*ctx, mode, sdc, insts, masters);
    pairs = &fresh;
    // Stash the fresh result for future calls even though this call
    // missed; the next click on a different cell of the same mode
    // gets a cache hit.
    std::lock_guard<std::mutex> lock(pair_cache_->mutex);
    pair_cache_->by_mode[mode->name()] = fresh;
  }

  // Filter to the requested cell, plus optional category narrowing.
  std::vector<const CdcPair*> hits;
  hits.reserve(pairs->size());
  int cat_synced = 0, cat_excluded = 0, cat_unsynced = 0;
  for (const CdcPair& p : *pairs) {
    if (p.launch_name != launch_filter) {
      continue;
    }
    if (p.capture_name != capture_filter) {
      continue;
    }
    if (p.category == "synchronized") {
      ++cat_synced;
    } else if (p.category == "excluded") {
      ++cat_excluded;
    } else {
      ++cat_unsynced;
    }
    if (!category_filter.empty() && category_filter != "all"
        && p.category != category_filter) {
      continue;
    }
    // Capture-pin glob narrowing — applied AFTER the
    // category-total tally above so the per-category counts
    // surface the full population (matches the `kinds_total`
    // semantics on the SDC endpoints handler) and the user can
    // see "this category has N paths overall, M after my
    // pattern".
    if (pin_pat) {
      const std::string pin_name = ctx->network->pathName(p.endpoint_pin);
      if (pin_name.empty() || !pin_pat->match(pin_name.c_str())) {
        continue;
      }
    }
    hits.push_back(&p);
  }

  // Stable order: launch-pin path-name lex (so the same scan emits the
  // same page on reload).
  std::sort(hits.begin(), hits.end(), [&](const CdcPair* a, const CdcPair* b) {
    return ctx->network->pathName(a->endpoint_pin)
           < ctx->network->pathName(b->endpoint_pin);
  });

  const int total = static_cast<int>(hits.size());
  int offset = extract_int_or(req.json, "offset", 0);
  if (offset < 0) {
    offset = 0;
  }
  if (offset > total) {
    offset = total;
  }
  int limit = extract_int_or(req.json, "limit", -1);
  if (limit < 0) {
    limit = total - offset;
  }
  int end = offset + limit;
  if (end > total) {
    end = total;
  }

  boost::json::object root;
  root["time_unit"] = ctx->time_suffix;
  root["total"] = total;
  root["offset"] = offset;
  boost::json::object cat_obj;
  cat_obj["synced"] = cat_synced;
  cat_obj["excluded"] = cat_excluded;
  cat_obj["unsynced"] = cat_unsynced;
  root["category_total"] = std::move(cat_obj);
  boost::json::array paths_arr;
  for (int i = offset; i < end; ++i) {
    const CdcPair& p = *hits[i];
    boost::json::object o;
    o["capture_pin"] = std::string(ctx->network->pathName(p.endpoint_pin));
    emitPinOdbBare(o, p.endpoint_pin, ctx->db_network);
    o["capture_inst"] = std::string(ctx->network->pathName(p.capture_inst));
    {
      // Master cell name for the capture flop — useful for the user when
      // deciding what to add to the master whitelist.
      sta::LibertyCell* lc = ctx->network->libertyCell(p.capture_inst);
      if (lc) {
        o["capture_cell"] = std::string(lc->name());
      } else {
        o["capture_cell"] = nullptr;
      }
    }
    o["launch_clock"] = p.launch_name;
    o["capture_clock"] = p.capture_name;
    o["category"] = p.category;
    o["sync_chain_kind"] = p.sync.kind;
    o["sync_chain_depth"] = p.sync.depth;
    if (!p.sync.whitelist_match.empty()) {
      o["whitelist_match"] = p.sync.whitelist_match;
      o["whitelist_pattern"] = p.sync.whitelist_pattern;
    } else {
      o["whitelist_match"] = nullptr;
      o["whitelist_pattern"] = nullptr;
    }
    paths_arr.push_back(std::move(o));
  }
  root["paths"] = std::move(paths_arr);
  return jsonResponse(req.id, boost::json::serialize(root));
}

WebSocketResponse CdcHandler::handleCdcPathDetail(const WebSocketRequest& req)
{
  // Phase 3 — full implementation lands later. For now, return a
  // minimal envelope so the frontend's "drill in" link can render a
  // placeholder until the SVG flow diagram is wired.
  auto ctx = makeCdcContext(gen_);
  if (!ctx) {
    return jsonResponse(
        req.id,
        R"({"stages":[],"sync_chain":{"kind":"none","depth":0,"stages":[]}})");
  }
  // The detail call is keyed by capture-pin ODB ref (capture_odb_type +
  // capture_odb_id). We re-walk from capture flop's Q to enumerate the
  // sync stages. Launch stage enumeration (back-walking through comb
  // logic to the launch flop) is deferred until we wire in
  // findPathEnds; for now we emit only the capture-side chain since
  // that's enough to validate "how was it identified".
  const std::string odb_type = extract_string(req.json, "capture_odb_type");
  const int odb_id = extract_int_or(req.json, "capture_odb_id", -1);
  odb::dbBlock* block = gen_ ? gen_->getBlock() : nullptr;
  if (!block || odb_type.empty() || odb_id < 0) {
    return jsonResponse(
        req.id,
        R"({"stages":[],"sync_chain":{"kind":"none","depth":0,"stages":[]}})");
  }

  // Resolve capture pin → capture instance.
  odb::dbITerm* iterm = nullptr;
  if (odb_type == "iterm") {
    iterm = odb::dbITerm::getITerm(block, odb_id);
  }
  if (!iterm) {
    return jsonResponse(
        req.id,
        R"({"stages":[],"sync_chain":{"kind":"none","depth":0,"stages":[]}})");
  }
  sta::Pin* capture_pin = ctx->db_network->dbToSta(iterm);
  if (!capture_pin) {
    return jsonResponse(
        req.id,
        R"({"stages":[],"sync_chain":{"kind":"none","depth":0,"stages":[]}})");
  }
  sta::Instance* capture_inst = ctx->network->instance(capture_pin);
  if (!capture_inst) {
    return jsonResponse(
        req.id,
        R"({"stages":[],"sync_chain":{"kind":"none","depth":0,"stages":[]}})");
  }

  // Mode resolution mirrors handleCdcPaths — explicit field if set,
  // otherwise current cmdMode.
  const std::string mode_name = extract_string(req.json, "mode");
  sta::Mode* mode = nullptr;
  if (!mode_name.empty()) {
    mode = ctx->sta->findMode(mode_name);
  }
  if (!mode) {
    mode = ctx->sta->cmdMode();
  }

  // Pick the capture clock — first clock domain on the CK pin.
  sta::Clock* capture_clk = nullptr;
  if (mode) {
    std::unique_ptr<sta::InstancePinIterator> pi(
        ctx->network->pinIterator(capture_inst));
    while (pi->hasNext()) {
      const sta::Pin* p = pi->next();
      if (!p) {
        continue;
      }
      if (ctx->network->isRegClkPin(p)) {
        sta::ClockSet cks = ctx->sta->clockDomains(p, mode);
        for (sta::Clock* c : cks) {
          if (c) {
            capture_clk = c;
            break;
          }
        }
        if (capture_clk) {
          break;
        }
      }
    }
  }

  // Walk the FF→FF chain from the capture flop forward.
  std::vector<const sta::Instance*> chain;
  chain.push_back(capture_inst);
  // chain_passthroughs[i] holds the buf/inv collected when walking from
  // chain[i] to chain[i+1]; the trailing entry is always empty (the
  // last stage has no "next"). Used to populate each forward stage's
  // `passthroughs_after` so the inter-stage gap can show the
  // `+ N hidden cells` expander, matching the back-walk's UX.
  std::vector<std::vector<ForwardPassThrough>> chain_passthroughs;
  chain_passthroughs.emplace_back();
  if (capture_clk) {
    const sta::Instance* cur = capture_inst;
    std::unordered_set<const sta::Instance*> seen{cur};
    while (true) {
      const sta::Pin* q = findInstanceOutputPin(cur, ctx->network);
      std::vector<ForwardPassThrough> these;
      const sta::Instance* next = walkToNextSyncFlop(q,
                                                     capture_clk,
                                                     ctx->sta,
                                                     ctx->network,
                                                     mode,
                                                     /*depth=*/5,
                                                     &these);
      if (!next || seen.count(next)) {
        break;
      }
      seen.insert(next);
      chain_passthroughs.back() = std::move(these);
      chain.push_back(next);
      chain_passthroughs.emplace_back();
      cur = next;
      if (chain.size() >= 8) {
        break;
      }
    }
  }

  std::vector<std::string> insts, masters;
  {
    std::lock_guard<std::mutex> lock(whitelist_mutex_);
    insts = instance_patterns_;
    masters = master_patterns_;
  }
  SyncClass sc = classifyCdcEndpoint(
      capture_inst, capture_clk, ctx->sta, ctx->network, mode, insts, masters);

  // The capture flop's D pin gets its launch clock from the SDC
  // (whatever drives the CDC pair the user picked). The frontend
  // already has it from the path-list response, but threading it back
  // through avoids a recompute and supports multi-launch endpoints
  // where clockDomains() returns several clocks. Empty when omitted.
  const std::string requested_launch
      = extract_string(req.json, "launch_clock");

  // Resolve the launch clock up front — needed both for the capture
  // stage's `launch_clock` field, to tint launch-side comb / launch-
  // register stages, and to drive the multi-input back-walk through
  // gates with mixed-domain inputs.
  const sta::Pin* capture_d_pin
      = findRegisterDataInput(capture_inst, ctx->network);
  std::string launch_clock_str;
  const sta::Clock* launch_clk = nullptr;
  if (!requested_launch.empty()) {
    launch_clock_str = requested_launch;
    if (sta::Sdc* sdc = sdcForMode(mode)) {
      launch_clk = sdc->findClock(requested_launch);
    }
  } else if (mode && capture_d_pin) {
    sta::ClockSet launch_clks = ctx->sta->clockDomains(capture_d_pin, mode);
    for (const sta::Clock* c : launch_clks) {
      if (c && c != capture_clk) {
        launch_clock_str = c->name();
        launch_clk = c;
        break;
      }
    }
  }

  // Phase 3: back-walk from capture.D through pass-throughs and comb
  // gates to the launch register / port. Cap depth at 12 — enough for
  // realistic comb logic between flops, low enough that a runaway walk
  // can't stall the request.
  std::vector<LaunchStage> back_chain = walkLaunchSide(capture_d_pin,
                                                       ctx->sta,
                                                       ctx->network,
                                                       mode,
                                                       launch_clk,
                                                       /*max_depth=*/12);

  // Helper lambdas — write into a target object so each stage builds
  // its own boost::json::object.
  auto emitOdbPinFields = [&](boost::json::object& target,
                              const std::string& prefix,
                              const sta::Pin* pin) {
    emitPinOdbPrefixed(target, prefix, pin, ctx->db_network);
  };
  auto emitOutNet = [&](boost::json::object& target, const sta::Pin* out_pin) {
    if (!out_pin) {
      target["out_net"] = nullptr;
      return;
    }
    odb::dbNet* dnet = ctx->db_network->findFlatDbNet(out_pin);
    if (!dnet) {
      target["out_net"] = nullptr;
      return;
    }
    boost::json::object net_obj;
    net_obj["name"] = std::string(dnet->getName());
    net_obj["odb_type"] = std::string("net");
    net_obj["odb_id"] = static_cast<int>(dnet->getId());
    target["out_net"] = std::move(net_obj);
  };
  // Emit the array of pass-through cells (buf/inv) the back-walk
  // collapsed between this stage and the next. Frontend renders these
  // behind a "+ N hidden" expander so users can reveal the buffer
  // chain when they want to. Each entry carries enough info for an
  // inline mini-card with click-through to the inspector.
  auto emitPassthroughs
      = [&](boost::json::object& target,
            const std::vector<LaunchStage::PassThrough>& pts) {
          boost::json::array arr;
          for (const auto& pt : pts) {
            boost::json::object pt_obj;
            if (pt.inst) {
              pt_obj["instance"]
                  = std::string(ctx->network->pathName(pt.inst));
              sta::LibertyCell* lc = ctx->network->libertyCell(pt.inst);
              if (lc) {
                pt_obj["cell"] = std::string(lc->name());
              } else {
                pt_obj["cell"] = nullptr;
              }
              emitInstanceOdbBare(pt_obj, pt.inst, ctx->db_network);
            }
            if (pt.in_pin) {
              pt_obj["in_pin"]
                  = std::string(ctx->network->pathName(pt.in_pin));
              emitOdbPinFields(pt_obj, "in_pin", pt.in_pin);
            }
            if (pt.out_pin) {
              pt_obj["out_pin"]
                  = std::string(ctx->network->pathName(pt.out_pin));
              emitOdbPinFields(pt_obj, "out_pin", pt.out_pin);
              odb::dbNet* dnet = ctx->db_network->findFlatDbNet(pt.out_pin);
              if (dnet) {
                boost::json::object net_obj;
                net_obj["name"] = std::string(dnet->getName());
                net_obj["odb_type"] = std::string("net");
                net_obj["odb_id"] = static_cast<int>(dnet->getId());
                pt_obj["out_net"] = std::move(net_obj);
              } else {
                pt_obj["out_net"] = nullptr;
              }
            }
            arr.push_back(std::move(pt_obj));
          }
          target["passthroughs_after"] = std::move(arr);
        };

  boost::json::object root;
  boost::json::array stages_arr;

  // 1) Launch-side back-chain — temporal order, launch register first.
  for (size_t i = 0; i < back_chain.size(); ++i) {
    const LaunchStage& ls = back_chain[i];
    boost::json::object stage;
    if (ls.kind == LaunchStage::Kind::Port) {
      // Top-level INPUT port that drives the launch-side chain. No
      // instance — the port itself stands in for the launch flop.
      const std::string port_path
          = ls.out_pin ? ctx->network->pathName(ls.out_pin) : "";
      stage["instance"] = port_path;  // best identifier for the row
      stage["cell"] = nullptr;
      // No instance ODB ref — pin's own ODB ref carries the click.
      if (ls.out_pin) {
        stage["out_pin"] = port_path;
        emitOdbPinFields(stage, "out_pin", ls.out_pin);
        // Also emit as the legacy `q_pin` field for the existing
        // linkify wiring on the stage card.
        stage["q_pin"] = port_path;
        emitOdbPinFields(stage, "q_pin", ls.out_pin);
      }
      stage["d_pin"] = nullptr;
      stage["kind"] = std::string("port");
      stage["is_launch"] = true;
      stage["is_capture"] = false;
      stage["is_sync_stage"] = false;
    } else if (ls.kind == LaunchStage::Kind::Register) {
      const sta::Pin* d_pin
          = ls.inst ? findRegisterDataInput(ls.inst, ctx->network) : nullptr;
      const sta::Pin* ck_pin
          = ls.inst ? findRegisterClockInput(ls.inst, ctx->network) : nullptr;
      stage["instance"] = std::string(ctx->network->pathName(ls.inst));
      sta::LibertyCell* lc = ctx->network->libertyCell(ls.inst);
      if (lc) {
        stage["cell"] = std::string(lc->name());
      } else {
        stage["cell"] = nullptr;
      }
      emitInstanceOdbBare(stage, ls.inst, ctx->db_network);
      if (d_pin) {
        stage["d_pin"] = std::string(ctx->network->pathName(d_pin));
        emitOdbPinFields(stage, "d_pin", d_pin);
      } else {
        stage["d_pin"] = nullptr;
      }
      if (ls.out_pin) {
        stage["q_pin"] = std::string(ctx->network->pathName(ls.out_pin));
        emitOdbPinFields(stage, "q_pin", ls.out_pin);
      } else {
        stage["q_pin"] = nullptr;
      }
      // CK pin + every clock that reaches it. Multi-clock CK
      // (clock-mux'd flops) is the case where ONE register
      // legitimately appears as the launch source for multiple
      // clocks — the CK row in the rendered card surfaces this.
      if (ck_pin) {
        stage["ck_pin"] = std::string(ctx->network->pathName(ck_pin));
        emitOdbPinFields(stage, "ck_pin", ck_pin);
        boost::json::array ck_clocks_arr;
        if (mode) {
          sta::ClockSet cks = ctx->sta->clockDomains(ck_pin, mode);
          for (const sta::Clock* c : cks) {
            if (c) ck_clocks_arr.push_back(boost::json::value(c->name()));
          }
        }
        stage["ck_pin_clocks"] = std::move(ck_clocks_arr);
      } else {
        stage["ck_pin"] = nullptr;
        stage["ck_pin_clocks"] = boost::json::array();
      }
      stage["kind"] = std::string("register");
      stage["is_launch"] = true;
      stage["is_capture"] = false;
      stage["is_sync_stage"] = false;
    } else {  // Comb
      stage["instance"] = std::string(ctx->network->pathName(ls.inst));
      sta::LibertyCell* lc = ctx->network->libertyCell(ls.inst);
      if (lc) {
        stage["cell"] = std::string(lc->name());
      } else {
        stage["cell"] = nullptr;
      }
      emitInstanceOdbBare(stage, ls.inst, ctx->db_network);
      if (ls.in_pin.pin) {
        stage["in_pin"]
            = std::string(ctx->network->pathName(ls.in_pin.pin));
        emitOdbPinFields(stage, "in_pin", ls.in_pin.pin);
        // Per-input clock domains so the rendered pin row carries the
        // actual clock badge for THIS input — matters on domain-mix
        // gates where each input arrives on a different domain.
        boost::json::array in_clocks_arr;
        for (const std::string& c : ls.in_pin.clocks) {
          in_clocks_arr.push_back(boost::json::value(c));
        }
        stage["in_pin_clocks"] = std::move(in_clocks_arr);
      } else {
        stage["in_pin"] = nullptr;
      }
      if (ls.out_pin) {
        stage["out_pin"] = std::string(ctx->network->pathName(ls.out_pin));
        emitOdbPinFields(stage, "out_pin", ls.out_pin);
      } else {
        stage["out_pin"] = nullptr;
      }
      // Other inputs of multi-input gates so the user sees the full
      // fan-in. The walk only follows in_pin; aux pins are clickable
      // but don't drive further chain expansion. Each aux carries its
      // own clock-domain list so domain-mix gates render correctly.
      boost::json::array aux_arr;
      for (const auto& aux : ls.aux_in_pins) {
        boost::json::object aux_obj;
        aux_obj["name"] = std::string(ctx->network->pathName(aux.pin));
        const char* tp = nullptr;
        int id = 0;
        if (resolvePinOdb(aux.pin, ctx->db_network, tp, id)) {
          aux_obj["odb_type"] = std::string(tp);
          aux_obj["odb_id"] = id;
        }
        boost::json::array aux_clocks_arr;
        for (const std::string& c : aux.clocks) {
          aux_clocks_arr.push_back(boost::json::value(c));
        }
        aux_obj["clocks"] = std::move(aux_clocks_arr);
        aux_arr.push_back(std::move(aux_obj));
      }
      stage["aux_in_pins"] = std::move(aux_arr);
      stage["kind"] = std::string("comb");
      stage["is_launch"] = false;
      stage["is_capture"] = false;
      stage["is_sync_stage"] = false;
      // Genuine cross-domain mix: this gate's inputs span more than
      // one clock domain (and the launch clock is among them). The
      // crossover physically happens HERE, upstream of the capture
      // flop, which only re-samples the already-mixed signal.
      stage["is_domain_mix"] = ls.is_domain_mix;
      // Set when strict-clock-match cut the walk at this gate
      // because no input carried the requested clock. The
      // frontend renders this with a distinct "trace stopped"
      // banner so the user can audit which inputs are present.
      stage["stuck_clock_not_in_inputs"] = ls.stuck_clock_not_in_inputs;
    }
    // No launch_clock on back-chain stages (they're entirely IN the
    // launch domain — there's no clock-flip happening here).
    stage["launch_clock"] = nullptr;
    if (capture_clk) {
      stage["capture_clock"] = std::string(capture_clk->name());
    } else {
      stage["capture_clock"] = nullptr;
    }
    // `clock` is the domain colour the frontend tints with. Launch-side
    // stages all live in the launch domain.
    if (!launch_clock_str.empty()) {
      stage["clock"] = launch_clock_str;
    } else {
      stage["clock"] = nullptr;
    }
    emitOutNet(stage, ls.out_pin);
    emitPassthroughs(stage, ls.passthroughs_after);
    stages_arr.push_back(std::move(stage));
  }

  // 2) Forward chain — capture flop (the crossover) followed by any
  //    sync stages walked forward through buf/inv/FF→FF.
  for (size_t i = 0; i < chain.size(); ++i) {
    const sta::Instance* inst = chain[i];
    boost::json::object stage;
    stage["instance"] = std::string(ctx->network->pathName(inst));
    sta::LibertyCell* lc = ctx->network->libertyCell(inst);
    if (lc) {
      stage["cell"] = std::string(lc->name());
    } else {
      stage["cell"] = nullptr;
    }
    emitInstanceOdbBare(stage, inst, ctx->db_network);

    // D and Q pin paths + ODB refs so the frontend can render each
    // pin as its own clickable row.
    const sta::Pin* d_pin
        = (i == 0) ? capture_d_pin : findRegisterDataInput(inst, ctx->network);
    const sta::Pin* q_pin = findInstanceOutputPin(inst, ctx->network);
    const sta::Pin* ck_pin
        = findRegisterClockInput(inst, ctx->network);
    if (d_pin) {
      stage["d_pin"] = std::string(ctx->network->pathName(d_pin));
      emitOdbPinFields(stage, "d_pin", d_pin);
    } else {
      stage["d_pin"] = nullptr;
    }
    if (q_pin) {
      stage["q_pin"] = std::string(ctx->network->pathName(q_pin));
      emitOdbPinFields(stage, "q_pin", q_pin);
    } else {
      stage["q_pin"] = nullptr;
    }
    // CK pin + every clock that reaches it. Same multi-clock CK
    // surfacing as the launch-side block: capture flops can also
    // be clock-mux'd, in which case the CK row makes the
    // "this flop is in N domains" fact explicit.
    if (ck_pin) {
      stage["ck_pin"] = std::string(ctx->network->pathName(ck_pin));
      emitOdbPinFields(stage, "ck_pin", ck_pin);
      boost::json::array ck_clocks_arr;
      if (mode) {
        sta::ClockSet cks = ctx->sta->clockDomains(ck_pin, mode);
        for (const sta::Clock* c : cks) {
          if (c) ck_clocks_arr.push_back(boost::json::value(c->name()));
        }
      }
      stage["ck_pin_clocks"] = std::move(ck_clocks_arr);
    } else {
      stage["ck_pin"] = nullptr;
      stage["ck_pin_clocks"] = boost::json::array();
    }

    // The capture flop is the only stage with a launch ≠ capture
    // clock — that's what makes it the actual crossover point.
    // Subsequent sync stages all clock on the capture domain on
    // both their D and Q pins.
    if (i == 0 && !launch_clock_str.empty()) {
      stage["launch_clock"] = launch_clock_str;
    } else {
      stage["launch_clock"] = nullptr;
    }
    if (capture_clk) {
      stage["capture_clock"] = std::string(capture_clk->name());
      // Legacy field name some early clients (the first Phase-3
      // stage card) still read.  Same value as capture_clock.
      stage["clock"] = std::string(capture_clk->name());
    } else {
      stage["capture_clock"] = nullptr;
      stage["clock"] = nullptr;
    }
    stage["kind"] = std::string("register");
    stage["is_launch"] = false;
    stage["is_capture"] = (i == 0);
    stage["is_sync_stage"] = (i > 0);
    emitOutNet(stage, q_pin);
    // Forward-chain pass-throughs collected by walkToNextSyncFlop
    // when walking from this stage to the next. Same `+ N hidden`
    // expander UX as the back-walk side. Schema mirrors the
    // back-chain's emitPassthroughs — keep the two synchronized so
    // the frontend has one code path.
    boost::json::array pts_arr;
    if (i < chain_passthroughs.size()) {
      for (const auto& pt : chain_passthroughs[i]) {
        boost::json::object pt_obj;
        if (pt.inst) {
          pt_obj["instance"]
              = std::string(ctx->network->pathName(pt.inst));
          sta::LibertyCell* lc2 = ctx->network->libertyCell(pt.inst);
          if (lc2) {
            pt_obj["cell"] = std::string(lc2->name());
          } else {
            pt_obj["cell"] = nullptr;
          }
          emitInstanceOdbBare(pt_obj, pt.inst, ctx->db_network);
        }
        if (pt.in_pin) {
          pt_obj["in_pin"] = std::string(ctx->network->pathName(pt.in_pin));
          emitOdbPinFields(pt_obj, "in_pin", pt.in_pin);
        }
        if (pt.out_pin) {
          pt_obj["out_pin"] = std::string(ctx->network->pathName(pt.out_pin));
          emitOdbPinFields(pt_obj, "out_pin", pt.out_pin);
          odb::dbNet* dnet = ctx->db_network->findFlatDbNet(pt.out_pin);
          if (dnet) {
            boost::json::object net_obj;
            net_obj["name"] = std::string(dnet->getName());
            net_obj["odb_type"] = std::string("net");
            net_obj["odb_id"] = static_cast<int>(dnet->getId());
            pt_obj["out_net"] = std::move(net_obj);
          } else {
            pt_obj["out_net"] = nullptr;
          }
        }
        pts_arr.push_back(std::move(pt_obj));
      }
    }
    stage["passthroughs_after"] = std::move(pts_arr);
    stages_arr.push_back(std::move(stage));
  }
  root["stages"] = std::move(stages_arr);
  boost::json::object sync_obj;
  sync_obj["kind"] = sc.kind;
  sync_obj["depth"] = sc.depth;
  if (!sc.whitelist_match.empty()) {
    sync_obj["whitelist_match"] = sc.whitelist_match;
    sync_obj["whitelist_pattern"] = sc.whitelist_pattern;
  } else {
    sync_obj["whitelist_match"] = nullptr;
    sync_obj["whitelist_pattern"] = nullptr;
  }
  root["sync_chain"] = std::move(sync_obj);
  return jsonResponse(req.id, boost::json::serialize(root));
}

// Helper: parse a comma-separated string of glob patterns into a list,
// trimming whitespace and dropping empties. Patterns themselves can
// contain `*` and `?` (sta::PatternMatch unix-glob semantics).
static std::vector<std::string> splitPatterns(const std::string& csv)
{
  std::vector<std::string> out;
  if (csv.empty()) {
    return out;
  }
  std::string::size_type i = 0;
  while (i <= csv.size()) {
    std::string::size_type j = csv.find(',', i);
    if (j == std::string::npos) {
      j = csv.size();
    }
    std::string tok = csv.substr(i, j - i);
    auto a = tok.find_first_not_of(" \t\r\n");
    auto b = tok.find_last_not_of(" \t\r\n");
    if (a != std::string::npos && b != std::string::npos) {
      out.push_back(tok.substr(a, b - a + 1));
    }
    i = j + 1;
  }
  return out;
}

WebSocketResponse CdcHandler::handleCdcSetWhitelist(const WebSocketRequest& req)
{
  // Both lists are sent as comma-separated globs. An empty value clears
  // the corresponding list. Caller is responsible for re-fetching the
  // overview / paths after setting — we don't push an event ourselves.
  const std::string inst_csv
      = extract_string(req.json, "instance_patterns");
  const std::string master_csv
      = extract_string(req.json, "master_patterns");

  std::vector<std::string> new_insts = splitPatterns(inst_csv);
  std::vector<std::string> new_masters = splitPatterns(master_csv);

  {
    std::lock_guard<std::mutex> lock(whitelist_mutex_);
    instance_patterns_ = std::move(new_insts);
    master_patterns_ = std::move(new_masters);
  }
  // The cached pair lists were classified against the previous
  // whitelist; drop them so the next overview call re-walks. A
  // subsequent cdc_paths against a stale cache would mis-categorise
  // rows that the new whitelist now accepts/rejects.
  {
    std::lock_guard<std::mutex> lock(pair_cache_->mutex);
    pair_cache_->by_mode.clear();
  }

  return handleCdcGetWhitelist(req);
}

WebSocketResponse CdcHandler::handleCdcGetWhitelist(const WebSocketRequest& req)
{
  std::vector<std::string> insts, masters;
  {
    std::lock_guard<std::mutex> lock(whitelist_mutex_);
    insts = instance_patterns_;
    masters = master_patterns_;
  }
  boost::json::object root;
  boost::json::array inst_arr;
  for (const std::string& p : insts) {
    inst_arr.push_back(boost::json::value(p));
  }
  root["instance_patterns"] = std::move(inst_arr);
  boost::json::array master_arr;
  for (const std::string& p : masters) {
    master_arr.push_back(boost::json::value(p));
  }
  root["master_patterns"] = std::move(master_arr);
  return jsonResponse(req.id, boost::json::serialize(root));
}

// ── Per-pin fan-in expansion ────────────────────────────────────────────────
//
// Resolves a pin (via odb_type + odb_id) and walks BACK through the
// fan-in cone — same algorithm as `walkLaunchSide` — emitting the
// stages encountered until the first sequential cell or top-level
// port. Used by the path-detail "expand fan-in" affordance: when a
// pin has multiple clock domains landing on it (e.g. a domain-mix
// gate's input), the user clicks a clock chip and this RPC traces
// back to the source flop in THAT clock's domain. The response
// shape mirrors the launch-side stages emitted by
// `handleCdcPathDetail` so the frontend can splice the results
// into the existing diagram.
//
// The user requested "one click = one ancestor sequential" — under
// the hood that's a back-walk with `max_depth` set generously
// enough to chase past pass-through cells (buf/inv) until we land
// on a real sequential. The walk's natural termination at the
// first sequential gives us exactly that.
WebSocketResponse CdcHandler::handleCdcPinFanIn(const WebSocketRequest& req)
{
  static constexpr const char* kEmpty = R"({"stages":[]})";
  auto ctx = makeCdcContext(gen_);
  if (!ctx) {
    return jsonResponse(req.id, kEmpty);
  }
  // Resolve the starting pin — same odb_type/odb_id pattern as
  // `cdc_path_detail` uses for the capture pin.
  const std::string odb_type = extract_string(req.json, "pin_odb_type");
  const int odb_id = extract_int_or(req.json, "pin_odb_id", -1);
  odb::dbBlock* block = gen_ ? gen_->getBlock() : nullptr;
  if (!block || odb_type.empty() || odb_id < 0) {
    return jsonResponse(req.id, kEmpty);
  }
  odb::dbITerm* iterm = nullptr;
  if (odb_type == "iterm") {
    iterm = odb::dbITerm::getITerm(block, odb_id);
  }
  if (!iterm) {
    return jsonResponse(req.id, kEmpty);
  }
  sta::Pin* start_pin = ctx->db_network->dbToSta(iterm);
  if (!start_pin) {
    return jsonResponse(req.id, kEmpty);
  }

  // Mode resolution mirrors handleCdcPaths.
  const std::string mode_name = extract_string(req.json, "mode");
  sta::Mode* mode = nullptr;
  if (!mode_name.empty()) {
    mode = ctx->sta->findMode(mode_name);
  }
  if (!mode) {
    mode = ctx->sta->cmdMode();
  }

  // Resolve the requested clock — used by the back-walk to pick
  // which input of a multi-input gate to follow. Empty / missing
  // clock falls back to the first available input.
  const std::string clock_name = extract_string(req.json, "clock");
  const sta::Clock* clk = nullptr;
  if (!clock_name.empty()) {
    if (sta::Sdc* sdc = sdcForMode(mode)) {
      clk = sdc->findClock(clock_name);
    }
  }

  // Output-pin click translation. The diagram lets the user click a
  // clock chip on a stage card's OUT row — that is, on the cell's
  // OWN output pin. walkLaunchSide bootstraps by calling
  // `findSingleNetDriver(cur_load)` which, for an output pin,
  // returns the pin itself (the cell drives its net). The walk's
  // cycle-guard `seen` set was seeded with that cell's instance at
  // line 683, so the first iteration breaks immediately and the
  // user gets an empty fan-in wrapper. Workaround: when start_pin
  // is an output of a leaf comb cell, advance into that cell by
  // hand — pick the input whose clock-domain set contains the
  // requested clock (or the first input as fallback) and start the
  // walk from there. The leaf cell stays the "current stage" that
  // was already on the diagram (no re-emit), and the walk proceeds
  // upstream through the chosen input's net to find the launch
  // flop. Clicking the matching IN row chip already works because
  // findSingleNetDriver of an input pin naturally returns the
  // upstream driver.
  {
    sta::PortDirection* dir = ctx->network->direction(start_pin);
    const bool is_output_dir
        = dir
          && (dir->isOutput() || dir->isTristate() || dir->isBidirect());
    if (is_output_dir && !ctx->network->isTopLevelPort(start_pin)) {
      sta::Instance* drv_inst = ctx->network->instance(start_pin);
      sta::LibertyCell* lc
          = drv_inst ? ctx->network->libertyCell(drv_inst) : nullptr;
      if (lc && !lc->hasSequentials()) {
        auto inputs = findCombInputPins(drv_inst, ctx->network);
        const sta::Pin* picked = nullptr;
        if (!clock_name.empty()) {
          for (const sta::Pin* in : inputs) {
            sta::ClockSet cks = ctx->sta->clockDomains(in, mode);
            for (const sta::Clock* c : cks) {
              if (c && std::string(c->name()) == clock_name) {
                picked = in;
                break;
              }
            }
            if (picked) {
              break;
            }
          }
        }
        if (!picked && !inputs.empty()) {
          picked = inputs.front();
        }
        if (picked) {
          start_pin = const_cast<sta::Pin*>(picked);
        }
      }
    }
  }

  // Same back-walk as the launch-side path-detail walker. Depth 12
  // matches `handleCdcPathDetail` so the two views stay consistent;
  // the natural termination at the first sequential gives us "one
  // ancestor stage per click" as the dominant outcome.
  //
  // `strict_clock_match=true` is the load-bearing difference: when
  // the user asks for clk_X's fan-in, we MUST stop at any gate
  // whose inputs don't carry clk_X. Falling back to input 0 (the
  // path-detail walker's behaviour) would silently report a
  // different-clock register as "the launch flop for clk_X" and
  // convergence-of-bogus-clocks-onto-one-register was the user-
  // reported bug that motivated this RPC's strict mode.
  std::vector<LaunchStage> back_chain
      = walkLaunchSide(start_pin,
                       ctx->sta,
                       ctx->network,
                       mode,
                       clk,
                       /*max_depth=*/12,
                       /*strict_clock_match=*/true);

  // Emit logic — mirrors the launch-side block in
  // `handleCdcPathDetail`. Kept as a sibling implementation rather
  // than refactored into a shared helper because the two callers
  // tag stages slightly differently (the path-detail block knows
  // about the capture flop and threads launch_clock_str through;
  // here, all stages live entirely in `clock_name` and there's no
  // capture flop to anchor against).
  auto emitOdbPinFields = [&](boost::json::object& target,
                              const std::string& prefix,
                              const sta::Pin* pin) {
    emitPinOdbPrefixed(target, prefix, pin, ctx->db_network);
  };
  auto emitOutNet = [&](boost::json::object& target, const sta::Pin* out_pin) {
    if (!out_pin) {
      target["out_net"] = nullptr;
      return;
    }
    odb::dbNet* dnet = ctx->db_network->findFlatDbNet(out_pin);
    if (!dnet) {
      target["out_net"] = nullptr;
      return;
    }
    boost::json::object net_obj;
    net_obj["name"] = std::string(dnet->getName());
    net_obj["odb_type"] = std::string("net");
    net_obj["odb_id"] = static_cast<int>(dnet->getId());
    target["out_net"] = std::move(net_obj);
  };
  auto emitPassthroughs
      = [&](boost::json::object& target,
            const std::vector<LaunchStage::PassThrough>& pts) {
          boost::json::array arr;
          for (const auto& pt : pts) {
            boost::json::object pt_obj;
            if (pt.inst) {
              pt_obj["instance"]
                  = std::string(ctx->network->pathName(pt.inst));
              sta::LibertyCell* lc = ctx->network->libertyCell(pt.inst);
              if (lc) {
                pt_obj["cell"] = std::string(lc->name());
              } else {
                pt_obj["cell"] = nullptr;
              }
              emitInstanceOdbBare(pt_obj, pt.inst, ctx->db_network);
            }
            if (pt.in_pin) {
              pt_obj["in_pin"]
                  = std::string(ctx->network->pathName(pt.in_pin));
              emitOdbPinFields(pt_obj, "in_pin", pt.in_pin);
            }
            if (pt.out_pin) {
              pt_obj["out_pin"]
                  = std::string(ctx->network->pathName(pt.out_pin));
              emitOdbPinFields(pt_obj, "out_pin", pt.out_pin);
              odb::dbNet* dnet = ctx->db_network->findFlatDbNet(pt.out_pin);
              if (dnet) {
                boost::json::object net_obj;
                net_obj["name"] = std::string(dnet->getName());
                net_obj["odb_type"] = std::string("net");
                net_obj["odb_id"] = static_cast<int>(dnet->getId());
                pt_obj["out_net"] = std::move(net_obj);
              } else {
                pt_obj["out_net"] = nullptr;
              }
            }
            arr.push_back(std::move(pt_obj));
          }
          target["passthroughs_after"] = std::move(arr);
        };

  boost::json::object root;
  boost::json::array stages_arr;
  for (const LaunchStage& ls : back_chain) {
    boost::json::object stage;
    if (ls.kind == LaunchStage::Kind::Port) {
      const std::string port_path
          = ls.out_pin ? ctx->network->pathName(ls.out_pin) : "";
      stage["instance"] = port_path;
      stage["cell"] = nullptr;
      if (ls.out_pin) {
        stage["out_pin"] = port_path;
        emitOdbPinFields(stage, "out_pin", ls.out_pin);
        stage["q_pin"] = port_path;
        emitOdbPinFields(stage, "q_pin", ls.out_pin);
      }
      stage["d_pin"] = nullptr;
      stage["kind"] = std::string("port");
      stage["is_launch"] = true;
      stage["is_capture"] = false;
      stage["is_sync_stage"] = false;
    } else if (ls.kind == LaunchStage::Kind::Register) {
      const sta::Pin* d_pin
          = ls.inst ? findRegisterDataInput(ls.inst, ctx->network) : nullptr;
      const sta::Pin* ck_pin
          = ls.inst ? findRegisterClockInput(ls.inst, ctx->network) : nullptr;
      stage["instance"] = std::string(ctx->network->pathName(ls.inst));
      sta::LibertyCell* lc = ctx->network->libertyCell(ls.inst);
      if (lc) {
        stage["cell"] = std::string(lc->name());
      } else {
        stage["cell"] = nullptr;
      }
      emitInstanceOdbBare(stage, ls.inst, ctx->db_network);
      if (d_pin) {
        stage["d_pin"] = std::string(ctx->network->pathName(d_pin));
        emitOdbPinFields(stage, "d_pin", d_pin);
      } else {
        stage["d_pin"] = nullptr;
      }
      if (ls.out_pin) {
        stage["q_pin"] = std::string(ctx->network->pathName(ls.out_pin));
        emitOdbPinFields(stage, "q_pin", ls.out_pin);
      } else {
        stage["q_pin"] = nullptr;
      }
      // CK pin + every clock that reaches it. Multi-clock CK
      // (clock-mux'd flops) is the case where ONE register
      // legitimately appears as the launch source for multiple
      // clocks — the CK row in the rendered card surfaces this.
      if (ck_pin) {
        stage["ck_pin"] = std::string(ctx->network->pathName(ck_pin));
        emitOdbPinFields(stage, "ck_pin", ck_pin);
        boost::json::array ck_clocks_arr;
        if (mode) {
          sta::ClockSet cks = ctx->sta->clockDomains(ck_pin, mode);
          for (const sta::Clock* c : cks) {
            if (c) ck_clocks_arr.push_back(boost::json::value(c->name()));
          }
        }
        stage["ck_pin_clocks"] = std::move(ck_clocks_arr);
      } else {
        stage["ck_pin"] = nullptr;
        stage["ck_pin_clocks"] = boost::json::array();
      }
      stage["kind"] = std::string("register");
      stage["is_launch"] = true;
      stage["is_capture"] = false;
      stage["is_sync_stage"] = false;
    } else {  // Comb
      stage["instance"] = std::string(ctx->network->pathName(ls.inst));
      sta::LibertyCell* lc = ctx->network->libertyCell(ls.inst);
      if (lc) {
        stage["cell"] = std::string(lc->name());
      } else {
        stage["cell"] = nullptr;
      }
      emitInstanceOdbBare(stage, ls.inst, ctx->db_network);
      if (ls.in_pin.pin) {
        stage["in_pin"]
            = std::string(ctx->network->pathName(ls.in_pin.pin));
        emitOdbPinFields(stage, "in_pin", ls.in_pin.pin);
        boost::json::array in_clocks_arr;
        for (const std::string& c : ls.in_pin.clocks) {
          in_clocks_arr.push_back(boost::json::value(c));
        }
        stage["in_pin_clocks"] = std::move(in_clocks_arr);
      } else {
        stage["in_pin"] = nullptr;
      }
      if (ls.out_pin) {
        stage["out_pin"] = std::string(ctx->network->pathName(ls.out_pin));
        emitOdbPinFields(stage, "out_pin", ls.out_pin);
      } else {
        stage["out_pin"] = nullptr;
      }
      boost::json::array aux_arr;
      for (const auto& aux : ls.aux_in_pins) {
        boost::json::object aux_obj;
        aux_obj["name"] = std::string(ctx->network->pathName(aux.pin));
        const char* tp = nullptr;
        int id = 0;
        if (resolvePinOdb(aux.pin, ctx->db_network, tp, id)) {
          aux_obj["odb_type"] = std::string(tp);
          aux_obj["odb_id"] = id;
        }
        boost::json::array aux_clocks_arr;
        for (const std::string& c : aux.clocks) {
          aux_clocks_arr.push_back(boost::json::value(c));
        }
        aux_obj["clocks"] = std::move(aux_clocks_arr);
        aux_arr.push_back(std::move(aux_obj));
      }
      stage["aux_in_pins"] = std::move(aux_arr);
      stage["kind"] = std::string("comb");
      stage["is_launch"] = false;
      stage["is_capture"] = false;
      stage["is_sync_stage"] = false;
      stage["is_domain_mix"] = ls.is_domain_mix;
      // Set when strict-clock-match cut the walk at this gate
      // because no input carried the requested clock. The
      // frontend renders this with a distinct "trace stopped"
      // banner so the user can audit which inputs are present.
      stage["stuck_clock_not_in_inputs"] = ls.stuck_clock_not_in_inputs;
    }
    // Tag every emitted stage with the requested clock so the
    // frontend's clock-tint pulls from the right colour. No
    // launch_clock / capture_clock here: this is a pure ancestry
    // walk — there's no clock-flip on this side.
    stage["launch_clock"] = nullptr;
    stage["capture_clock"] = nullptr;
    if (!clock_name.empty()) {
      stage["clock"] = clock_name;
    } else {
      stage["clock"] = nullptr;
    }
    emitOutNet(stage, ls.out_pin);
    emitPassthroughs(stage, ls.passthroughs_after);
    stages_arr.push_back(std::move(stage));
  }
  root["stages"] = std::move(stages_arr);
  return jsonResponse(req.id, boost::json::serialize(root));
}

// =====================================================================
// Clock-mix tracer
// =====================================================================
//
// Walks upstream from a pin carrying multiple clocks to find every
// gate where those clocks merge. Returns a TREE — each Mixer node has
// per-input branches that recurse to ports / registers / depth limit.
// User feedback drove the BFS-vs-greedy switch (2026-05-02 round 4):
// the greedy single-path walker missed muxes that lived on
// non-followed input paths, and required the user to manually click
// through every branch to find them.
//
// `on_clock_path` flips to true as soon as the walk hops through a
// register's CK. Mixers found while it's true are clock-network
// convergences (clock muxes); mixers found while it's false are
// data-path convergences (the actual CDC bug shape — different
// clock-domain data signals merging at a comb gate).
//
// Buffers / inverters compress into `passthroughs_after` exactly like
// today's data-path walker. Multi-input pass-through cells (one input
// ⊇ cur_clocks AND only that input contributes) emit a CombTransit
// node so the user sees the cell on the diagram.
struct ClockMixNode
{
  enum class Kind
  {
    Mixer,           // ≥2 contributing inputs at a comb cell — fan out
    NetMixer,        // ≥2 drivers on the same flat net — fan out
    RegisterTransit, // hopped through a sequential cell's CK pin
    CombTransit,     // single-input pass-through (cell ⊇ cur_clocks)
    Port,            // top-level port (terminal)
    Stuck,           // no input intersects cur_clocks (phantom propagation)
    Feedback,        // cycle detected (terminal)
    DepthLimit,      // depth cap hit (terminal)
    Unresolved       // no driver / no liberty cell (terminal)
  };
  // One contributing input on a mixer-kind node — its pin and the
  // recursive subtree representing the upstream walk for that
  // branch's narrowed clock set.
  struct Branch
  {
    const sta::Pin* pin = nullptr;
    std::vector<std::string> clocks;  // intersection ∩ cur_clocks
    std::unique_ptr<ClockMixNode> subtree;
  };
  Kind kind;
  const sta::Instance* inst = nullptr;
  const sta::Pin* via_pin = nullptr;  // CK pin (RegisterTransit) or
                                      // followed input (CombTransit) or
                                      // the cur_pin we were walking from
                                      // when DepthLimit fired
  const sta::Pin* out_pin = nullptr;  // driver pin / port pin
  std::vector<std::string> clocks;    // cur_clocks at this node
  bool on_clock_path = false;         // walked through a register's CK
                                      // anywhere on the way here?
  bool degenerate = false;            // single-contributor mixer
                                      // (subset, STA edge case)
  // Mixer / NetMixer fan-out: each branch recurses upward.
  std::vector<Branch> branches;
  // RegisterTransit / CombTransit: a single upstream subtree.
  std::unique_ptr<ClockMixNode> child;
  // Buf/inv silent compressions that live between this node's output
  // and the NEXT temporal stage downstream (matches the launch-side
  // walker convention).
  std::vector<LaunchStage::PassThrough> passthroughs_after;
  // Terminal-only diagnostics. `actual_clocks` is the intersection of
  // cur_clocks with the terminal pin's *real* clockDomains (the
  // clocks that actually originate / propagate through here);
  // `unaccounted_clocks` is `cur_clocks - actual_clocks` — the clocks
  // we were tracing but that don't terminate at this point. A
  // non-empty unaccounted list means the walk on THIS branch missed
  // those clocks (they reach cur_pin via some other path the walker
  // didn't follow). The frontend renders a warning band on the
  // terminal card so the user can see the discrepancy.
  std::vector<std::string> actual_clocks;
  std::vector<std::string> unaccounted_clocks;
};

// Recursive BFS walker. Builds a tree rooted at `pin` representing the
// clock-mix structure upstream. `seen` is passed by value so each
// branch has its own cycle-guard set — two branches converging on the
// same upstream cell are NOT misclassified as feedback.
//
// Termination:
//   - depth >= max_depth          → DepthLimit (carries cur_pin via via_pin)
//   - no flat-net driver          → Unresolved
//   - cycle (drv_inst in seen)    → Feedback
//   - top-level port              → Port (with actual_clocks /
//                                          unaccounted_clocks computed)
//   - sequential w/ CK ⊇ clocks   → RegisterTransit (recurses on CK
//                                    with on_clock_path=true)
//   - sequential w/ CK ⊉ clocks   → Stuck (with diagnostics)
//   - 0 contributing comb inputs  → Stuck
//   - 1 contributing input AND it
//     covers cur_clocks            → CombTransit (single child)
//   - ≥2 contributors / single
//     contributor with subset      → Mixer (one branch per contrib)
//   - >1 flat-net drivers (no
//     covering driver)             → NetMixer (one branch per driver)
//
// Buf/inv pass-through cells are silently recursed through — the
// child subtree absorbs them into its passthroughs_after.
static std::unique_ptr<ClockMixNode> walkClockMixNode(
    const sta::Pin* pin,
    std::set<std::string> cur_clocks,
    sta::dbSta* sta_eng,
    sta::Network* network,
    sta::Mode* mode,
    int depth,
    int max_depth,
    bool on_clock_path,
    std::unordered_set<const sta::Instance*> seen)
{
  auto node = std::make_unique<ClockMixNode>();
  node->on_clock_path = on_clock_path;
  node->clocks.assign(cur_clocks.begin(), cur_clocks.end());

  auto pinClockSet = [&](const sta::Pin* p) -> std::set<std::string> {
    std::set<std::string> s;
    if (!p || !sta_eng || !mode) {
      return s;
    }
    sta::ClockSet cks = sta_eng->clockDomains(p, mode);
    for (const sta::Clock* c : cks) {
      if (c) {
        s.insert(c->name());
      }
    }
    return s;
  };
  auto setIntersection = [](const std::set<std::string>& a,
                             const std::set<std::string>& b) {
    std::set<std::string> r;
    std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
                          std::inserter(r, r.begin()));
    return r;
  };
  auto setMinus = [](const std::set<std::string>& a,
                      const std::set<std::string>& b) {
    std::vector<std::string> r;
    for (const auto& x : a) {
      if (b.count(x) == 0) {
        r.push_back(x);
      }
    }
    return r;
  };

  if (depth >= max_depth) {
    node->kind = ClockMixNode::Kind::DepthLimit;
    node->via_pin = pin;
    return node;
  }
  if (!pin || cur_clocks.empty() || !network) {
    node->kind = ClockMixNode::Kind::Unresolved;
    return node;
  }

  auto drivers = collectFlatNetDrivers(pin, network);
  if (drivers.empty()) {
    node->kind = ClockMixNode::Kind::Unresolved;
    return node;
  }

  // Multi-driver net handling. Apply the same cover/contributor
  // analysis as a comb cell, but with drivers playing the role of
  // inputs. Single covering driver → fall through to single-driver
  // path; otherwise emit a NetMixer with each contributing driver
  // as a branch.
  if (drivers.size() > 1) {
    struct DriverInfo
    {
      const sta::Pin* pin = nullptr;
      std::set<std::string> clocks;
      std::set<std::string> intersection;
    };
    std::vector<DriverInfo> infos;
    infos.reserve(drivers.size());
    for (const sta::Pin* dp : drivers) {
      DriverInfo di;
      di.pin = dp;
      di.clocks = pinClockSet(dp);
      di.intersection = setIntersection(di.clocks, cur_clocks);
      infos.push_back(std::move(di));
    }
    std::vector<size_t> contrib_idx;
    for (size_t i = 0; i < infos.size(); ++i) {
      if (!infos[i].intersection.empty()) {
        contrib_idx.push_back(i);
      }
    }
    if (contrib_idx.empty()) {
      node->kind = ClockMixNode::Kind::Stuck;
      node->actual_clocks.clear();
      node->unaccounted_clocks.assign(cur_clocks.begin(), cur_clocks.end());
      for (auto& di : infos) {
        ClockMixNode::Branch b;
        b.pin = di.pin;
        b.clocks.assign(di.clocks.begin(), di.clocks.end());
        node->branches.push_back(std::move(b));
      }
      return node;
    }
    size_t cover_idx = SIZE_MAX;
    for (size_t i = 0; i < infos.size(); ++i) {
      if (std::includes(infos[i].clocks.begin(), infos[i].clocks.end(),
                        cur_clocks.begin(), cur_clocks.end())) {
        cover_idx = i;
        break;
      }
    }
    if (cover_idx != SIZE_MAX) {
      // Any covering driver → behave like a single-driver net,
      // for the same reason combinational cells fall through on
      // any cover (the net isn't actually narrowing cur_clocks).
      drivers.assign({infos[cover_idx].pin});
    } else {
      node->kind = ClockMixNode::Kind::NetMixer;
      node->degenerate = (contrib_idx.size() == 1);
      for (size_t i : contrib_idx) {
        ClockMixNode::Branch b;
        b.pin = infos[i].pin;
        b.clocks.assign(infos[i].intersection.begin(),
                        infos[i].intersection.end());
        std::set<std::string> branch_clocks(infos[i].intersection.begin(),
                                             infos[i].intersection.end());
        b.subtree = walkClockMixNode(infos[i].pin, std::move(branch_clocks),
                                     sta_eng, network, mode, depth + 1,
                                     max_depth, on_clock_path, seen);
        node->branches.push_back(std::move(b));
      }
      return node;
    }
  }

  // Single driver path.
  const sta::Pin* driver = drivers.front();

  if (network->isTopLevelPort(driver)) {
    node->kind = ClockMixNode::Kind::Port;
    node->out_pin = driver;
    auto port_clocks = pinClockSet(driver);
    auto intersect = setIntersection(port_clocks, cur_clocks);
    node->actual_clocks.assign(intersect.begin(), intersect.end());
    node->unaccounted_clocks = setMinus(cur_clocks, intersect);
    return node;
  }

  sta::Instance* drv_inst = network->instance(driver);
  if (!drv_inst) {
    node->kind = ClockMixNode::Kind::Unresolved;
    node->out_pin = driver;
    return node;
  }
  if (seen.count(drv_inst)) {
    node->kind = ClockMixNode::Kind::Feedback;
    node->inst = drv_inst;
    node->out_pin = driver;
    return node;
  }

  sta::LibertyCell* lc = network->libertyCell(drv_inst);
  if (!lc) {
    node->kind = ClockMixNode::Kind::Unresolved;
    node->inst = drv_inst;
    node->out_pin = driver;
    return node;
  }

  // Buf/inv → silent compression: don't emit a node, recurse and let
  // the child absorb this cell into its passthroughs_after.
  if (isPassThroughCell(lc)) {
    auto inputs = findCombInputPins(drv_inst, network);
    if (inputs.empty()) {
      node->kind = ClockMixNode::Kind::Unresolved;
      node->inst = drv_inst;
      node->out_pin = driver;
      return node;
    }
    seen.insert(drv_inst);
    auto child = walkClockMixNode(inputs.front(), std::move(cur_clocks),
                                  sta_eng, network, mode, depth + 1,
                                  max_depth, on_clock_path, seen);
    LaunchStage::PassThrough pt;
    pt.inst = drv_inst;
    pt.in_pin = inputs.front();
    pt.out_pin = driver;
    child->passthroughs_after.push_back(pt);
    return child;
  }

  // Sequential (non-ICG) → hop through CK. on_clock_path flips on.
  if (lc->hasSequentials() && !lc->isClockGate()) {
    const sta::Pin* ck_pin = findRegisterClockInput(drv_inst, network);
    if (!ck_pin) {
      // Truly no CK pin — this isn't a normal flop; bail.
      node->kind = ClockMixNode::Kind::Stuck;
      node->inst = drv_inst;
      node->out_pin = driver;
      return node;
    }
    // ALWAYS hop through CK. Earlier versions only hopped when CK ⊇
    // cur_clocks — that turned generated-clock flops (Q is the
    // generated clock; CK is the master, with disjoint clock sets)
    // into Stuck terminals, which made the trace useless past the
    // first flop in any clock-divider topology (user feedback
    // 2026-05-02 round 5: "I don't get anything past the flipflops
    // — I would think we would want to trace the clock pin to get
    // the clock nets that fan into the FF").
    //
    // Semantic shift on the hop: we were tracing cur_clocks on the
    // Q net; the upstream walk traces what feeds CK. The new
    // cur_clocks is the intersection if non-empty (common case:
    // multi-clock CK, all clocks shared with Q), else CK's own
    // clock set (generated-clock case: Q.clocks ∩ CK.clocks = ∅,
    // so trace CK's master clocks instead). Either way the CK pin
    // path + its clocks land on the RegisterTransit card so the
    // user sees the boundary explicitly; unaccounted_clocks lists
    // any tracked clocks NOT on CK so the diagnostic surfaces
    // when a generated-clock boundary is crossed.
    auto ck_clocks = pinClockSet(ck_pin);
    auto intersect = setIntersection(ck_clocks, cur_clocks);
    std::set<std::string> next_clocks;
    if (!intersect.empty()) {
      next_clocks = intersect;
    } else {
      next_clocks = ck_clocks;
    }

    node->kind = ClockMixNode::Kind::RegisterTransit;
    node->inst = drv_inst;
    node->via_pin = ck_pin;
    node->out_pin = driver;
    // Stamp the CK side so the transit card can show what's
    // actually on the clock pin — useful when CK's clocks differ
    // from Q's (generated-clock crossing).
    node->actual_clocks.assign(ck_clocks.begin(), ck_clocks.end());
    node->unaccounted_clocks = setMinus(cur_clocks, ck_clocks);

    if (next_clocks.empty()) {
      // CK has NO clocks at all (no create_clock reaches it). Stop
      // here — there's nothing to trace upstream of CK.
      return node;
    }

    seen.insert(drv_inst);
    // From here upstream we're tracing the CLOCK network — flip
    // on_clock_path so any mixer found from here on labels as a
    // clock-path mix in the UI.
    node->child = walkClockMixNode(ck_pin, std::move(next_clocks),
                                   sta_eng, network, mode, depth + 1,
                                   max_depth, /*on_clock_path=*/true,
                                   seen);
    return node;
  }

  // Combinational (or ICG) — classify by per-input intersections.
  auto inputs = findCombInputPins(drv_inst, network);
  if (inputs.empty()) {
    node->kind = ClockMixNode::Kind::Unresolved;
    node->inst = drv_inst;
    node->out_pin = driver;
    return node;
  }
  struct InputInfo
  {
    const sta::Pin* pin = nullptr;
    std::set<std::string> clocks;
    std::set<std::string> intersection;
  };
  std::vector<InputInfo> infos;
  infos.reserve(inputs.size());
  for (const sta::Pin* in : inputs) {
    InputInfo info;
    info.pin = in;
    info.clocks = pinClockSet(in);
    info.intersection = setIntersection(info.clocks, cur_clocks);
    infos.push_back(std::move(info));
  }
  std::vector<size_t> contrib_idx;
  for (size_t i = 0; i < infos.size(); ++i) {
    if (!infos[i].intersection.empty()) {
      contrib_idx.push_back(i);
    }
  }
  if (contrib_idx.empty()) {
    node->kind = ClockMixNode::Kind::Stuck;
    node->inst = drv_inst;
    node->out_pin = driver;
    node->unaccounted_clocks.assign(cur_clocks.begin(), cur_clocks.end());
    for (auto& info : infos) {
      ClockMixNode::Branch b;
      b.pin = info.pin;
      b.clocks.assign(info.clocks.begin(), info.clocks.end());
      node->branches.push_back(std::move(b));
    }
    return node;
  }
  // Cover check: if ANY input ⊇ cur_clocks, the cell isn't narrowing
  // the clock set — at least one route already carries everything we
  // care about. Treat as a pass-through (CombTransit) and follow
  // that input regardless of how many OTHER inputs also contribute
  // some subset.
  //
  // The "any other inputs ALSO contribute" case is redundant
  // routing: e.g. a 2-input AND tied as a buffer, or a downstream
  // signal that re-picks up clocks already on the covering input.
  // Walking back from the covering input still reaches every
  // upstream source that contributes to cur_clocks; the BFS exhaustive
  // search lives at the level UPSTREAM of this cell, where the
  // clock set narrows for real. (User feedback 2026-05-02 round 4
  // initially asked for fan-out here, but that misclassified
  // single-clock chains as mixers — countNodes(tree, "mixer") was
  // 2 on a clk_a-only chain. Reverted: cover always wins.)
  size_t cover_idx = SIZE_MAX;
  for (size_t i = 0; i < infos.size(); ++i) {
    if (std::includes(infos[i].clocks.begin(), infos[i].clocks.end(),
                      cur_clocks.begin(), cur_clocks.end())) {
      cover_idx = i;
      break;
    }
  }
  if (cover_idx != SIZE_MAX) {
    node->kind = ClockMixNode::Kind::CombTransit;
    node->inst = drv_inst;
    node->via_pin = infos[cover_idx].pin;
    node->out_pin = driver;
    seen.insert(drv_inst);
    node->child = walkClockMixNode(infos[cover_idx].pin,
                                   std::move(cur_clocks), sta_eng,
                                   network, mode, depth + 1, max_depth,
                                   on_clock_path, seen);
    return node;
  }
  // Mixer: fan out into every contributing input.
  node->kind = ClockMixNode::Kind::Mixer;
  node->inst = drv_inst;
  node->out_pin = driver;
  node->degenerate = (contrib_idx.size() == 1);
  seen.insert(drv_inst);
  for (size_t i : contrib_idx) {
    ClockMixNode::Branch b;
    b.pin = infos[i].pin;
    b.clocks.assign(infos[i].intersection.begin(),
                    infos[i].intersection.end());
    std::set<std::string> branch_clocks(infos[i].intersection.begin(),
                                         infos[i].intersection.end());
    b.subtree = walkClockMixNode(infos[i].pin, std::move(branch_clocks),
                                 sta_eng, network, mode, depth + 1,
                                 max_depth, on_clock_path, seen);
    node->branches.push_back(std::move(b));
  }
  return node;
}


WebSocketResponse CdcHandler::handleCdcClockMixTrace(
    const WebSocketRequest& req)
{
  static constexpr const char* kEmpty = R"({"tree":null})";
  auto ctx = makeCdcContext(gen_);
  if (!ctx) {
    return jsonResponse(req.id, kEmpty);
  }
  const std::string odb_type = extract_string(req.json, "pin_odb_type");
  const int odb_id = extract_int_or(req.json, "pin_odb_id", -1);
  odb::dbBlock* block = gen_ ? gen_->getBlock() : nullptr;
  if (!block || odb_type.empty() || odb_id < 0) {
    return jsonResponse(req.id, kEmpty);
  }
  odb::dbITerm* iterm = nullptr;
  if (odb_type == "iterm") {
    iterm = odb::dbITerm::getITerm(block, odb_id);
  }
  if (!iterm) {
    return jsonResponse(req.id, kEmpty);
  }
  sta::Pin* start_pin = ctx->db_network->dbToSta(iterm);
  if (!start_pin) {
    return jsonResponse(req.id, kEmpty);
  }

  const std::string mode_name = extract_string(req.json, "mode");
  sta::Mode* mode = nullptr;
  if (!mode_name.empty()) {
    mode = ctx->sta->findMode(mode_name);
  }
  if (!mode) {
    mode = ctx->sta->cmdMode();
  }

  std::set<std::string> requested_clocks
      = extract_string_array(req.json, "clocks");
  if (requested_clocks.empty() && mode) {
    sta::ClockSet cks = ctx->sta->clockDomains(start_pin, mode);
    for (const sta::Clock* c : cks) {
      if (c) {
        requested_clocks.insert(c->name());
      }
    }
  }
  if (requested_clocks.empty()) {
    return jsonResponse(req.id, kEmpty);
  }

  // Initial on_clock_path: true iff the start pin is itself a
  // register clock-input. Clicking trace-mix on a CK row should
  // immediately label upstream mixers as clock-path mixes; clicking
  // on D / IN / +IN starts on the data path.
  const bool start_on_clock_path
      = ctx->network->isRegClkPin(start_pin);

  // Per-branch depth cap. Hierarchical real designs can run 15+ deep
  // through clock trees + buffering before reaching a primary port,
  // so 12 was cutting users off mid-trace. 20 covers typical real
  // designs while still keeping the JSON bounded — a 20-deep tree
  // with branching is already big enough that the depth-limit
  // terminal card warning serves as a hint, not a wall.
  constexpr int kMaxDepth = 20;

  std::unordered_set<const sta::Instance*> seen;
  if (sta::Instance* start_inst = ctx->network->instance(start_pin)) {
    seen.insert(start_inst);
  }
  auto root = walkClockMixNode(start_pin, requested_clocks, ctx->sta,
                               ctx->network, mode, /*depth=*/0,
                               kMaxDepth, start_on_clock_path,
                               std::move(seen));

  // Recursive JSON emission. The tree mirrors the in-memory shape:
  // mixer / net_mixer carry `branches[]`, transits carry `child`,
  // terminals carry `actual_clocks` / `unaccounted_clocks` for
  // diagnostics.
  //
  // Walker is a plain struct rather than a self-capturing
  // `std::function` lambda. The std::function form was hitting a
  // recursive-dispatch crash under deep trees built from real
  // hierarchical designs after the dbNetwork modnet fixes (gdb
  // showed the invoker pointer being clobbered to 0x80 before a
  // deep recursive call). A struct with member methods recurses
  // directly through the symbol table, so there's no closure
  // pointer to corrupt and no virtual dispatch in the path.
  // Walker is a plain struct rather than a self-capturing
  // `std::function` lambda. The std::function form was hitting a
  // recursive-dispatch crash under deep trees built from real
  // hierarchical designs after the dbNetwork modnet fixes (gdb
  // showed the invoker pointer being clobbered to 0x80 before a
  // deep recursive call). A struct with member methods recurses
  // directly through the symbol table, so there's no closure
  // pointer to corrupt and no virtual dispatch in the path.
  struct ClockMixEmitter
  {
    sta::Network* network;
    sta::dbNetwork* db_network;

    static const char* kindString(ClockMixNode::Kind k)
    {
      switch (k) {
        case ClockMixNode::Kind::Mixer:
          return "mixer";
        case ClockMixNode::Kind::NetMixer:
          return "net_mixer";
        case ClockMixNode::Kind::RegisterTransit:
          return "register_transit";
        case ClockMixNode::Kind::CombTransit:
          return "comb_transit";
        case ClockMixNode::Kind::Port:
          return "port";
        case ClockMixNode::Kind::Stuck:
          return "stuck";
        case ClockMixNode::Kind::Feedback:
          return "feedback";
        case ClockMixNode::Kind::DepthLimit:
          return "depth_limit";
        case ClockMixNode::Kind::Unresolved:
          return "unresolved";
      }
      return "unresolved";
    }

    void emitOdbPinFields(boost::json::object& target,
                          const std::string& prefix,
                          const sta::Pin* pin)
    {
      emitPinOdbPrefixed(target, prefix, pin, db_network);
    }

    void emitOutNet(boost::json::object& target, const sta::Pin* out_pin)
    {
      if (!out_pin) {
        target["out_net"] = nullptr;
        return;
      }
      odb::dbNet* dnet = db_network->findFlatDbNet(out_pin);
      if (!dnet) {
        target["out_net"] = nullptr;
        return;
      }
      boost::json::object net_obj;
      net_obj["name"] = std::string(dnet->getName());
      net_obj["odb_type"] = std::string("net");
      net_obj["odb_id"] = static_cast<int>(dnet->getId());
      target["out_net"] = std::move(net_obj);
    }

    void emitPassthroughs(boost::json::object& target,
                          const std::vector<LaunchStage::PassThrough>& pts)
    {
      boost::json::array arr;
      for (const auto& pt : pts) {
        boost::json::object pt_obj;
        if (pt.inst) {
          pt_obj["instance"] = std::string(network->pathName(pt.inst));
          sta::LibertyCell* lc = network->libertyCell(pt.inst);
          if (lc) {
            pt_obj["cell"] = std::string(lc->name());
          } else {
            pt_obj["cell"] = nullptr;
          }
          emitInstanceOdbBare(pt_obj, pt.inst, db_network);
        }
        if (pt.in_pin) {
          pt_obj["in_pin"] = std::string(network->pathName(pt.in_pin));
          emitOdbPinFields(pt_obj, "in_pin", pt.in_pin);
        }
        if (pt.out_pin) {
          pt_obj["out_pin"] = std::string(network->pathName(pt.out_pin));
          emitOdbPinFields(pt_obj, "out_pin", pt.out_pin);
        }
        arr.push_back(std::move(pt_obj));
      }
      target["passthroughs_after"] = std::move(arr);
    }

    // Build and return the JSON object representing one node. Recurses
    // directly through this method (no std::function indirection) — see
    // the struct comment above for why.
    boost::json::object emit(const ClockMixNode& n)
    {
      boost::json::object obj;
      obj["kind"] = std::string(kindString(n.kind));
      if (n.inst) {
        obj["instance"] = std::string(network->pathName(n.inst));
        sta::LibertyCell* lc = network->libertyCell(n.inst);
        if (lc) {
          obj["cell"] = std::string(lc->name());
        } else {
          obj["cell"] = nullptr;
        }
        emitInstanceOdbBare(obj, n.inst, db_network);
      } else {
        obj["instance"] = nullptr;
        obj["cell"] = nullptr;
      }
      if (n.out_pin) {
        obj["out_pin"] = std::string(network->pathName(n.out_pin));
        emitOdbPinFields(obj, "out_pin", n.out_pin);
      } else {
        obj["out_pin"] = nullptr;
      }
      if (n.via_pin) {
        obj["via_pin"] = std::string(network->pathName(n.via_pin));
        emitOdbPinFields(obj, "via_pin", n.via_pin);
      } else {
        obj["via_pin"] = nullptr;
      }
      boost::json::array clocks_arr;
      for (const std::string& c : n.clocks) {
        clocks_arr.push_back(boost::json::value(c));
      }
      obj["clocks"] = std::move(clocks_arr);
      obj["on_clock_path"] = n.on_clock_path;
      obj["degenerate"] = n.degenerate;
      if (!n.actual_clocks.empty() || !n.unaccounted_clocks.empty()) {
        boost::json::array actual_arr;
        for (const std::string& c : n.actual_clocks) {
          actual_arr.push_back(boost::json::value(c));
        }
        obj["actual_clocks"] = std::move(actual_arr);
        boost::json::array unaccounted_arr;
        for (const std::string& c : n.unaccounted_clocks) {
          unaccounted_arr.push_back(boost::json::value(c));
        }
        obj["unaccounted_clocks"] = std::move(unaccounted_arr);
      }
      boost::json::array branches_arr;
      for (const auto& br : n.branches) {
        boost::json::object br_obj;
        if (br.pin) {
          br_obj["pin"] = std::string(network->pathName(br.pin));
          const char* tp = nullptr;
          int id = 0;
          if (resolvePinOdb(br.pin, db_network, tp, id)) {
            br_obj["pin_odb_type"] = std::string(tp);
            br_obj["pin_odb_id"] = id;
          }
        } else {
          br_obj["pin"] = nullptr;
        }
        boost::json::array br_clocks_arr;
        for (const std::string& c : br.clocks) {
          br_clocks_arr.push_back(boost::json::value(c));
        }
        br_obj["clocks"] = std::move(br_clocks_arr);
        if (br.subtree) {
          br_obj["subtree"] = emit(*br.subtree);
        } else {
          br_obj["subtree"] = nullptr;
        }
        branches_arr.push_back(std::move(br_obj));
      }
      obj["branches"] = std::move(branches_arr);
      if (n.child) {
        obj["child"] = emit(*n.child);
      } else {
        obj["child"] = nullptr;
      }
      emitOutNet(obj, n.out_pin);
      emitPassthroughs(obj, n.passthroughs_after);
      return obj;
    }
  };
  ClockMixEmitter emitter{ctx->network, ctx->db_network};

  boost::json::object root_obj;
  boost::json::array req_clocks_arr;
  for (const std::string& c : requested_clocks) {
    req_clocks_arr.push_back(boost::json::value(c));
  }
  root_obj["requested_clocks"] = std::move(req_clocks_arr);
  if (root) {
    root_obj["tree"] = emitter.emit(*root);
  } else {
    root_obj["tree"] = nullptr;
  }
  return jsonResponse(req.id, boost::json::serialize(root_obj));
}

void CdcHandler::registerRequests(RequestDispatcher& d)
{
  d.add("cdc_overview",
        WebSocketRequest::kCdcOverview,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleCdcOverview(req);
        });
  d.add("cdc_paths",
        WebSocketRequest::kCdcPaths,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleCdcPaths(req);
        });
  d.add("cdc_path_detail",
        WebSocketRequest::kCdcPathDetail,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleCdcPathDetail(req);
        });
  d.add("cdc_pin_fan_in",
        WebSocketRequest::kCdcPinFanIn,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleCdcPinFanIn(req);
        });
  d.add("cdc_clock_mix_trace",
        WebSocketRequest::kCdcClockMixTrace,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleCdcClockMixTrace(req);
        });
  d.add("cdc_set_whitelist",
        WebSocketRequest::kCdcSetWhitelist,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleCdcSetWhitelist(req);
        });
  d.add("cdc_get_whitelist",
        WebSocketRequest::kCdcGetWhitelist,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleCdcGetWhitelist(req);
        });
}

}  // namespace web
