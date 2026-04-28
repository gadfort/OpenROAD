// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "json_builder.h"
#include "request_dispatcher.h"
#include "request_handler.h"
#include "sta/Clock.hh"
#include "sta/ClockGroups.hh"
#include "sta/ClockInsertion.hh"
#include "sta/ClockLatency.hh"
#include "sta/DisabledPorts.hh"
#include "sta/ExceptionPath.hh"
#include "sta/Graph.hh"
#include "sta/InputDrive.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/MinMaxValues.hh"
#include "sta/Mode.hh"
#include "sta/NetworkClass.hh"
#include "sta/PatternMatch.hh"
#include "sta/PocvMode.hh"
#include "sta/PortDelay.hh"
#include "sta/PortDirection.hh"
#include "sta/PortExtCap.hh"
#include "sta/RiseFallMinMax.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/Sequential.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingModel.hh"
#include "sta/TimingRole.hh"
#include "sta/Transition.hh"
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

// Convert an STA-internal time value to display units. Folds the
// `static_cast<double>(x / time_scale)` pattern (~20 sites) into one
// place so we don't accidentally drop the cast or divide twice.
inline double scaleTime(float v, float time_scale)
{
  return static_cast<double>(v / time_scale);
}

static WebSocketResponse emptyClocks(uint32_t id)
{
  return jsonResponse(id, R"({"clocks":[],"clock_tree":[],"time_unit":"ns"})");
}

static WebSocketResponse emptyModes(uint32_t id)
{
  return jsonResponse(id,
                      R"({"current_mode":"","scene_names":[],)"
                      R"("case_analysis":[],"logic_values":[]})");
}

static WebSocketResponse emptyPortDelays(uint32_t id)
{
  return jsonResponse(id,
                      R"({"time_unit":"ns","total":0,"offset":0,)"
                      R"("clocks_total":{},"port_delays":[]})");
}

static WebSocketResponse emptyExceptions(uint32_t id)
{
  return jsonResponse(id, R"({"time_unit":"ns","exceptions":[]})");
}

static WebSocketResponse emptyLimits(uint32_t id)
{
  return jsonResponse(
      id,
      R"({"time_unit":"ns","cap_unit":"pF",)"
      R"("clock_latencies":[],"clock_insertions":[],"clock_uncertainties":[],)"
      R"("port_loads":[],"disabled_timing":[],)"
      R"("cell_limits":[],"clock_slew_limits":[],)"
      R"("pin_clock_uncertainties":[],"pin_cap_limits":[]})");
}

// Bundle of pointers + unit scales every handler reaches for. makeSdcContext
// returns nullopt when STA / scene / sdc are unavailable so handlers can bail
// to their empty-response variant.
struct SdcContext
{
  sta::dbSta* sta;
  sta::Scene* scene;
  sta::Sdc* sdc;
  sta::Network* network;
  sta::dbNetwork* db_network;
  // Display-unit conversion: STA stores values in SI; divide by *_scale to
  // convert to display units (ns, pF, ...).
  float time_scale;
  std::string time_suffix;
  float cap_scale;
  std::string cap_suffix;
};

static std::optional<SdcContext> makeSdcContext(
    const std::shared_ptr<TileGenerator>& gen)
{
  sta::dbSta* sta = gen ? gen->getSta() : nullptr;
  if (!sta) {
    return std::nullopt;
  }
  sta::Scene* scene = sta->cmdScene();
  if (!scene) {
    return std::nullopt;
  }
  sta::Sdc* sdc = scene->sdc();
  if (!sdc) {
    return std::nullopt;
  }
  const sta::Unit* tu = sta->units()->timeUnit();
  const sta::Unit* cu = sta->units()->capacitanceUnit();
  return SdcContext{
      sta,
      scene,
      sdc,
      sta->network(),
      sta->getDbNetwork(),
      tu ? tu->scale() : 1e-9f,
      tu ? std::string(tu->scaleAbbrevSuffix()) : "ns",
      cu ? cu->scale() : 1e-12f,
      cu ? std::string(cu->scaleAbbrevSuffix()) : "pF",
  };
}

// Emit a RiseFallMinMax as four nullable JSON fields (rise_max, rise_min,
// fall_max, fall_min). When `rfmm` is null, every slot is emitted as null.
static void emitRiseFallMinMax(JsonBuilder& b,
                               const sta::RiseFallMinMax* rfmm,
                               float scale)
{
  const struct
  {
    const char* key;
    const sta::RiseFall* rf;
    const sta::MinMax* mm;
  } slots[] = {
      {"rise_max", sta::RiseFall::rise(), sta::MinMax::max()},
      {"rise_min", sta::RiseFall::rise(), sta::MinMax::min()},
      {"fall_max", sta::RiseFall::fall(), sta::MinMax::max()},
      {"fall_min", sta::RiseFall::fall(), sta::MinMax::min()},
  };
  for (const auto& s : slots) {
    float val;
    bool exists;
    if (rfmm) {
      rfmm->value(s.rf, s.mm, val, exists);
    } else {
      exists = false;
    }
    if (exists) {
      b.field(s.key, static_cast<double>(val / scale));
    } else {
      b.nullField(s.key);
    }
  }
}

// Classify an SDC exception path into the wire-format type string used by
// both handleSdcExceptions and the per-pin endpoint emit.
static const char* exceptionTypeStr(const sta::ExceptionPath* exc)
{
  if (exc->isFalse()) {
    return "false_path";
  }
  if (exc->isMultiCycle()) {
    return "multi_cycle";
  }
  if (exc->isPathDelay()) {
    return "path_delay";
  }
  if (exc->isGroupPath()) {
    return "group_path";
  }
  return "unknown";
}

// Convert a LogicValue enum to a short display string.
static const char* logicValueStr(sta::LogicValue val)
{
  switch (val) {
    case sta::LogicValue::zero:
      return "0";
    case sta::LogicValue::one:
      return "1";
    case sta::LogicValue::rise:
      return "rise";
    case sta::LogicValue::fall:
      return "fall";
    default:
      return "X";
  }
}

// Resolve a sta::Pin to its ODB handle and write {type, id} into the two
// out-params. Returns false if either input is null or the pin is unresolved.
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
    // Hierarchical (module-boundary) pin — also inspectable via kInspect
    // with odb_type=moditerm.
    out_type = "moditerm";
    out_id = static_cast<int>(moditerm->getId());
    return true;
  }
  return false;
}

// Emit "<prefix>_odb_type" / "<prefix>_odb_id" so the frontend can dispatch
// an inspect-by-ODB without a name-resolution round-trip. Emits nothing for
// unresolved pins; _linkifyPin on the frontend no-ops in that case.
static void emitPinOdbRef(JsonBuilder& b,
                          const std::string& prefix,
                          const sta::Pin* pin,
                          sta::dbNetwork* db_network)
{
  const char* type = nullptr;
  int id = 0;
  if (resolvePinOdb(pin, db_network, type, id)) {
    b.field(prefix + "_odb_type", std::string(type));
    b.field(prefix + "_odb_id", id);
  }
}

// Bare-field variant of emitPinOdbRef for arrays of {name, odb_type, odb_id}.
static void emitPinOdbBare(JsonBuilder& b,
                           const sta::Pin* pin,
                           sta::dbNetwork* db_network)
{
  const char* type = nullptr;
  int id = 0;
  if (resolvePinOdb(pin, db_network, type, id)) {
    b.field("odb_type", std::string(type));
    b.field("odb_id", id);
  }
}

// Emit bare "odb_type" / "odb_id" for an Instance: dbInst → "inst",
// dbModInst → "modinst".
static void emitInstanceOdbBare(JsonBuilder& b,
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
    b.field("odb_type", std::string("inst"));
    b.field("odb_id", static_cast<int>(di->getId()));
  } else if (dmi) {
    b.field("odb_type", std::string("modinst"));
    b.field("odb_id", static_cast<int>(dmi->getId()));
  }
}

// Setup (max) and hold (min) uncertainty values, with explicit existence
// flags so callers can null-emit absent fields. Extracted so the
// "unc->value(max,...) / unc->value(min,...)" pattern doesn't have to be
// inlined six times.
struct UncertaintyValues
{
  float setup_val = 0.0f;
  float hold_val = 0.0f;
  bool setup_exists = false;
  bool hold_exists = false;
  bool any() const { return setup_exists || hold_exists; }
};

static UncertaintyValues extractUncertainty(const sta::ClockUncertainties* unc)
{
  UncertaintyValues u;
  if (unc) {
    unc->value(sta::MinMax::max(), u.setup_val, u.setup_exists);
    unc->value(sta::MinMax::min(), u.hold_val, u.hold_exists);
  }
  return u;
}

// Emit two scaled fields for the setup/hold pair. Missing values become
// nullField so the JSON shape stays uniform.
static void emitUncertainty(JsonBuilder& b,
                            const char* setup_key,
                            const char* hold_key,
                            const UncertaintyValues& u,
                            float scale)
{
  if (u.setup_exists) {
    b.field(setup_key, static_cast<double>(u.setup_val / scale));
  } else {
    b.nullField(setup_key);
  }
  if (u.hold_exists) {
    b.field(hold_key, static_cast<double>(u.hold_val / scale));
  } else {
    b.nullField(hold_key);
  }
}

// Build a map from each clock to its direct generated-clock children.
// Clock has no children() accessor, so we build the map by scanning all clocks.
static std::map<const sta::Clock*, std::vector<sta::Clock*>> buildChildrenMap(
    const sta::ClockSeq& clocks)
{
  std::map<const sta::Clock*, std::vector<sta::Clock*>> children;
  for (sta::Clock* clk : clocks) {
    if (!clk) {
      continue;
    }
    sta::Clock* master = clk->masterClk();
    if (master) {
      children[master].push_back(clk);
    }
  }
  return children;
}

// Recursively emit a clock-tree node (name + nested children array).
static void emitTreeNode(
    JsonBuilder& b,
    const sta::Clock* clk,
    const std::map<const sta::Clock*, std::vector<sta::Clock*>>& children)
{
  if (!clk) {
    return;
  }
  b.beginObject();
  b.field("name", std::string(clk->name()));
  b.beginArray("children");
  auto it = children.find(clk);
  if (it != children.end()) {
    for (sta::Clock* child : it->second) {
      emitTreeNode(b, child, children);
    }
  }
  b.endArray();
  b.endObject();
}

// ── SdcHandler ──────────────────────────────────────────────────────────────

SdcHandler::SdcHandler(std::shared_ptr<TileGenerator> gen)
    : gen_(std::move(gen))
{
}

WebSocketResponse SdcHandler::handleSdcClocks(const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return emptyClocks(req.id);
  }
  sta::Sdc* sdc = ctx->sdc;
  sta::Network* network = ctx->network;
  sta::dbNetwork* db_network = ctx->db_network;
  const float time_scale = ctx->time_scale;
  const std::string& time_suffix = ctx->time_suffix;
  const sta::ClockSeq& clocks = sdc->clocks();

  auto childrenMap = buildChildrenMap(clocks);

  JsonBuilder b;
  b.beginObject();
  b.field("time_unit", time_suffix);

  // ── "clocks" array ──────────────────────────────────────────────────────
  b.beginArray("clocks");
  for (sta::Clock* clk : clocks) {
    if (!clk) {
      continue;
    }
    b.beginObject();
    b.field("name", std::string(clk->name()));

    // period() is always a float scalar — safe to read directly.
    b.field("period", scaleTime(clk->period(), time_scale));

    // Waveform: alternating rise/fall edge times. May be empty.
    const sta::FloatSeq& waveform = clk->waveform();
    if (!waveform.empty()) {
      b.beginArray("waveform");
      for (float t : waveform) {
        b.value(scaleTime(t, time_scale));
      }
      b.endArray();
    } else {
      // Synthesize a default 50% duty-cycle waveform from the period.
      b.beginArray("waveform");
      b.value(0.0);
      b.value(scaleTime(clk->period(), time_scale) / 2.0);
      b.endArray();
    }

    b.field("is_generated", clk->isGenerated());
    b.field("is_virtual", clk->isVirtual());
    b.field("is_propagated", clk->isPropagated());
    // create_clock -add: when true, this clock was added to existing pins
    // rather than replacing them. Surfaced so users can spot multi-clock
    // pins.
    b.field("add_to_pins", clk->addToPins());
    // Generated-clock diagnostics (omitted/null on primary clocks).
    if (clk->isGenerated()) {
      // create_generated_clock -combinational: the master clock is reached
      // through combinational logic only (no flop in between).
      b.field("combinational", clk->combinational());
      // True when STA inferred the master clock from the graph rather than
      // the user specifying it explicitly with -master_clock. Useful
      // diagnostic when chasing why STA picked a particular master.
      b.field("master_inferred", clk->masterClkInfered());
      // True when STA has computed the generated waveform from the master
      // clock. False = the period/waveform shown is stale (typically until
      // update_generated_clks runs). The toolbar's "Resolve generated"
      // button surfaces this in aggregate; this field exposes it per-card
      // so the user can see which specific generated clocks are stale.
      // (Clock::generatedUpToDate is declared but not implemented in
      // OpenSTA today — waveformValid() carries the same meaning and is
      // the same check used elsewhere in this handler.)
      b.field("generated_up_to_date", clk->waveformValid());
    } else {
      b.nullField("combinational");
      b.nullField("master_inferred");
      b.nullField("generated_up_to_date");
    }
    // Comment carried by Clock (and other SdcCmdComment subclasses) — set
    // via `-comment` on `create_clock`.  Always emit (empty string when
    // none) so the JSON shape is uniform.
    b.field("comment", clk->comment());

    // master_clock: non-null only for generated clocks.
    sta::Clock* master = clk->masterClk();
    if (master) {
      b.field("master_clock", std::string(master->name()));
    } else {
      b.nullField("master_clock");
    }

    // Generated-clock-specific fields.
    if (clk->isGenerated()) {
      int div = clk->divideBy();
      int mul = clk->multiplyBy();
      if (div > 0) {
        b.field("divide_by", div);
      } else {
        b.nullField("divide_by");
      }
      if (mul > 0) {
        b.field("multiply_by", mul);
      } else {
        b.nullField("multiply_by");
      }
      b.field("invert", clk->invert());

      // -edges form: edge indices into the master-clock waveform.
      const sta::IntSeq& edges = clk->edges();
      if (!edges.empty()) {
        b.beginArray("edges");
        for (int e : edges) {
          b.value(e);
        }
        b.endArray();
      } else {
        b.nullField("edges");
      }

      // -edge_shift values paired with -edges.
      const sta::FloatSeq& shifts = clk->edgeShifts();
      if (!shifts.empty()) {
        b.beginArray("edge_shifts");
        for (float s : shifts) {
          b.value(scaleTime(s, time_scale));
        }
        b.endArray();
      } else {
        b.nullField("edge_shifts");
      }

      // Source pin (the design pin where this generated clock originates).
      sta::Pin* src = clk->srcPin();
      if (src && network) {
        b.field("src_pin", std::string(network->pathName(src)));
        emitPinOdbRef(b, "src_pin", src, db_network);
      } else {
        b.nullField("src_pin");
      }
    } else {
      // Primary clock — emit null placeholders so the schema is uniform.
      b.nullField("divide_by");
      b.nullField("multiply_by");
      b.field("invert", false);
      b.nullField("edges");
      b.nullField("edge_shifts");
      b.nullField("src_pin");
    }

    // Source pins (the design pins this clock is defined on).
    // Parallel arrays: sources[i] is the display name, sources_odb[i] is
    // the ODB handle {type, id} — or null when the pin couldn't be
    // resolved. Using parallel arrays (rather than an array of objects)
    // keeps existing consumers of `sources[i]` working unchanged.
    b.beginArray("sources");
    if (network) {
      for (const sta::Pin* pin : clk->pins()) {
        if (pin) {
          b.value(std::string(network->pathName(pin)));
        }
      }
    }
    b.endArray();
    b.beginArray("sources_odb");
    if (network) {
      for (const sta::Pin* pin : clk->pins()) {
        if (!pin) {
          continue;
        }
        b.beginObject();
        emitPinOdbBare(b, pin, db_network);
        b.endObject();
      }
    }
    b.endArray();

    // Uncertainty from set_clock_uncertainty for this clock.
    emitUncertainty(b,
                    "uncertainty_setup",
                    "uncertainty_hold",
                    extractUncertainty(&clk->uncertainties()),
                    time_scale);

    b.endObject();
  }
  b.endArray();

  // ── "clock_tree" array — root clocks with recursive generated children ──
  b.beginArray("clock_tree");
  for (sta::Clock* clk : clocks) {
    if (!clk) {
      continue;
    }
    if (!clk->masterClk()) {
      emitTreeNode(b, clk, childrenMap);
    }
  }
  b.endArray();

  b.endObject();
  return jsonResponse(req.id, b.str());
}

WebSocketResponse SdcHandler::handleSdcClockModes(const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return emptyModes(req.id);
  }
  sta::Sdc* sdc = ctx->sdc;
  sta::Scene* scene = ctx->scene;
  sta::Network* network = ctx->network;
  sta::dbNetwork* db_network = ctx->db_network;

  std::string mode_name;
  sta::Mode* mode = ctx->sta->cmdMode();
  if (mode) {
    mode_name = mode->name();
  }

  JsonBuilder b;
  b.beginObject();
  b.field("current_mode", mode_name);

  // Active scene name only (full MMMC scene enumeration is not exposed yet).
  b.beginArray("scene_names");
  b.value(scene->name());
  b.endArray();

  // Case-analysis values (set_case_analysis).
  // set_case_analysis: constants pinned on mux selects, etc.
  b.beginArray("case_analysis");
  for (const auto& [pin, val] : sdc->caseLogicValues()) {
    if (!pin) {
      continue;
    }
    b.beginObject();
    if (network) {
      b.field("pin", std::string(network->pathName(pin)));
    } else {
      b.field("pin", "");
    }
    b.field("value", logicValueStr(val));
    emitPinOdbRef(b, "pin", pin, db_network);
    b.endObject();
  }
  b.endArray();

  // set_logic_zero / set_logic_one / set_logic_dc — separate Sdc map but
  // emitted alongside case_analysis since the UI displays them together.
  b.beginArray("logic_values");
  for (const auto& [pin, val] : sdc->logicValues()) {
    if (!pin) {
      continue;
    }
    b.beginObject();
    if (network) {
      b.field("pin", std::string(network->pathName(pin)));
    } else {
      b.field("pin", "");
    }
    emitPinOdbRef(b, "pin", pin, db_network);
    b.field("value", logicValueStr(val));
    b.endObject();
  }
  b.endArray();

  b.endObject();
  return jsonResponse(req.id, b.str());
}

WebSocketResponse SdcHandler::handleSdcPortDelays(const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return emptyPortDelays(req.id);
  }
  sta::Sdc* sdc = ctx->sdc;
  sta::Network* network = ctx->network;
  sta::dbNetwork* db_network = ctx->db_network;
  const float time_scale = ctx->time_scale;
  const std::string& time_suffix = ctx->time_suffix;
  const float cap_scale = ctx->cap_scale;
  const std::string& cap_suffix = ctx->cap_suffix;

  // Build ODB port-order index and direction map so we can emit in floorplan
  // order and report the correct port direction.
  odb::dbBlock* block = gen_ ? gen_->getBlock() : nullptr;
  std::map<std::string, int> portOrder;
  std::map<std::string, std::string> portDir;
  if (block) {
    int idx = 0;
    for (odb::dbBTerm* bterm : block->getBTerms()) {
      const std::string pname = bterm->getName();
      portOrder[pname] = idx++;
      const odb::dbIoType io = bterm->getIoType();
      portDir[pname] = (io == odb::dbIoType::OUTPUT)  ? "output"
                       : (io == odb::dbIoType::INOUT) ? "inout"
                                                      : "input";
    }
  }

  // Collect all delays into a sortable list so we emit in ODB port order.
  // pd == nullptr → exception-only port (referenced by an SDC exception
  // such as set_false_path but with no set_input_delay / set_output_delay
  // attached); the row carries no diagram data, just the header + the
  // collapsible "Applicable Exceptions" toggle.
  struct RawDelay
  {
    sta::PortDelay* pd;
    const sta::Pin* pin;
    bool is_input;
    std::string port_name;
    int order;
  };
  std::vector<RawDelay> raw;
  std::set<std::string> emitted_names;  // port names already in `raw`
  // Per-top-level-port count of how many distinct exceptions reference
  // it. Used to suppress the "Applicable Exceptions" toggle on the
  // frontend for ports that have no exceptions attached.
  std::map<std::string, int> port_exc_count;
  if (network) {
    for (sta::InputDelay* id : sdc->inputDelays()) {
      if (!id || !id->pin()) {
        continue;
      }
      const std::string nm = std::string(network->pathName(id->pin()));
      raw.push_back({id,
                     id->pin(),
                     true,
                     nm,
                     portOrder.count(nm) ? portOrder.at(nm) : 999999});
      emitted_names.insert(nm);
    }
    for (sta::OutputDelay* od : sdc->outputDelays()) {
      if (!od || !od->pin()) {
        continue;
      }
      const std::string nm = std::string(network->pathName(od->pin()));
      raw.push_back({od,
                     od->pin(),
                     false,
                     nm,
                     portOrder.count(nm) ? portOrder.at(nm) : 999999});
      emitted_names.insert(nm);
    }

    // Walk every SDC exception's from / thru / to pin sets and:
    //   • count, per top-level port, how many distinct exceptions hit it
    //     (same exception can appear in from + to but is counted once);
    //   • add a delay-less row for any exception-referenced port that
    //     doesn't already have a set_input/output_delay entry, so
    //     false-pathed (or otherwise exception-attached) ports stay
    //     visible on the Port Delays tab.
    std::set<std::pair<std::string, int>> seen_port_exc;  // (port_name, exc_id)
    auto noteExcPin = [&](sta::ExceptionPath* exc, const sta::Pin* pin) {
      if (!pin || !network->isTopLevelPort(pin)) {
        return;
      }
      const std::string nm = std::string(network->pathName(pin));
      // Dedupe (port, exception) pairs so an exception listing the same
      // port in both from and to is counted once.
      const int id = exc ? static_cast<int>(exc->id()) : 0;
      if (!seen_port_exc.insert({nm, id}).second) {
        return;
      }
      ++port_exc_count[nm];
      // First time we see this port via an exception → if it has no
      // delay row, synthesise an exception-only one.
      if (emitted_names.count(nm)) {
        return;
      }
      const bool is_input
          = portDir.count(nm) ? portDir.at(nm) == "input" : true;
      raw.push_back({nullptr,
                     pin,
                     is_input,
                     nm,
                     portOrder.count(nm) ? portOrder.at(nm) : 999999});
      emitted_names.insert(nm);
    };
    auto walkPinSet = [&](sta::ExceptionPath* exc, const sta::PinSet* pins) {
      if (!pins) {
        return;
      }
      for (const sta::Pin* pin : *pins) {
        noteExcPin(exc, pin);
      }
    };
    for (sta::ExceptionPath* exc : sdc->exceptions()) {
      if (!exc || exc->isFilter() || exc->isLoop()) {
        continue;
      }
      if (auto from = exc->from()) {
        walkPinSet(exc, from->pins());
      }
      if (auto to = exc->to()) {
        walkPinSet(exc, to->pins());
      }
      if (auto thrus = exc->thrus()) {
        for (auto thru : *thrus) {
          if (thru) {
            walkPinSet(exc, thru->pins());
          }
        }
      }
    }

    // Third pass: every remaining signal port. Surfacing every port
    // (not just ones with set_load / set_driving_cell) means the user
    // can scan the full IO at a glance — unconstrained ports show up
    // as such, and any external attribute (set_load, set_drive,
    // set_input_transition, set_fanout_load) renders inline in the
    // header. Power and ground BTerms are excluded — they're never
    // timed and would just be noise on the Port Delays tab.
    if (block) {
      for (odb::dbBTerm* bterm : block->getBTerms()) {
        const odb::dbSigType st = bterm->getSigType();
        if (st == odb::dbSigType::POWER || st == odb::dbSigType::GROUND) {
          continue;
        }
        const std::string nm = bterm->getName();
        if (emitted_names.count(nm)) {
          continue;
        }
        sta::Pin* p = db_network->dbToSta(bterm);
        if (!p) {
          continue;
        }
        const bool is_input
            = portDir.count(nm) ? portDir.at(nm) != "output" : true;
        raw.push_back({nullptr,
                       p,
                       is_input,
                       nm,
                       portOrder.count(nm) ? portOrder.at(nm) : 999999});
        emitted_names.insert(nm);
      }
    }
  }
  std::stable_sort(
      raw.begin(), raw.end(), [](const RawDelay& a, const RawDelay& b) {
        return a.order < b.order;
      });

  // Pagination: clamp offset/limit to the assembled raw list.
  // limit < 0 → "no limit" (legacy behavior).
  const int total = static_cast<int>(raw.size());
  int offset = extract_int_or(req.raw_json, "offset", 0);
  if (offset < 0) {
    offset = 0;
  }
  if (offset > total) {
    offset = total;
  }
  int limit = extract_int_or(req.raw_json, "limit", -1);
  if (limit < 0) {
    limit = total - offset;
  }
  int end = offset + limit;
  if (end > total) {
    end = total;
  }

  // Per-clock totals across the *full* raw list so the frontend
  // checkbox dropdown shows the complete picture even mid-pagination.
  // Entries without a clock (exception-only ports, or set_*_delay
  // calls that didn't take -clock) bucket under the literal sentinel
  // "__none__" — mirrors the key the frontend's
  // _makeClockCheckboxFilter helper uses for its "(no clock)" row,
  // so no translation is needed on the JS side.
  std::map<std::string, int> clocks_total;
  // Per-direction totals across the *full* raw list, deduped by port
  // name so a port that appears in multiple raw entries (e.g. a flop
  // with both rise and fall set_input_delay) counts once. The frontend
  // uses this to label the direction filter buttons with stable
  // counts across paginated batches — without it, the labels grew as
  // more pages loaded, which made it look like ports were appearing
  // out of nowhere as the user scrolled.
  std::map<std::string, std::set<std::string>> dir_total_ports;
  for (const RawDelay& rd : raw) {
    sta::Clock* clk = rd.pd ? rd.pd->clock() : nullptr;
    const std::string key
        = clk ? std::string(clk->name()) : std::string("__none__");
    clocks_total[key] += 1;
    const std::string dir = portDir.count(rd.port_name)
                                ? portDir.at(rd.port_name)
                                : (rd.is_input ? "input" : "output");
    dir_total_ports[dir].insert(rd.port_name);
  }

  JsonBuilder b;
  b.beginObject();
  b.field("time_unit", time_suffix);
  b.field("cap_unit", cap_suffix);
  b.field("total", total);
  b.field("offset", offset);
  b.beginObject("clocks_total");
  for (const auto& [name, count] : clocks_total) {
    b.field(name, count);
  }
  b.endObject();
  // Direction totals — deduped port counts per direction, plus an
  // "all" key for the matching filter button.
  b.beginObject("directions_total");
  int all_count = 0;
  for (const auto& [dir, ports] : dir_total_ports) {
    b.field(dir, static_cast<int>(ports.size()));
    all_count += static_cast<int>(ports.size());
  }
  b.field("all", all_count);
  b.endObject();
  b.beginArray("port_delays");

  // Emit one delay entry. Both rise/fall × min/max delay values are included
  // so the frontend can show the full constraint band. When `rd.pd` is null
  // the row is an "exception-only port" (no set_input/output_delay; the
  // port shows up because it's referenced by an SDC exception); we emit
  // every delay/clock/driving field as null and tag the row with
  // `exception_only: true` so the frontend skips drawing a diagram.
  auto emitDelay = [&](const RawDelay& rd) {
    sta::PortDelay* pd = rd.pd;
    bool is_input = rd.is_input;
    const sta::Pin* pin = rd.pin;
    if (!pin) {
      return;
    }
    const bool is_exception_only = (pd == nullptr);

    b.beginObject();
    b.field("port", rd.port_name);
    emitPinOdbRef(b, "port", pin, db_network);
    b.field("is_input", is_input);
    b.field("direction",
            portDir.count(rd.port_name) ? portDir.at(rd.port_name)
                                        : (is_input ? "input" : "output"));
    b.field("exception_only", is_exception_only);
    // Clock-source ports (the top-level pin a `create_clock` anchors
    // on) carry no set_input_delay / set_output_delay because they
    // launch the clock rather than being timed against one. They'd
    // otherwise show up as "unconstrained" — emit `is_clock_port: true`
    // so the frontend renders a "clock" pill instead of the
    // unconstrained label.
    const bool is_clock_port = sdc && pin && sdc->isClock(pin);
    b.field("is_clock_port", is_clock_port);
    // is_unconstrained tells the frontend "no delay, no exception
    // reference, no set_load, no set_driving_cell, and not a
    // clock-source port — the row is emitted purely so every signal
    // port is visible". The frontend labels it as such instead of
    // rendering nothing.
    bool unconstrained = false;
    if (is_exception_only) {
      sta::Port* port_un = network ? network->port(pin) : nullptr;
      const bool has_extcap = port_un ? sdc->hasPortExtCap(port_un) : false;
      const bool has_drive
          = port_un ? (sdc->findInputDrive(port_un) != nullptr) : false;
      const bool has_exc = port_exc_count.count(rd.port_name)
                           && port_exc_count.at(rd.port_name) > 0;
      unconstrained = !has_extcap && !has_drive && !has_exc && !is_clock_port;
    }
    b.field("is_unconstrained", unconstrained);
    // Number of distinct SDC exceptions that reference this port. Lets
    // the frontend suppress the "Applicable Exceptions" toggle when the
    // port has none, instead of fetching just to discover that.
    b.field("exception_count",
            port_exc_count.count(rd.port_name) ? port_exc_count.at(rd.port_name)
                                               : 0);
    // Delay-related fields (PortDelay-derived). Delay-less rows
    // (exception-only or set_load-only) emit nulls here and skip
    // straight to the load/drive emission below — those *are*
    // populated for delay-less rows so set_load and set_driving_cell
    // remain visible without requiring a paired delay constraint.
    if (is_exception_only) {
      b.nullField("source_latency_included");
      b.nullField("network_latency_included");
      b.nullField("ref_pin");
      b.nullField("clock");
      b.nullField("clk_period");
      b.nullField("uncertainty_setup");
      b.nullField("uncertainty_hold");
      b.nullField("clk_edge");
      b.nullField("rise_max");
      b.nullField("rise_min");
      b.nullField("fall_max");
      b.nullField("fall_min");
    } else {
      // Reference info from set_input_delay / set_output_delay:
      //   -source_latency_included / -network_latency_included flags affect
      //   how the delay value is interpreted relative to clock latency.
      //   -reference_pin <pin> uses an alternate pin's clock instead of the
      //   default reference clock — surface its name + ODB ref so the user
      //   can click through.
      b.field("source_latency_included", pd->sourceLatencyIncluded());
      b.field("network_latency_included", pd->networkLatencyIncluded());
      if (const sta::Pin* ref = pd->refPin()) {
        b.field("ref_pin", network ? std::string(network->pathName(ref)) : "");
        emitPinOdbRef(b, "ref_pin", ref, db_network);
      } else {
        b.nullField("ref_pin");
      }

      // Reference clock and edge.
      sta::Clock* clk = pd->clock();
      if (clk) {
        b.field("clock", std::string(clk->name()));
        b.field("clk_period", scaleTime(clk->period(), time_scale));
        emitUncertainty(b,
                        "uncertainty_setup",
                        "uncertainty_hold",
                        extractUncertainty(&clk->uncertainties()),
                        time_scale);
      } else {
        b.nullField("clock");
        b.nullField("clk_period");
        b.nullField("uncertainty_setup");
        b.nullField("uncertainty_hold");
      }

      const sta::ClockEdge* clk_edge = pd->clkEdge();
      if (clk_edge) {
        b.field("clk_edge", std::string(clk_edge->transition()->name()));
      } else {
        b.nullField("clk_edge");
      }

      emitRiseFallMinMax(b, pd->delays(), time_scale);
    }

    // Driving cell + drive resistance + input transition slews (input ports
    // only). All three live on the same InputDrive object.
    //   driving_cell  : set_driving_cell <cell>          → cell + to_port name
    //   drive_res_*   : set_drive <res>                  → output resistance
    //   drive_slew_*  : set_input_transition <slew>      → input slew
    // We emit drive_res / drive_slew as a 4-cell rise/fall × min/max product
    // so asymmetric values (e.g. set_drive -rise X -fall Y) survive into the
    // JSON. The frontend collapses this to a single line on the Port Delays
    // card unless the user explicitly cares about asymmetry.
    sta::Port* port = network->port(pin);
    if (port && is_input) {
      sta::InputDrive* drive = sdc->findInputDrive(port);
      if (drive) {
        const sta::LibertyCell* cell = nullptr;
        const sta::LibertyPort* from_port = nullptr;
        const sta::DriveCellSlews* from_slews = nullptr;
        const sta::LibertyPort* to_port = nullptr;
        drive->driveCell(sta::RiseFall::rise(),
                         sta::MinMax::max(),
                         cell,
                         from_port,
                         from_slews,
                         to_port);
        if (cell) {
          b.field("driving_cell", std::string(cell->name()));
          if (to_port) {
            b.field("driving_pin", std::string(to_port->name()));
          } else {
            b.nullField("driving_pin");
          }
        } else {
          b.nullField("driving_cell");
          b.nullField("driving_pin");
        }
        // Helper to emit a 4-cell rise/fall × min/max group with a shared key
        // prefix and per-key suffixes ("rise_max", "rise_min", "fall_max",
        // "fall_min"). Time scale or 1.0 (for resistance ohms — no SI scaling
        // applied here).
        auto emitRfmm = [&](const char* prefix, auto query, double scale) {
          const struct
          {
            const char* suffix;
            const sta::RiseFall* rf;
            const sta::MinMax* mm;
          } slots[] = {
              {"rise_max", sta::RiseFall::rise(), sta::MinMax::max()},
              {"rise_min", sta::RiseFall::rise(), sta::MinMax::min()},
              {"fall_max", sta::RiseFall::fall(), sta::MinMax::max()},
              {"fall_min", sta::RiseFall::fall(), sta::MinMax::min()},
          };
          for (const auto& s : slots) {
            float val = 0.0f;
            bool exists = false;
            query(s.rf, s.mm, val, exists);
            const std::string key = std::string(prefix) + s.suffix;
            if (exists) {
              b.field(key, static_cast<double>(val) / scale);
            } else {
              b.nullField(key);
            }
          }
        };
        emitRfmm(
            "drive_slew_",
            [&](const sta::RiseFall* rf,
                const sta::MinMax* mm,
                float& v,
                bool& ex) { drive->slew(rf, mm, v, ex); },
            static_cast<double>(time_scale));
        emitRfmm(
            "drive_res_",
            [&](const sta::RiseFall* rf,
                const sta::MinMax* mm,
                float& v,
                bool& ex) { drive->driveResistance(rf, mm, v, ex); },
            1.0);  // resistance: ohms, no scaling
      } else {
        b.nullField("driving_cell");
        b.nullField("driving_pin");
        // Symmetry: emit nulls for the 4-cell groups when no InputDrive.
        for (const char* k : {"drive_slew_rise_max",
                              "drive_slew_rise_min",
                              "drive_slew_fall_max",
                              "drive_slew_fall_min",
                              "drive_res_rise_max",
                              "drive_res_rise_min",
                              "drive_res_fall_max",
                              "drive_res_fall_min"}) {
          b.nullField(k);
        }
      }
    } else {
      b.nullField("driving_cell");
      b.nullField("driving_pin");
      for (const char* k : {"drive_slew_rise_max",
                            "drive_slew_rise_min",
                            "drive_slew_fall_max",
                            "drive_slew_fall_min",
                            "drive_res_rise_max",
                            "drive_res_rise_min",
                            "drive_res_fall_max",
                            "drive_res_fall_min"}) {
        b.nullField(k);
      }
    }

    // Load capacitance, wire cap, and fanout (all port types).
    //   load_cap : set_load <cap> [get_ports …]                — pin-level
    //   wire_cap : set_load -wire <cap> [get_ports …]          — net-level
    //   fanout   : set_fanout_load <load> [get_ports …]        — load count
    // Each is rise/max biased to keep the JSON simple; users almost never
    // set rise vs. fall asymmetrically on these.
    const sta::PortExtCap* ext_cap = port ? sdc->portExtCap(port) : nullptr;
    if (ext_cap) {
      float pinc = 0.0f, wirec = 0.0f;
      int fan = 0;
      bool pinc_ex = false, wirec_ex = false, fan_ex = false;
      ext_cap->pinCap(sta::RiseFall::rise(), sta::MinMax::max(), pinc, pinc_ex);
      ext_cap->wireCap(
          sta::RiseFall::rise(), sta::MinMax::max(), wirec, wirec_ex);
      ext_cap->fanout(sta::MinMax::max(), fan, fan_ex);
      if (pinc_ex) {
        b.field("load_cap", static_cast<double>(pinc / cap_scale));
      } else {
        b.nullField("load_cap");
      }
      if (wirec_ex) {
        b.field("wire_cap", static_cast<double>(wirec / cap_scale));
      } else {
        b.nullField("wire_cap");
      }
      if (fan_ex) {
        b.field("fanout_load", fan);
      } else {
        b.nullField("fanout_load");
      }
    } else {
      b.nullField("load_cap");
      b.nullField("wire_cap");
      b.nullField("fanout_load");
    }

    b.endObject();
  };

  for (int i = offset; i < end; ++i) {
    emitDelay(raw[i]);
  }

  b.endArray();
  b.endObject();
  return jsonResponse(req.id, b.str());
}

WebSocketResponse SdcHandler::handleSdcExceptions(const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return emptyExceptions(req.id);
  }
  sta::Sdc* sdc = ctx->sdc;
  sta::Network* network = ctx->network;
  sta::dbNetwork* db_network = ctx->db_network;
  const float time_scale = ctx->time_scale;
  const std::string& time_suffix = ctx->time_suffix;

  // Emit a PinSet as an array of {name, odb_type?, odb_id?} objects. No
  // truncation — wide exceptions list every pin so the user can search for
  // specific names.
  auto emitPins
      = [&](JsonBuilder& b, const char* key, const sta::PinSet* pins) {
          b.beginArray(key);
          if (pins && network) {
            for (const sta::Pin* pin : *pins) {
              if (!pin) {
                continue;
              }
              b.beginObject();
              b.field("name", std::string(network->pathName(pin)));
              emitPinOdbBare(b, pin, db_network);
              b.endObject();
            }
          }
          b.endArray();
        };

  auto emitClocks
      = [&](JsonBuilder& b, const char* key, const sta::ClockSet* clks) {
          b.beginArray(key);
          if (clks) {
            for (sta::Clock* clk : *clks) {
              if (clk) {
                b.value(std::string(clk->name()));
              }
            }
          }
          b.endArray();
        };

  // Emit every instance from an InstanceSet as an array of {name, odb_*}
  // objects. Mirrors emitPins so the frontend can linkify hierarchical
  // instances exactly like pins.
  auto emitInsts
      = [&](JsonBuilder& b, const char* key, const sta::InstanceSet* insts) {
          b.beginArray(key);
          if (insts && network) {
            for (const sta::Instance* inst : *insts) {
              if (!inst) {
                continue;
              }
              b.beginObject();
              b.field("name", std::string(network->pathName(inst)));
              emitInstanceOdbBare(b, inst, db_network);
              b.endObject();
            }
          }
          b.endArray();
        };

  // Nets in a -thru list (set_*_path -through { net1 net2 ... }). Names only;
  // odb has dbNet but the SDC widget doesn't currently linkify nets, so we
  // emit just the path name. (Adding ODB refs later is straightforward.)
  auto emitNets
      = [&](JsonBuilder& b, const char* key, const sta::NetSet* nets) {
          b.beginArray(key);
          if (nets && network) {
            for (const sta::Net* net : *nets) {
              if (net) {
                b.value(std::string(network->pathName(net)));
              }
            }
          }
          b.endArray();
        };

  // Emit a RiseFallBoth* edge specifier as "rise" / "fall" / "rise_fall".
  // Null when there's no transition restriction (plain -from / -to).
  auto emitTransition
      = [&](JsonBuilder& b, const char* key, const sta::RiseFallBoth* rf) {
          if (rf) {
            b.field(key, rf->name());
          } else {
            b.nullField(key);
          }
        };

  JsonBuilder b;
  b.beginObject();
  b.field("time_unit", time_suffix);
  b.beginArray("exceptions");

  for (sta::ExceptionPath* exc : sdc->exceptions()) {
    if (!exc) {
      continue;
    }
    // Skip internal filter/loop paths — they are not user-visible SDC.
    // Group paths (set_group_path) ARE user-visible and surface as a distinct
    // exception subtype below.
    if (exc->isFilter() || exc->isLoop()) {
      continue;
    }

    b.beginObject();
    b.field("type", exceptionTypeStr(exc));

    // Group path name (only meaningful when type == "group_path"; empty
    // otherwise).
    if (exc->isGroupPath()) {
      b.field("name", std::string(exc->name()));
      b.field("is_default", exc->isDefault());
    } else {
      b.nullField("name");
      b.field("is_default", false);
    }

    // Setup/hold/both scope.
    const sta::MinMaxAll* mm = exc->minMax();
    b.field("min_max", mm ? mm->to_string() : std::string("max"));

    // Multi-cycle multiplier (0 for false paths and path delays).
    b.field("multiplier", exc->pathMultiplier());

    // Explicit delay value (only meaningful for path_delay type).
    if (exc->isPathDelay()) {
      b.field("delay", scaleTime(exc->delay(), time_scale));
    } else {
      b.nullField("delay");
    }

    // path_delay-only flags: -ignore_clock_latency and -break_path.
    // Defined as virtuals on ExceptionPath returning false on non-PathDelay
    // subclasses, so we can call them unconditionally.
    b.field("ignore_clk_latency", exc->ignoreClkLatency());
    b.field("break_path", exc->breakPath());

    // multicycle_path -end / -start: when -end, MCP applies relative to the
    // capture-clock cycles; otherwise relative to launch. Returns false on
    // non-MCP subclasses so it's safe to call unconditionally.
    b.field("use_end_clk", exc->useEndClk());

    // -from endpoints (pins / clocks / instances) plus optional -rise_from /
    // -fall_from transition selector.
    sta::ExceptionFrom* from = exc->from();
    emitPins(b, "from_pins", from ? from->pins() : nullptr);
    emitClocks(b, "from_clocks", from ? from->clks() : nullptr);
    emitInsts(b, "from_insts", from ? from->instances() : nullptr);
    emitTransition(b, "from_transition", from ? from->transition() : nullptr);

    // -thru endpoints (ordered list of filter stages). Each stage may
    // independently restrict by -rise_through / -fall_through, and may
    // contain pins, clocks, instances, AND nets (-through { net1 net2 }).
    b.beginArray("thrus");
    if (exc->thrus()) {
      for (sta::ExceptionThru* thru : *exc->thrus()) {
        if (!thru) {
          continue;
        }
        b.beginObject();
        emitPins(b, "pins", thru->pins());
        emitClocks(b, "clocks", thru->clks());
        emitInsts(b, "insts", thru->instances());
        emitNets(b, "nets", thru->nets());
        emitTransition(b, "transition", thru->transition());
        b.endObject();
      }
    }
    b.endArray();

    // -to endpoints (pins / clocks / instances) plus optional -rise_to /
    // -fall_to via endTransition().
    sta::ExceptionTo* to = exc->to();
    emitPins(b, "to_pins", to ? to->pins() : nullptr);
    emitClocks(b, "to_clocks", to ? to->clks() : nullptr);
    emitInsts(b, "to_insts", to ? to->instances() : nullptr);
    emitTransition(b, "to_transition", to ? to->endTransition() : nullptr);

    // Unique ID for stable ordering in the UI.
    b.field("id", static_cast<int>(exc->id()));
    // SDC -comment carried on every exception (false_path, multi_cycle,
    // path_delay, group_path).  Empty string when none, kept in the
    // payload so the frontend can decide whether to render it.
    b.field("comment", exc->comment());

    b.endObject();
  }

  b.endArray();
  b.endObject();
  return jsonResponse(req.id, b.str());
}

// ── Network latencies (set_clock_latency without -source). Source
//    latencies emit below as clock_insertions; is_source discriminates.
static void emitClockLatencies(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("clock_latencies");
  // Iterate non-const because ClockLatency::delays() is non-const upstream.
  for (sta::ClockLatency* lat : *ctx.sdc->clockLatencies()) {
    if (!lat) {
      continue;
    }
    b.beginObject();
    const sta::Clock* clk = lat->clock();
    if (clk) {
      b.field("clock", std::string(clk->name()));
    } else {
      b.nullField("clock");
    }
    const sta::Pin* pin = lat->pin();
    if (pin && ctx.network) {
      b.field("pin", std::string(ctx.network->pathName(pin)));
    } else {
      b.nullField("pin");
    }
    b.field("is_source", false);
    emitRiseFallMinMax(b, lat->delays(), ctx.time_scale);
    b.endObject();
  }
  b.endArray();
}

// ── Clock insertion delays (set_clock_latency -source) — distinct from
//    network latency: source latency is the off-chip / pre-netlist tree
//    delay, with rise/fall × min/max × early/late values.
static void emitClockInsertions(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("clock_insertions");
  for (sta::ClockInsertion* ins : ctx.sdc->clockInsertions()) {
    if (!ins) {
      continue;
    }
    b.beginObject();
    const sta::Clock* clk = ins->clock();
    if (clk) {
      b.field("clock", std::string(clk->name()));
    } else {
      b.nullField("clock");
    }
    const sta::Pin* pin = ins->pin();
    if (pin && ctx.network) {
      b.field("pin", std::string(ctx.network->pathName(pin)));
    } else {
      b.nullField("pin");
    }
    // Counterpart of the `is_source: false` flag on clock_latencies — these
    // entries are always source latency (set_clock_latency -source).
    b.field("is_source", true);
    // Flatten early/late into one early_*/late_* prefix per direction so the
    // JSON is self-describing without requiring the client to know the full
    // RiseFall × MinMax × EarlyLate product.
    const struct
    {
      const char* key;
      const sta::RiseFall* rf;
      const sta::MinMax* mm;
      const sta::EarlyLate* el;
    } slots[] = {
        {"early_rise_max",
         sta::RiseFall::rise(),
         sta::MinMax::max(),
         sta::EarlyLate::early()},
        {"early_rise_min",
         sta::RiseFall::rise(),
         sta::MinMax::min(),
         sta::EarlyLate::early()},
        {"early_fall_max",
         sta::RiseFall::fall(),
         sta::MinMax::max(),
         sta::EarlyLate::early()},
        {"early_fall_min",
         sta::RiseFall::fall(),
         sta::MinMax::min(),
         sta::EarlyLate::early()},
        {"late_rise_max",
         sta::RiseFall::rise(),
         sta::MinMax::max(),
         sta::EarlyLate::late()},
        {"late_rise_min",
         sta::RiseFall::rise(),
         sta::MinMax::min(),
         sta::EarlyLate::late()},
        {"late_fall_max",
         sta::RiseFall::fall(),
         sta::MinMax::max(),
         sta::EarlyLate::late()},
        {"late_fall_min",
         sta::RiseFall::fall(),
         sta::MinMax::min(),
         sta::EarlyLate::late()},
    };
    for (const auto& s : slots) {
      float val;
      bool exists;
      ins->delay(s.rf, s.mm, s.el, val, exists);
      if (exists) {
        b.field(s.key, scaleTime(val, ctx.time_scale));
      } else {
        b.nullField(s.key);
      }
    }
    b.endObject();
  }
  b.endArray();
}

// ── Per-clock uncertainties (set_clock_uncertainty -clock) ──────────────
static void emitClockUncertainties(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("clock_uncertainties");
  for (const sta::Clock* clk : ctx.sdc->clocks()) {
    if (!clk) {
      continue;
    }
    UncertaintyValues u = extractUncertainty(&clk->uncertainties());
    if (!u.any()) {
      continue;
    }
    b.beginObject();
    b.field("clock", std::string(clk->name()));
    emitUncertainty(b, "setup", "hold", u, ctx.time_scale);
    b.endObject();
  }
  b.endArray();
}

// ── Top-level port loads and limits ─────────────────────────────────────
static void emitPortLoads(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("port_loads");
  if (ctx.network) {
    const sta::Instance* top = ctx.network->topInstance();
    if (top) {
      const sta::Cell* top_cell = ctx.network->cell(top);
      // unique_ptr so the iterator is freed even if a body throws.
      std::unique_ptr<sta::CellPortIterator> it(
          ctx.network->portIterator(top_cell));
      while (it->hasNext()) {
        sta::Port* port = it->next();
        if (!port) {
          continue;
        }

        // Check if any constraint is set for this port.
        const sta::PortExtCap* ext_cap = ctx.sdc->portExtCap(port);
        float slew_max = 0.0f, slew_min = 0.0f, cap_max = 0.0f,
              fanout_max = 0.0f;
        bool slew_max_ex = false, slew_min_ex = false, cap_max_ex = false,
             fanout_max_ex = false;
        ctx.sdc->slewLimit(port, sta::MinMax::max(), slew_max, slew_max_ex);
        ctx.sdc->slewLimit(port, sta::MinMax::min(), slew_min, slew_min_ex);
        ctx.sdc->capacitanceLimit(
            port, sta::MinMax::max(), cap_max, cap_max_ex);
        ctx.sdc->fanoutLimit(
            port, sta::MinMax::max(), fanout_max, fanout_max_ex);

        if (!ext_cap && !slew_max_ex && !slew_min_ex && !cap_max_ex
            && !fanout_max_ex) {
          continue;
        }

        b.beginObject();
        b.field("port", std::string(ctx.network->name(port)));

        // External load capacitance (set_load).
        if (ext_cap) {
          const sta::RiseFallMinMax* pin_cap = ext_cap->pinCap();
          const sta::RiseFallMinMax* wire_cap = ext_cap->wireCap();
          emitRiseFallMinMax(b, pin_cap, ctx.cap_scale);
          // Emit wire cap only as a single max value for compactness.
          float wc;
          bool wc_ex;
          if (wire_cap) {
            wire_cap->value(
                sta::RiseFall::rise(), sta::MinMax::max(), wc, wc_ex);
          } else {
            wc_ex = false;
          }
          if (wc_ex) {
            b.field("wire_cap_max", static_cast<double>(wc / ctx.cap_scale));
          } else {
            b.nullField("wire_cap_max");
          }
        } else {
          b.nullField("rise_max");
          b.nullField("rise_min");
          b.nullField("fall_max");
          b.nullField("fall_min");
          b.nullField("wire_cap_max");
        }

        // Slew limits.
        if (slew_max_ex) {
          b.field("slew_max", scaleTime(slew_max, ctx.time_scale));
        } else {
          b.nullField("slew_max");
        }
        if (slew_min_ex) {
          b.field("slew_min", scaleTime(slew_min, ctx.time_scale));
        } else {
          b.nullField("slew_min");
        }

        // Cap limit.
        if (cap_max_ex) {
          b.field("cap_limit", static_cast<double>(cap_max / ctx.cap_scale));
        } else {
          b.nullField("cap_limit");
        }

        // Fanout limit.
        if (fanout_max_ex) {
          b.field("fanout_limit", static_cast<int>(fanout_max));
        } else {
          b.nullField("fanout_limit");
        }

        b.endObject();
      }
    }
  }
  b.endArray();
}

// ── Disabled timing (set_disable_timing) ────────────────────────────────
// Flattened into one array with a `scope` discriminator so the frontend
// can render a single table. Covers every flavor that the Sdc API exposes
// publicly:
//   - scope="pin"         → set_disable_timing <instance pin>
//   - scope="port"        → set_disable_timing <top-level port>
//   - scope="lib_port"    → set_disable_timing <liberty_cell/port>
//   - scope="cell_port"   → set_disable_timing cell -from/-to (per cell,
//   lists from/to ports)
//   - scope="inst_port"   → set_disable_timing inst -from/-to (per instance,
//   lists from/to ports)
static void emitDisabledTiming(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("disabled_timing");
  auto emitPortList = [&](const char* key, const sta::LibertyPortSet* ports) {
    b.beginArray(key);
    if (ports) {
      for (const sta::LibertyPort* lp : *ports) {
        if (lp) {
          b.value(std::string(lp->name()));
        }
      }
    }
    b.endArray();
  };
  // Paired -from/-to disables (set_disable_timing -from X -to Y) live in a
  // separate LibertyPortPairSet, NOT in the single-port from/to lists.
  // Emit those as an array of {from, to} objects so the frontend can show
  // each pair as a row of its own.
  auto emitFromToPairs = [&](const sta::LibertyPortPairSet* pairs) {
    b.beginArray("from_to");
    if (pairs) {
      for (const sta::LibertyPortPair& p : *pairs) {
        b.beginObject();
        b.field("from", p.first ? std::string(p.first->name()) : std::string());
        b.field("to", p.second ? std::string(p.second->name()) : std::string());
        b.endObject();
      }
    }
    b.endArray();
  };
  auto emitDisabledPorts = [&](const sta::DisabledPorts* dp) {
    b.field("all", dp ? dp->all() : false);
    emitPortList("from", dp ? dp->from() : nullptr);
    emitPortList("to", dp ? dp->to() : nullptr);
    emitFromToPairs(dp ? dp->fromTo() : nullptr);
  };

  if (const sta::PinSet* pins = ctx.sdc->disabledPins()) {
    for (const sta::Pin* pin : *pins) {
      if (!pin) {
        continue;
      }
      b.beginObject();
      b.field("scope", std::string("pin"));
      if (ctx.network) {
        b.field("name", std::string(ctx.network->pathName(pin)));
      } else {
        b.field("name", std::string(""));
      }
      b.endObject();
    }
  }
  if (const sta::PortSet* ports = ctx.sdc->disabledPorts()) {
    for (const sta::Port* port : *ports) {
      if (!port) {
        continue;
      }
      b.beginObject();
      b.field("scope", std::string("port"));
      if (ctx.network) {
        b.field("name", std::string(ctx.network->name(port)));
      } else {
        b.field("name", std::string(""));
      }
      b.endObject();
    }
  }
  if (const sta::LibertyPortSet* lports = ctx.sdc->disabledLibPorts()) {
    for (const sta::LibertyPort* lp : *lports) {
      if (!lp) {
        continue;
      }
      b.beginObject();
      b.field("scope", std::string("lib_port"));
      b.field("name", std::string(lp->name()));
      b.endObject();
    }
  }
  // Helper: emit DisabledCellPorts::timingArcSets() if it carries any
  // arc-set granularity. Each arc-set's to_string() reads roughly like
  // "from_port → to_port (timing_role)" — handy for users tracking down
  // exactly which arc was disabled vs. the whole port pair.
  // Only DisabledCellPorts carries timingArcSets — DisabledInstancePorts
  // is per-instance and uses the inherited from/to/all only.
  auto emitTimingArcSets = [&](const sta::DisabledCellPorts* dcp) {
    const sta::TimingArcSetSet* sets = dcp ? dcp->timingArcSets() : nullptr;
    b.beginArray("timing_arc_sets");
    if (sets) {
      for (sta::TimingArcSet* arc : *sets) {
        if (arc) {
          b.value(arc->to_string());
        }
      }
    }
    b.endArray();
  };
  if (const sta::DisabledCellPortsMap* cm = ctx.sdc->disabledCellPorts()) {
    for (const auto& [cell, dcp] : *cm) {
      if (!cell || !dcp) {
        continue;
      }
      b.beginObject();
      b.field("scope", std::string("cell_port"));
      b.field("name", std::string(cell->name()));
      emitDisabledPorts(dcp);
      emitTimingArcSets(dcp);
      b.endObject();
    }
  }
  if (const sta::DisabledInstancePortsMap* im
      = ctx.sdc->disabledInstancePorts()) {
    for (const auto& [inst, dip] : *im) {
      if (!inst || !dip) {
        continue;
      }
      b.beginObject();
      b.field("scope", std::string("inst_port"));
      if (ctx.network) {
        b.field("name", std::string(ctx.network->pathName(inst)));
      } else {
        b.field("name", std::string(""));
      }
      emitDisabledPorts(dip);
      // DisabledInstancePorts has no timing_arc_sets accessor; emit empty
      // so the JSON shape stays uniform across all disabled-timing rows.
      b.beginArray("timing_arc_sets");
      b.endArray();
      b.endObject();
    }
  }
  b.endArray();
}

// ── Design-wide cell limits (set_max_transition / cap / fanout on the
//    top design) ───────────────────────────────────────────────────────
// Sdc only exposes per-Cell* query getters (no bulk map iterator), so
// we probe just the top cell. Most flows set these design-wide values
// exactly once via `set_max_* X [current_design]`, which lands here.
// Per-cell-name limits applied via `set_max_* X [get_cells <pat>]` would
// need a public Sdc map getter to enumerate — left as upstream follow-on.
static void emitCellLimits(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("cell_limits");
  if (ctx.network) {
    sta::Cell* top_cell = ctx.network->cell(ctx.network->topInstance());
    if (top_cell) {
      float slew_max, slew_min, cap_max, fanout_max;
      bool slew_max_ex, slew_min_ex, cap_max_ex, fanout_max_ex;
      ctx.sdc->slewLimit(top_cell, sta::MinMax::max(), slew_max, slew_max_ex);
      ctx.sdc->slewLimit(top_cell, sta::MinMax::min(), slew_min, slew_min_ex);
      ctx.sdc->capacitanceLimit(
          top_cell, sta::MinMax::max(), cap_max, cap_max_ex);
      ctx.sdc->fanoutLimit(
          top_cell, sta::MinMax::max(), fanout_max, fanout_max_ex);
      if (slew_max_ex || slew_min_ex || cap_max_ex || fanout_max_ex) {
        b.beginObject();
        b.field("scope", std::string("design"));
        b.field("name", std::string(ctx.network->name(top_cell)));
        if (slew_max_ex) {
          b.field("slew_max", scaleTime(slew_max, ctx.time_scale));
        } else {
          b.nullField("slew_max");
        }
        if (slew_min_ex) {
          b.field("slew_min", scaleTime(slew_min, ctx.time_scale));
        } else {
          b.nullField("slew_min");
        }
        if (cap_max_ex) {
          b.field("cap_limit", static_cast<double>(cap_max / ctx.cap_scale));
        } else {
          b.nullField("cap_limit");
        }
        if (fanout_max_ex) {
          b.field("fanout_limit", static_cast<int>(fanout_max));
        } else {
          b.nullField("fanout_limit");
        }
        b.endObject();
      }
    }
  }
  b.endArray();
}

// ── Clock-level slew limits (set_max_transition -clock_path/data_path) ──
// Only enumerate when haveClkSlewLimits() reports true; without that
// gate we'd query every clock × rise/fall × clk/data × min/max
// combination just to find out nothing is set.
static void emitClockSlewLimits(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("clock_slew_limits");
  if (ctx.sdc->haveClkSlewLimits()) {
    const sta::PathClkOrData clk_only = sta::PathClkOrData::clk;
    const sta::PathClkOrData data_only = sta::PathClkOrData::data;
    for (const sta::Clock* clk : ctx.sdc->clocks()) {
      if (!clk) {
        continue;
      }
      // Probe all 8 slots; emit a row per clock when at least one is set.
      const struct
      {
        const char* key;
        const sta::RiseFall* rf;
        sta::PathClkOrData cd;
        const sta::MinMax* mm;
      } slots[] = {
          {"clk_rise_max", sta::RiseFall::rise(), clk_only, sta::MinMax::max()},
          {"clk_rise_min", sta::RiseFall::rise(), clk_only, sta::MinMax::min()},
          {"clk_fall_max", sta::RiseFall::fall(), clk_only, sta::MinMax::max()},
          {"clk_fall_min", sta::RiseFall::fall(), clk_only, sta::MinMax::min()},
          {"data_rise_max",
           sta::RiseFall::rise(),
           data_only,
           sta::MinMax::max()},
          {"data_rise_min",
           sta::RiseFall::rise(),
           data_only,
           sta::MinMax::min()},
          {"data_fall_max",
           sta::RiseFall::fall(),
           data_only,
           sta::MinMax::max()},
          {"data_fall_min",
           sta::RiseFall::fall(),
           data_only,
           sta::MinMax::min()},
      };
      bool anyExists = false;
      float vals[8] = {};
      bool ex[8] = {};
      for (int i = 0; i < 8; ++i) {
        ctx.sdc->slewLimit(
            clk, slots[i].rf, slots[i].cd, slots[i].mm, vals[i], ex[i]);
        anyExists = anyExists || ex[i];
      }
      if (!anyExists) {
        continue;
      }
      b.beginObject();
      b.field("clock", std::string(clk->name()));
      for (int i = 0; i < 8; ++i) {
        if (ex[i]) {
          b.field(slots[i].key, scaleTime(vals[i], ctx.time_scale));
        } else {
          b.nullField(slots[i].key);
        }
      }
      b.endObject();
    }
  }
  b.endArray();
}

// ── Pin-anchored clock uncertainty (set_clock_uncertainty -to <pin>).
//    Same gate as pin_cap_limits below — graph-built only.
static void emitPinClockUncertainties(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("pin_clock_uncertainties");
  if (ctx.sta->graph()) {
    sta::Search* search = ctx.sta->search();
    if (search) {
      sta::VertexSet& endpoints = search->endpoints();
      for (sta::Vertex* v : endpoints) {
        if (!v) {
          continue;
        }
        const sta::Pin* pin = v->pin();
        if (!pin || !ctx.network) {
          continue;
        }
        UncertaintyValues u
            = extractUncertainty(ctx.sdc->clockUncertainties(pin));
        if (!u.any()) {
          continue;
        }
        b.beginObject();
        b.field("pin", std::string(ctx.network->pathName(pin)));
        emitPinOdbRef(b, "pin", pin, ctx.db_network);
        emitUncertainty(b, "setup", "hold", u, ctx.time_scale);
        b.endObject();
      }
    }
  }
  b.endArray();
}

// ── Pin-scope cap limits (set_max_capacitance [get_pins …]) ────────
// Probe every endpoint vertex via the public per-pin getter (no bulk
// getter exists upstream). Gated on the graph being built so opening
// the Limits tab doesn't trigger an O(V) walk by itself.
static void emitPinCapLimits(JsonBuilder& b, const SdcContext& ctx)
{
  b.beginArray("pin_cap_limits");
  if (ctx.sta->graph()) {
    sta::Search* search = ctx.sta->search();
    if (search) {
      sta::VertexSet& endpoints = search->endpoints();
      for (sta::Vertex* v : endpoints) {
        if (!v) {
          continue;
        }
        const sta::Pin* pin = v->pin();
        if (!pin || !ctx.network) {
          continue;
        }
        float cap_max = 0, cap_min = 0;
        bool cap_max_ex = false, cap_min_ex = false;
        // Sdc::capacitanceLimit(Pin*, ...) takes a non-const Pin* even
        // though it's a read-only query — the cast strips const for the API
        // boundary only.
        sta::Pin* mut_pin = const_cast<sta::Pin*>(pin);
        ctx.sdc->capacitanceLimit(
            mut_pin, sta::MinMax::max(), cap_max, cap_max_ex);
        ctx.sdc->capacitanceLimit(
            mut_pin, sta::MinMax::min(), cap_min, cap_min_ex);
        if (!cap_max_ex && !cap_min_ex) {
          continue;
        }
        b.beginObject();
        b.field("pin", std::string(ctx.network->pathName(pin)));
        emitPinOdbRef(b, "pin", pin, ctx.db_network);
        if (cap_max_ex) {
          b.field("cap_max", static_cast<double>(cap_max / ctx.cap_scale));
        } else {
          b.nullField("cap_max");
        }
        if (cap_min_ex) {
          b.field("cap_min", static_cast<double>(cap_min / ctx.cap_scale));
        } else {
          b.nullField("cap_min");
        }
        b.endObject();
      }
    }
  }
  b.endArray();
}

WebSocketResponse SdcHandler::handleSdcLimits(const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return emptyLimits(req.id);
  }

  JsonBuilder b;
  b.beginObject();
  b.field("time_unit", ctx->time_suffix);
  b.field("cap_unit", ctx->cap_suffix);

  emitClockLatencies(b, *ctx);
  emitClockInsertions(b, *ctx);
  emitClockUncertainties(b, *ctx);
  emitPortLoads(b, *ctx);
  emitDisabledTiming(b, *ctx);
  emitCellLimits(b, *ctx);
  emitClockSlewLimits(b, *ctx);
  emitPinClockUncertainties(b, *ctx);
  emitPinCapLimits(b, *ctx);

  b.endObject();
  return jsonResponse(req.id, b.str());
}

WebSocketResponse SdcHandler::handleSdcClockGroups(const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return jsonResponse(req.id, R"({"groups":[]})");
  }
  sta::Sdc* sdc = ctx->sdc;

  JsonBuilder b;
  b.beginObject();
  b.beginArray("groups");

  for (const auto& [name, cg] : sdc->clockGroupsNameMap()) {
    if (!cg) {
      continue;
    }
    b.beginObject();
    b.field("name", name);

    const char* type_str = cg->asynchronous()          ? "asynchronous"
                           : cg->logicallyExclusive()  ? "logically_exclusive"
                           : cg->physicallyExclusive() ? "physically_exclusive"
                                                       : "unknown";
    b.field("type", type_str);
    b.field("allow_paths", cg->allowPaths());
    // SDC -comment carried on the set_clock_groups command.
    b.field("comment", cg->comment());

    // Each entry in groups() is a ClockGroup (= ClockSet*) — a set of clocks
    // that are synchronous to each other.  Clocks in different ClockGroups
    // entries are exclusive/asynchronous with each other.
    b.beginArray("clk_sets");
    for (const sta::ClockGroup* clk_set : *cg->groups()) {
      if (!clk_set) {
        continue;
      }
      b.beginArray();
      for (const sta::Clock* clk : *clk_set) {
        if (clk) {
          b.value(std::string(clk->name()));
        }
      }
      b.endArray();
    }
    b.endArray();

    b.endObject();
  }

  b.endArray();
  b.endObject();
  return jsonResponse(req.id, b.str());
}

WebSocketResponse SdcHandler::handleSdcEndpoint(const WebSocketRequest& req)
{
  static constexpr const char* kEmptyEndpoint
      = R"({"found":false,"time_unit":"ns","multi":false,)"
        R"("total":0,"offset":0,"pins":[]})";
  auto ctx = makeSdcContext(gen_);
  if (!ctx || !ctx->network) {
    return jsonResponse(req.id, kEmptyEndpoint);
  }
  sta::dbSta* sta = ctx->sta;
  sta::Sdc* sdc = ctx->sdc;
  sta::Network* network = ctx->network;
  sta::dbNetwork* db_network = ctx->db_network;
  sta::Scene* scene = ctx->scene;
  sta::Mode* mode = sta->cmdMode();
  const float time_scale = ctx->time_scale;
  const std::string& time_suffix = ctx->time_suffix;

  // Pre-build a pin → exceptions reverse index. Without this each requested
  // pin would walk every exception (O(N×M)); building the map once is O(M).
  // Path-group/filter/loop exceptions are excluded to match the original
  // per-pin filter logic.
  std::map<const sta::Pin*, std::vector<sta::ExceptionPath*>> pin_exceptions;
  for (sta::ExceptionPath* exc : sdc->exceptions()) {
    if (!exc || exc->isFilter() || exc->isLoop() || exc->isGroupPath()) {
      continue;
    }
    auto addPins = [&](const sta::PinSet* ps) {
      if (!ps) {
        return;
      }
      for (const sta::Pin* p : *ps) {
        if (p) {
          pin_exceptions[p].push_back(exc);
        }
      }
    };
    if (exc->from()) {
      addPins(exc->from()->pins());
    }
    if (exc->to()) {
      addPins(exc->to()->pins());
    }
    if (exc->thrus()) {
      for (sta::ExceptionThru* thru : *exc->thrus()) {
        if (thru) {
          addPins(thru->pins());
        }
      }
    }
  }

  // Emit all SDC constraint data for one resolved pin into the open JSON
  // object.
  auto emitPinData = [&](JsonBuilder& b, const sta::Pin* pin) {
    // ── Direction + clock-pin flag ───────────────────────────────────────────
    // The Endpoints tab now renders every pin on the instance (not just the
    // captured endpoint), so the frontend needs to pick a lane shape per pin
    // based on its direction and whether it's a CK input.
    sta::LibertyPort* lp = network->libertyPort(pin);
    sta::LibertyCell* lc = lp ? lp->libertyCell() : nullptr;
    {
      sta::PortDirection* dir = network->direction(pin);
      const char* dir_str = "unknown";
      if (dir == sta::PortDirection::input()) {
        dir_str = "input";
      } else if (dir == sta::PortDirection::output()) {
        dir_str = "output";
      } else if (dir == sta::PortDirection::tristate()) {
        dir_str = "tristate";
      } else if (dir == sta::PortDirection::bidirect()) {
        dir_str = "bidirect";
      } else if (dir == sta::PortDirection::internal()) {
        dir_str = "internal";
      } else if (dir == sta::PortDirection::power()) {
        dir_str = "power";
      } else if (dir == sta::PortDirection::ground()) {
        dir_str = "ground";
      } else if (dir == sta::PortDirection::well()) {
        dir_str = "well";
      }
      b.field("direction", std::string(dir_str));
    }
    // Clock pin = liberty port that drives a sequential's clock (CK / CLK).
    b.field("is_clock_pin", lp ? lp->isRegClk() : false);

    // ── Library setup/hold from the flip-flop's liberty timing arcs ──────────
    // Only meaningful for internal flip-flop pins, not top-level I/O ports.
    // The arc's `fromEdge()` tells us which clock transition closes the
    // capture window — rise for positive flops & active-low latches,
    // fall for active-high latches. We propagate it as `capture_edge`
    // so the frontend can anchor latch diagrams on the correct edge
    // without having to guess the latch polarity.
    float lib_setup = 0.0f, lib_hold = 0.0f;
    bool has_lib_setup = false, has_lib_hold = false;
    const sta::Transition* capture_edge = nullptr;
    sta::LibertyPort* enable_port = nullptr;
    if (lc && scene && !network->isTopLevelPort(pin)) {
      for (sta::TimingArcSet* tas : lc->timingArcSetsTo(lp)) {
        const sta::TimingRole* role = tas->role();
        const bool is_setup = (role == sta::TimingRole::setup());
        const bool is_hold = (role == sta::TimingRole::hold());
        if (!is_setup && !is_hold) {
          continue;
        }
        const sta::MinMax* mm
            = is_setup ? sta::MinMax::max() : sta::MinMax::min();
        for (sta::TimingArc* arc : tas->arcs()) {
          sta::CheckTimingModel* chk = arc->checkModel(scene, mm);
          if (!chk) {
            continue;
          }
          float val = static_cast<float>(chk->checkDelay(
              nullptr, 0.0f, 0.0f, 0.0f, mm, sta::PocvMode::scalar));
          if (is_setup) {
            if (!has_lib_setup || val > lib_setup) {
              lib_setup = val;
              has_lib_setup = true;
            }
            // Capture edge is the clock-pin transition that triggers
            // the setup check — pulled from the first setup arc's
            // fromEdge. Hold arcs reference the same edge in OpenSTA.
            if (!capture_edge && arc->fromEdge()) {
              capture_edge = arc->fromEdge();
            }
            // Enable port = the clock/enable input that the setup arc
            // is checked against. Used below to resolve per-pin latch
            // borrow limits via Sdc::latchBorrowLimit.
            if (!enable_port && arc->from()) {
              enable_port = arc->from();
            }
          } else {
            if (!has_lib_hold || val > lib_hold) {
              lib_hold = val;
              has_lib_hold = true;
            }
          }
        }
      }
    }
    if (capture_edge == sta::Transition::rise()) {
      b.field("capture_edge", std::string("rise"));
    } else if (capture_edge == sta::Transition::fall()) {
      b.field("capture_edge", std::string("fall"));
    } else {
      b.nullField("capture_edge");
    }

    // ── Library clk-to-Q for output pins of sequential cells ────────────────
    // Pulled from the liberty regClkToQ timing arcs whose 'to' port is this
    // pin. We track rise/fall × max/min separately so the frontend can draw
    // the clk-to-Q band with both early/late edges.
    double clkq[2][2] = {{0, 0}, {0, 0}};  // [rise=0/fall=1][max=0/min=1]
    bool has_clkq[2][2] = {{false, false}, {false, false}};
    if (lc && lp && scene && !network->isTopLevelPort(pin)) {
      for (sta::TimingArcSet* tas : lc->timingArcSetsTo(lp)) {
        if (tas->role() != sta::TimingRole::regClkToQ()) {
          continue;
        }
        for (sta::TimingArc* arc : tas->arcs()) {
          if (!arc->toEdge()) {
            continue;
          }
          const sta::RiseFall* to_rf = arc->toEdge()->asRiseFall();
          if (!to_rf) {
            continue;
          }
          const int rfi = (to_rf == sta::RiseFall::rise()) ? 0 : 1;
          for (int mmi = 0; mmi < 2; ++mmi) {
            const sta::MinMax* mm
                = (mmi == 0) ? sta::MinMax::max() : sta::MinMax::min();
            sta::GateTimingModel* gtm = arc->gateModel(scene, mm);
            if (!gtm) {
              continue;
            }
            float gd = 0.0f, ds = 0.0f;
            gtm->gateDelay(nullptr, 0.0f, 0.0f, gd, ds);
            const double v = scaleTime(gd, time_scale);
            if (!has_clkq[rfi][mmi]
                || (mmi == 0 ? v > clkq[rfi][mmi] : v < clkq[rfi][mmi])) {
              clkq[rfi][mmi] = v;
              has_clkq[rfi][mmi] = true;
            }
          }
        }
      }
    }
    if (has_clkq[0][0] || has_clkq[0][1] || has_clkq[1][0] || has_clkq[1][1]) {
      b.beginObject("clk_to_q");
      auto emitClkq = [&](const char* key, int rfi, int mmi) {
        if (has_clkq[rfi][mmi]) {
          b.field(key, clkq[rfi][mmi]);
        } else {
          b.nullField(key);
        }
      };
      emitClkq("rise_max", 0, 0);
      emitClkq("rise_min", 0, 1);
      emitClkq("fall_max", 1, 0);
      emitClkq("fall_min", 1, 1);
      b.endObject();
    } else {
      b.nullField("clk_to_q");
    }

    // ── Clock domains: name, period, waveform, uncertainty, library times ──
    // Resolve the enable pin once (shared across clock domains) so we
    // can query Sdc::latchBorrowLimit per (data_pin, enable_pin, clk)
    // and surface the explicit set_max_time_borrow value to the
    // frontend's latch lane renderer.
    const bool is_latch_data = lp && lp->isLatchData();
    const sta::Pin* enable_pin_resolved = nullptr;
    if (is_latch_data && enable_port) {
      sta::Instance* inst = network->instance(pin);
      if (inst) {
        enable_pin_resolved = network->findPin(inst, enable_port);
      }
    }
    b.beginArray("clocks");
    if (mode) {
      sta::ClockSet domains = sta->clockDomains(pin, mode);
      for (const sta::Clock* clk : domains) {
        if (!clk) {
          continue;
        }
        b.beginObject();
        b.field("name", std::string(clk->name()));
        b.field("period", scaleTime(clk->period(), time_scale));
        const sta::FloatSeq& wf = clk->waveform();
        b.beginArray("waveform");
        if (!wf.empty()) {
          for (float t : wf) {
            b.value(scaleTime(t, time_scale));
          }
        } else {
          b.value(0.0);
          b.value(scaleTime(clk->period(), time_scale) / 2.0);
        }
        b.endArray();
        emitUncertainty(b,
                        "uncertainty_setup",
                        "uncertainty_hold",
                        extractUncertainty(&clk->uncertainties()),
                        time_scale);
        if (has_lib_setup) {
          b.field("library_setup", scaleTime(lib_setup, time_scale));
        } else {
          b.nullField("library_setup");
        }
        if (has_lib_hold) {
          b.field("library_hold", scaleTime(lib_hold, time_scale));
        } else {
          b.nullField("library_hold");
        }
        // Explicit set_max_time_borrow (per-pin/per-instance/per-clock)
        // for latch data inputs. Latches without an explicit limit fall
        // back to the implicit transparent-period borrow on the frontend
        // — we leave the field null in that case so the renderer can
        // distinguish "user-specified limit" from "library default".
        if (is_latch_data) {
          float limit = 0.0f;
          bool has_limit = false;
          sdc->latchBorrowLimit(pin,
                                enable_pin_resolved,
                                const_cast<sta::Clock*>(clk),
                                limit,
                                has_limit);
          if (has_limit) {
            b.field("time_borrow_limit", scaleTime(limit, time_scale));
          } else {
            b.nullField("time_borrow_limit");
          }
        } else {
          b.nullField("time_borrow_limit");
        }
        b.endObject();
      }
    }
    b.endArray();

    // ── Exceptions that reference this pin ─────────────────────────────────
    // Lookup is O(1) thanks to the pre-built reverse index.
    b.beginArray("exceptions");
    auto eit = pin_exceptions.find(pin);
    if (eit != pin_exceptions.end()) {
      for (sta::ExceptionPath* exc : eit->second) {
        b.beginObject();
        b.field("type", exceptionTypeStr(exc));
        const sta::MinMaxAll* mm = exc->minMax();
        b.field("min_max", mm ? mm->to_string() : std::string("max"));
        b.field("multiplier", exc->pathMultiplier());
        if (exc->isPathDelay()) {
          b.field("delay", scaleTime(exc->delay(), time_scale));
        } else {
          b.nullField("delay");
        }
        b.endObject();
      }
    }
    b.endArray();

    // ── Port delays if this is a top-level port pin ─────────────────────────
    b.beginArray("port_delays");
    if (network->isTopLevelPort(pin)) {
      auto emitPortDelay = [&](sta::PortDelay* pd, bool is_input) {
        if (!pd) {
          return;
        }
        b.beginObject();
        b.field("is_input", is_input);
        sta::Clock* clk = pd->clock();
        if (clk) {
          b.field("clock", std::string(clk->name()));
          b.field("clk_period", scaleTime(clk->period(), time_scale));
          emitUncertainty(b,
                          "uncertainty_setup",
                          "uncertainty_hold",
                          extractUncertainty(&clk->uncertainties()),
                          time_scale);
        } else {
          b.nullField("clock");
          b.nullField("clk_period");
          b.nullField("uncertainty_setup");
          b.nullField("uncertainty_hold");
        }
        const sta::ClockEdge* clk_edge = pd->clkEdge();
        if (clk_edge) {
          b.field("clk_edge", std::string(clk_edge->transition()->name()));
        } else {
          b.nullField("clk_edge");
        }
        emitRiseFallMinMax(b, pd->delays(), time_scale);
        b.endObject();
      };
      for (sta::InputDelay* id : sdc->inputDelays()) {
        if (id && id->pin() == pin) {
          emitPortDelay(id, true);
        }
      }
      for (sta::OutputDelay* od : sdc->outputDelays()) {
        if (od && od->pin() == pin) {
          emitPortDelay(od, false);
        }
      }
    }
    b.endArray();
  };

  // ── Resolve pin(s): exact match or glob via SdcNetwork::findPinsMatching
  // (hierarchical, prefix-pruned) plus a separate port search since
  // SdcNetwork's pin search excludes top-level ports (mirrors `get_pins`
  // vs `get_ports` SDC semantics).
  const std::string pat = extract_string(req.raw_json, "pin");
  std::vector<const sta::Pin*> matched;
  if (sta::patternWildcards(pat)) {
    sta::PatternMatch pattern(pat);
    // Hierarchical pin search (get_pins).
    sta::Network* sdc_net = sta->sdcNetwork();
    if (sdc_net) {
      sta::PinSeq pins
          = sdc_net->findPinsMatching(sdc_net->topInstance(), &pattern);
      matched.assign(pins.begin(), pins.end());
    }
    // Top-level port search (get_ports) → resolve each port to its pin on
    // the top instance.  Avoid double-adding anything by keeping a tiny
    // de-dup set keyed on pin pointer identity.
    if (network) {
      const sta::Instance* top = network->topInstance();
      const sta::Cell* top_cell = top ? network->cell(top) : nullptr;
      if (top_cell) {
        std::set<const sta::Pin*> seen(matched.begin(), matched.end());
        sta::PortSeq ports = network->findPortsMatching(top_cell, &pattern);
        for (const sta::Port* port : ports) {
          if (!port) {
            continue;
          }
          const sta::Pin* pin = network->findPin(top, port);
          if (pin && seen.insert(pin).second) {
            matched.push_back(pin);
          }
        }
      }
    }
  } else {
    sta::Pin* pin = network->findPin(pat);
    if (pin) {
      matched.push_back(pin);
    }
  }

  // Pagination: glob resolution above is one-shot, but the per-pin
  // emission below (CheckTimingModel queries, exception walks, port-delay
  // walks) is the expensive part on globs that match thousands of pins.
  // Slice the matched list so the frontend can stream results in batches.
  const int total = static_cast<int>(matched.size());
  int offset = extract_int_or(req.raw_json, "offset", 0);
  if (offset < 0) {
    offset = 0;
  }
  if (offset > total) {
    offset = total;
  }
  int limit = extract_int_or(req.raw_json, "limit", -1);
  if (limit < 0) {
    limit = total - offset;
  }
  int end = offset + limit;
  if (end > total) {
    end = total;
  }

  JsonBuilder b;
  b.beginObject();
  b.field("time_unit", time_suffix);
  b.field("found", !matched.empty());
  b.field("multi", total > 1);
  b.field("total", total);
  b.field("offset", offset);
  b.field("truncated", false);
  b.beginArray("pins");
  for (int i = offset; i < end; ++i) {
    const sta::Pin* p = matched[i];
    b.beginObject();
    b.field("name", std::string(network->pathName(p)));
    emitPinOdbRef(b, "name", p, db_network);
    b.field("is_port", network->isTopLevelPort(p));
    emitPinData(b, p);
    b.endObject();
  }
  b.endArray();
  b.endObject();
  return jsonResponse(req.id, b.str());
}

// Paginated summary of timing endpoints (Search::endpoints()) with optional
// glob pattern + kind filter ("flipflop"|"latch"|"macro"|"stdcell"|"all").
// The first call is O(V) (graph build) — gated behind the explicit
// "List endpoints" button on the frontend.
//
// Top-level ports are deliberately excluded: they're real endpoints in
// STA's model, but the Port Delays tab already renders them with full
// timing diagrams + driving cell + load info, and showing them again
// here produced visually-identical duplicate cards.
WebSocketResponse SdcHandler::handleSdcEndpointList(const WebSocketRequest& req)
{
  static constexpr const char* kEmptyList
      = R"({"time_unit":"ns","total":0,"offset":0,)"
        R"("kinds_total":{"flipflop":0,"latch":0,"macro":0,"stdcell":0,)"
        R"("clock_gate":0},)"
        R"("clocks_total":{},)"
        R"("endpoints":[]})";
  auto ctx = makeSdcContext(gen_);
  if (!ctx || !ctx->network || !ctx->network->isLinked()) {
    return jsonResponse(req.id, kEmptyList);
  }
  sta::dbSta* sta = ctx->sta;
  sta::Network* network = ctx->network;
  sta::dbNetwork* db_network = ctx->db_network;
  sta::Mode* mode = sta->cmdMode();
  // Building the timing graph is the heavy step on first call; it's gated
  // behind the "List endpoints" button on the frontend so the cost is
  // user-initiated.
  sta->ensureGraph();
  sta::Search* search = sta->search();
  if (!search) {
    return jsonResponse(req.id, kEmptyList);
  }
  const std::string& time_suffix = ctx->time_suffix;
  const float time_scale = ctx->time_scale;

  // Classify a vertex's pin into one of:
  //   "flipflop" — sequential cell whose sequentials are *all* registers
  //                (Sequential::isRegister() — STA's own classification)
  //   "latch"    — sequential cell whose sequentials are *all* latches
  //                (Sequential::isLatch() — also STA's own classification)
  //   "macro"    — explicit macro / memory (LibertyCell::isMacro / isMemory)
  //   "stdcell"  — anything else, including:
  //                  - Liberty `statetable`-bodied cells (sync flops, async
  //                    FIFOs, anything where the standard ff/latch shapes
  //                    don't capture the behaviour) — STA doesn't classify
  //                    these as either flop or latch, so we don't either
  //                  - mixed cells with both register and latch sequentials
  //                  - combinational endpoints reached by set_max_delay -to
  //                    or set_output_delay through a buffer chain.
  // Top-level ports (network->isTopLevelPort) are filtered out before this
  // function runs — see the endpoint-walk loop below — because they live
  // on the Port Delays tab.
  auto classify = [&](const sta::Pin* pin) -> const char* {
    sta::Instance* inst = network->instance(pin);
    if (!inst) {
      return "stdcell";
    }
    sta::LibertyCell* lc = network->libertyCell(inst);
    if (!lc) {
      return "stdcell";
    }
    // Integrated clock-gating cells — checked BEFORE hasSequentials()
    // because Nangate45's CLKGATE_X* cells are statetable-bodied and
    // would otherwise fall through to stdcell. `isClockGate()` is the
    // public OpenSTA flag set when Liberty declared
    // `clock_gating_integrated_cell : <flavor>`.
    if (lc->isClockGate()) {
      return "clock_gate";
    }
    if (lc->hasSequentials()) {
      // Conservative: only mark as flipflop / latch when EVERY sequential
      // entry agrees on the kind. A mixed cell (register + latch in the
      // same liberty body) is a vendor-specific construct we don't want
      // to misrepresent — bucket it as stdcell along with statetable
      // bodies.
      bool any_register = false;
      bool any_latch = false;
      for (const sta::Sequential& seq : lc->sequentials()) {
        if (seq.isRegister()) {
          any_register = true;
        } else {
          any_latch = true;
        }
      }
      if (any_register && !any_latch) {
        return "flipflop";
      }
      if (any_latch && !any_register) {
        return "latch";
      }
      // Mixed or empty — fall through to the stdcell bucket below.
    }
    if (lc->isMacro() || lc->isMemory()) {
      return "macro";
    }
    return "stdcell";
  };

  // Single pass: walk every endpoint, classify, then keep only those
  // matching the kind + glob filters. We tally per-kind totals before
  // applying the kind filter so the frontend toolbar can show the full
  // picture even when narrowed.
  const std::string pattern = extract_string(req.raw_json, "pattern");
  std::unique_ptr<sta::PatternMatch> pat;
  if (!pattern.empty()) {
    // Glob-style match (default ctor uses unix-glob semantics, which is
    // what SDC-style get_pins / get_ports expect).
    pat = std::make_unique<sta::PatternMatch>(pattern);
  }
  const std::string kind_filter = extract_string(req.raw_json, "kind");
  auto kindMatches = [&](const char* k) {
    if (kind_filter.empty() || kind_filter == "all") {
      return true;
    }
    return kind_filter == k;
  };
  // Accept either "all"/empty (no filter), a single clock name, or a
  // comma-separated whitelist (the multi-select dropdown sends this
  // form when a subset of clocks is selected). Whitespace around each
  // entry is tolerated so the JS side can format the list however it
  // likes.
  std::set<std::string> clock_whitelist;
  {
    const std::string cf = extract_string(req.raw_json, "clock");
    if (!cf.empty() && cf != "all") {
      std::string::size_type i = 0;
      while (i <= cf.size()) {
        std::string::size_type j = cf.find(',', i);
        if (j == std::string::npos) {
          j = cf.size();
        }
        std::string tok = cf.substr(i, j - i);
        // trim
        std::string::size_type a = tok.find_first_not_of(" \t");
        std::string::size_type b = tok.find_last_not_of(" \t");
        if (a != std::string::npos && b != std::string::npos) {
          clock_whitelist.insert(tok.substr(a, b - a + 1));
        }
        i = j + 1;
      }
    }
  }
  auto clockMatches = [&](const std::vector<std::string>& clks) {
    if (clock_whitelist.empty()) {
      return true;
    }
    for (const auto& c : clks) {
      if (clock_whitelist.count(c)) {
        return true;
      }
    }
    return false;
  };

  struct Hit
  {
    const sta::Pin* pin;
    const char* kind;
    std::string name;
    std::vector<std::string> clocks;  // clockDomains() result, cached for emit
  };
  std::vector<Hit> hits;
  // The Endpoints tab groups pins by their owning instance (one card
  // per instance, regardless of how many endpoint pins it exposes), so
  // the kind-filter button counts surface *instances*, not pins. A
  // 64-bit-wide RAM with 64 endpoint pins counts as one MACRO. Without
  // this dedup the Macro count was inflated and the user was sent
  // chasing a count that didn't match the cards on screen.
  std::set<const sta::Instance*> seen_flop;
  std::set<const sta::Instance*> seen_latch;
  std::set<const sta::Instance*> seen_macro;
  std::set<const sta::Instance*> seen_stdcell;
  std::set<const sta::Instance*> seen_clock_gate;
  // Clocks tally also dedupes by instance so the same RAM doesn't
  // contribute 64 to its capture clock's count.
  std::map<std::string, std::set<const sta::Instance*>> clocks_total_insts;
  sta::VertexSet& endpoints = search->endpoints();
  hits.reserve(endpoints.size() / 4);  // rough hint
  for (sta::Vertex* v : endpoints) {
    if (!v) {
      continue;
    }
    const sta::Pin* pin = v->pin();
    if (!pin) {
      continue;
    }
    // Skip top-level ports — they live on the Port Delays tab.
    if (network->isTopLevelPort(pin)) {
      continue;
    }
    const char* kind = classify(pin);
    sta::Instance* inst = network->instance(pin);
    // Pre-filter instance-deduped tally.
    if (std::string_view(kind) == "flipflop") {
      seen_flop.insert(inst);
    } else if (std::string_view(kind) == "latch") {
      seen_latch.insert(inst);
    } else if (std::string_view(kind) == "macro") {
      seen_macro.insert(inst);
    } else if (std::string_view(kind) == "clock_gate") {
      seen_clock_gate.insert(inst);
    } else {
      seen_stdcell.insert(inst);
    }
    // Cache the clockDomains() result so we don't recompute it per
    // emit. Done here (not later) because the clock filter and the
    // clocks_total tally both need it before we slice the page.
    std::vector<std::string> clk_names;
    if (mode) {
      sta::ClockSet domains = sta->clockDomains(pin, mode);
      for (const sta::Clock* c : domains) {
        if (c) {
          clk_names.emplace_back(c->name());
        }
      }
    }
    for (const auto& cn : clk_names) {
      clocks_total_insts[cn].insert(inst);
    }
    if (!kindMatches(kind)) {
      continue;
    }
    std::string name = network->pathName(pin);
    if (pat && !pat->match(name.c_str())) {
      continue;
    }
    if (!clockMatches(clk_names)) {
      continue;
    }
    hits.push_back({pin, kind, std::move(name), std::move(clk_names)});
  }

  // Stable order = path-name lexicographic. Cheap relative to the walk
  // above and gives the UI a deterministic page layout.
  std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) {
    return a.name < b.name;
  });

  const int total = static_cast<int>(hits.size());
  // Instance-deduped count of the (filtered) hits. The frontend
  // renders one card per instance, so it uses this value (not `total`)
  // for the "Loaded X of Y" footer — without it, a macro-filter that
  // matches 76 instances but 6700 endpoint pins reads as
  // "Loaded N of 6700" which the user has no way to reconcile with
  // the 76 cards on screen.
  std::set<sta::Instance*> hit_insts_unique;
  for (const Hit& h : hits) {
    sta::Instance* inst = network->instance(h.pin);
    if (inst) {
      hit_insts_unique.insert(inst);
    }
  }
  const int total_instances = static_cast<int>(hit_insts_unique.size());

  int offset = extract_int_or(req.raw_json, "offset", 0);
  if (offset < 0) {
    offset = 0;
  }
  if (offset > total) {
    offset = total;
  }
  int limit = extract_int_or(req.raw_json, "limit", -1);
  if (limit < 0) {
    limit = total - offset;
  }
  int end = offset + limit;
  if (end > total) {
    end = total;
  }

  JsonBuilder b;
  b.beginObject();
  b.field("time_unit", time_suffix);
  b.field("total", total);
  b.field("total_instances", total_instances);
  b.field("offset", offset);
  // Counts are unique-instance counts (matches the one-card-per-
  // instance grouping the frontend renders). See the seen_* sets
  // above for the dedup pass.
  b.beginObject("kinds_total");
  b.field("flipflop", static_cast<int>(seen_flop.size()));
  b.field("latch", static_cast<int>(seen_latch.size()));
  b.field("macro", static_cast<int>(seen_macro.size()));
  b.field("stdcell", static_cast<int>(seen_stdcell.size()));
  b.field("clock_gate", static_cast<int>(seen_clock_gate.size()));
  b.endObject();
  // Per-clock totals — keyed by clock name, sorted alphabetically so
  // the frontend dropdown lists clocks in a stable order across calls.
  // Like kinds_total, the count is unique-instance, not unique-pin.
  b.beginObject("clocks_total");
  for (const auto& [name, insts] : clocks_total_insts) {
    b.field(name, static_cast<int>(insts.size()));
  }
  b.endObject();
  b.beginArray("endpoints");
  for (int i = offset; i < end; ++i) {
    const Hit& h = hits[i];
    b.beginObject();
    b.field("name", h.name);
    b.field("kind", std::string(h.kind));
    // ODB ref so the row can dispatch to the inspector via the existing
    // inspect(odb_type, odb_id) shortcut.
    emitPinOdbBare(b, h.pin, db_network);
    // Enclosing instance + liberty cell name (omitted for top ports).
    sta::Instance* inst = network->instance(h.pin);
    sta::LibertyCell* lc_for_emit = nullptr;
    if (inst && !network->isTopLevelPort(h.pin)) {
      b.field("instance", std::string(network->pathName(inst)));
      lc_for_emit = network->libertyCell(inst);
      if (lc_for_emit) {
        b.field("cell", std::string(lc_for_emit->name()));
      } else {
        b.nullField("cell");
      }
    } else {
      b.nullField("instance");
      b.nullField("cell");
    }
    // Clock-gate-specific fields (only emitted on kind=clock_gate). The
    // frontend uses these to render the ICG-flavoured waveform diagram
    // and to label which pin is CK / EN / scan-EN / GCK.
    if (lc_for_emit && lc_for_emit->isClockGate()) {
      const char* flavor = "other";
      if (lc_for_emit->isClockGateLatchPosedge()) {
        flavor = "latch_posedge";
      } else if (lc_for_emit->isClockGateLatchNegedge()) {
        flavor = "latch_negedge";
      }
      b.field("clock_gate_flavor", std::string(flavor));
      // Walk the cell's liberty ports tagging each by role. CLKGATE
      // cells in real-world libraries label the clock pin with
      // `clock_gate_clock_pin : true;`, the enable with
      // `clock_gate_enable_pin : true;`, scan-enable optional, output
      // with `clock_gate_out_pin : true;`. We expose pin paths so the
      // frontend can wire the waveform rows to the correct pins.
      auto namedPin = [&](const char* role,
                          bool (sta::LibertyPort::*pred)() const) {
        for (sta::LibertyCellPortIterator it(lc_for_emit); it.hasNext();) {
          sta::LibertyPort* lp = it.next();
          if (lp && (lp->*pred)()) {
            // Look up the corresponding instance pin.
            const sta::Pin* p = network->findPin(inst, lp->name());
            if (p) {
              b.field(std::string(role) + "_pin",
                      std::string(network->pathName(p)));
              const char* tp = nullptr;
              int id = 0;
              if (resolvePinOdb(p, db_network, tp, id)) {
                b.field(std::string(role) + "_pin_odb_type", std::string(tp));
                b.field(std::string(role) + "_pin_odb_id", id);
              }
              return;
            }
          }
        }
        b.nullField(std::string(role) + "_pin");
      };
      namedPin("clock_gate_ck", &sta::LibertyPort::isClockGateClock);
      namedPin("clock_gate_en", &sta::LibertyPort::isClockGateEnable);
      namedPin("clock_gate_out", &sta::LibertyPort::isClockGateOut);
      // Cell-level setup/hold values from the Liberty timing arcs.
      // Liberty stores a clock-gate's setup/hold as ordinary
      // `setup_*` / `hold_*` arcs targeting the enable pin (with the
      // cell tagged via `clock_gating_integrated_cell`); OpenSTA's
      // `gatedClockSetup` / `gatedClockHold` are PATH-END roles
      // synthesized during search and never appear on a cell's
      // `TimingArcSet`. So we look up the enable port and walk the
      // arcs ending there, filtering by the standard setup/hold
      // generic role, and evaluate the CheckTimingModel at the
      // library's default operating conditions with zero slew /
      // zero load — for scalar Liberty constraints (the common
      // shape on ICG cells) this returns the constant directly. Per-
      // pin SDC overrides via `set_clock_gating_check` are NOT
      // enumerable today (`Sdc::pin_clk_gating_check_map_` is private
      // and there's no public getter), so the value reported here is
      // the cell's library default — the SDC override layer can
      // replace it once the upstream getter lands.
      sta::LibertyPort* enable_port = nullptr;
      for (sta::LibertyCellPortIterator it(lc_for_emit); it.hasNext();) {
        sta::LibertyPort* lp_iter = it.next();
        if (lp_iter && lp_iter->isClockGateEnable()) {
          enable_port = lp_iter;
          break;
        }
      }
      const sta::Pvt* pvt
          = lc_for_emit->libertyLibrary()
                ? lc_for_emit->libertyLibrary()->defaultOperatingConditions()
                : nullptr;
      auto extractCheck = [&](const sta::TimingRole* role) -> double {
        if (!pvt || !enable_port) {
          return std::numeric_limits<double>::quiet_NaN();
        }
        for (sta::TimingArcSet* set :
             lc_for_emit->timingArcSetsTo(enable_port)) {
          if (!set || set->role() != role) {
            continue;
          }
          for (sta::TimingArc* arc : set->arcs()) {
            if (!arc) {
              continue;
            }
            sta::CheckTimingModel* model
                = dynamic_cast<sta::CheckTimingModel*>(arc->model());
            if (!model) {
              continue;
            }
            // Zero slews / zero load at the library's typical PVT —
            // for scalar tables the constraint is constant; for
            // table-lookup it lands at the bottom-left grid corner
            // which is a reasonable representative value.
            const float v
                = static_cast<float>(model->checkDelay(pvt,
                                                       0.0f,
                                                       0.0f,
                                                       0.0f,
                                                       sta::MinMax::max(),
                                                       sta::PocvMode::scalar));
            return v / time_scale;
          }
        }
        return std::numeric_limits<double>::quiet_NaN();
      };
      const double setup_v = extractCheck(sta::TimingRole::setup());
      const double hold_v = extractCheck(sta::TimingRole::hold());
      if (std::isnan(setup_v)) {
        b.nullField("clock_gate_setup");
      } else {
        b.field("clock_gate_setup", setup_v);
      }
      if (std::isnan(hold_v)) {
        b.nullField("clock_gate_hold");
      } else {
        b.field("clock_gate_hold", hold_v);
      }
    }
    // Clock domain names (cached during the walk above).
    b.beginArray("clocks");
    for (const auto& cn : h.clocks) {
      b.value(cn);
    }
    b.endArray();
    b.endObject();
  }
  b.endArray();
  b.endObject();
  return jsonResponse(req.id, b.str());
}

WebSocketResponse SdcHandler::handleSdcListModes(const WebSocketRequest& req)
{
  sta::dbSta* sta = gen_ ? gen_->getSta() : nullptr;
  if (!sta) {
    return jsonResponse(req.id, R"({"modes":[],"current":""})");
  }

  std::string current;
  if (sta::Mode* m = sta->cmdMode()) {
    current = m->name();
  }

  JsonBuilder b;
  b.beginObject();
  b.field("current", current);
  b.beginArray("modes");
  for (sta::Mode* m : sta->modes()) {
    if (!m) {
      continue;
    }
    b.value(std::string(m->name()));
  }
  b.endArray();
  b.endObject();
  return jsonResponse(req.id, b.str());
}

WebSocketResponse SdcHandler::handleSdcSetMode(const WebSocketRequest& req)
{
  sta::dbSta* sta = gen_ ? gen_->getSta() : nullptr;
  if (!sta) {
    return jsonResponse(req.id,
                        R"({"ok":false,"error":"no sta","current":""})");
  }

  const std::string target = extract_string(req.raw_json, "mode");
  if (target.empty()) {
    return jsonResponse(
        req.id, R"({"ok":false,"error":"empty mode name","current":""})");
  }

  sta::Mode* mode = sta->findMode(target);
  if (!mode) {
    JsonBuilder b;
    b.beginObject();
    b.field("ok", false);
    b.field("error", std::string("mode not found: ") + target);
    std::string cur;
    if (sta::Mode* cm = sta->cmdMode()) {
      cur = cm->name();
    }
    b.field("current", cur);
    b.endObject();
    return jsonResponse(req.id, b.str());
  }

  // Already active — no-op success.
  if (sta::Mode* cur = sta->cmdMode(); cur == mode) {
    JsonBuilder b;
    b.beginObject();
    b.field("ok", true);
    b.field("current", std::string(mode->name()));
    b.endObject();
    return jsonResponse(req.id, b.str());
  }

  // Switch by picking any scene registered to the target mode and making it
  // the command scene. Each mode owns its scenes; the first one is fine.
  const sta::SceneSeq& scenes = mode->scenes();
  if (scenes.empty()) {
    JsonBuilder b;
    b.beginObject();
    b.field("ok", false);
    b.field("error", std::string("no scene for mode: ") + target);
    std::string cur;
    if (sta::Mode* cm = sta->cmdMode()) {
      cur = cm->name();
    }
    b.field("current", cur);
    b.endObject();
    return jsonResponse(req.id, b.str());
  }

  sta->setCmdScene(scenes[0]);

  JsonBuilder b;
  b.beginObject();
  b.field("ok", true);
  b.field("current", std::string(mode->name()));
  b.endObject();
  return jsonResponse(req.id, b.str());
}

// Equivalent to Tcl `update_generated_clks` — fills period/waveform on
// generated clocks. Side-effect: builds the timing graph (heavier than the
// read-only handlers).
WebSocketResponse SdcHandler::handleSdcResolveGenClocks(
    const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return jsonResponse(req.id,
                        R"({"ok":false,"error":"no sta","resolved":0})");
  }
  sta::dbSta* sta = ctx->sta;
  sta::Sdc* sdc = ctx->sdc;

  // Count how many generated clocks were unresolved BEFORE the call so
  // the response can report progress.
  int before = 0;
  for (sta::Clock* clk : sdc->clocks()) {
    if (clk && clk->isGenerated() && !clk->waveformValid()) {
      ++before;
    }
  }

  std::string error;
  try {
    sta->updateGeneratedClks();
  } catch (const std::exception& e) {
    error = e.what();
  }

  int after = 0;
  for (sta::Clock* clk : sdc->clocks()) {
    if (clk && clk->isGenerated() && !clk->waveformValid()) {
      ++after;
    }
  }

  JsonBuilder b;
  b.beginObject();
  b.field("ok", error.empty());
  if (!error.empty()) {
    b.field("error", error);
  }
  // How many generated clocks now have a resolved waveform.
  b.field("resolved", before - after);
  b.field("remaining", after);
  b.endObject();
  return jsonResponse(req.id, b.str());
}

// Registration entry point — called from web.cpp at session
// construction to wire every kSdc* type into the dispatcher. Each
// handler reads its own fields from req.raw_json.
void SdcHandler::registerRequests(RequestDispatcher& d)
{
  d.add("sdc_clocks",
        WebSocketRequest::kSdcClocks,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcClocks(req);
        });
  d.add("sdc_clock_modes",
        WebSocketRequest::kSdcClockModes,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcClockModes(req);
        });
  d.add("sdc_port_delays",
        WebSocketRequest::kSdcPortDelays,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcPortDelays(req);
        });
  d.add("sdc_exceptions",
        WebSocketRequest::kSdcExceptions,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcExceptions(req);
        });
  d.add("sdc_limits",
        WebSocketRequest::kSdcLimits,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcLimits(req);
        });
  d.add("sdc_clock_groups",
        WebSocketRequest::kSdcClockGroups,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcClockGroups(req);
        });
  d.add("sdc_endpoint",
        WebSocketRequest::kSdcEndpoint,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcEndpoint(req);
        });
  d.add("sdc_endpoint_list",
        WebSocketRequest::kSdcEndpointList,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcEndpointList(req);
        });
  d.add("sdc_list_modes",
        WebSocketRequest::kSdcListModes,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcListModes(req);
        });
  d.add("sdc_set_mode",
        WebSocketRequest::kSdcSetMode,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcSetMode(req);
        });
  d.add("sdc_resolve_gen_clocks",
        WebSocketRequest::kSdcResolveGenClocks,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcResolveGenClocks(req);
        });
}

}  // namespace web
