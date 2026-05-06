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

#include <boost/json.hpp>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
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
static void emitRiseFallMinMax(boost::json::object& obj,
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
      obj[s.key] = static_cast<double>(val / scale);
    } else {
      obj[s.key] = nullptr;
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
static void emitPinOdbRef(boost::json::object& obj,
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

// Bare-field variant of emitPinOdbRef for arrays of {name, odb_type, odb_id}.
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

// Emit bare "odb_type" / "odb_id" for an Instance: dbInst → "inst",
// dbModInst → "modinst".
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
static void emitUncertainty(boost::json::object& obj,
                            const char* setup_key,
                            const char* hold_key,
                            const UncertaintyValues& u,
                            float scale)
{
  if (u.setup_exists) {
    obj[setup_key] = static_cast<double>(u.setup_val / scale);
  } else {
    obj[setup_key] = nullptr;
  }
  if (u.hold_exists) {
    obj[hold_key] = static_cast<double>(u.hold_val / scale);
  } else {
    obj[hold_key] = nullptr;
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

// Recursively build a clock-tree node (name + nested children array).
static boost::json::object buildTreeNode(
    const sta::Clock* clk,
    const std::map<const sta::Clock*, std::vector<sta::Clock*>>& children)
{
  boost::json::object node;
  if (!clk) {
    return node;
  }
  node["name"] = std::string(clk->name());
  boost::json::array kids;
  auto it = children.find(clk);
  if (it != children.end()) {
    for (sta::Clock* child : it->second) {
      if (child) {
        kids.push_back(buildTreeNode(child, children));
      }
    }
  }
  node["children"] = std::move(kids);
  return node;
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

  boost::json::object root;
  root["time_unit"] = time_suffix;

  // ── "clocks" array ──────────────────────────────────────────────────────
  {
    boost::json::array clocks_arr;
    for (sta::Clock* clk : clocks) {
      if (!clk) {
        continue;
      }
      boost::json::object o;
      o["name"] = std::string(clk->name());

      // period() is always a float scalar — safe to read directly.
      o["period"] = scaleTime(clk->period(), time_scale);

      // Waveform: alternating rise/fall edge times. May be empty.
      const sta::FloatSeq& waveform = clk->waveform();
      {
        boost::json::array wf;
        if (!waveform.empty()) {
          for (float t : waveform) {
            wf.push_back(scaleTime(t, time_scale));
          }
        } else {
          // Synthesize a default 50% duty-cycle waveform from the period.
          wf.push_back(0.0);
          wf.push_back(scaleTime(clk->period(), time_scale) / 2.0);
        }
        o["waveform"] = std::move(wf);
      }

      o["is_generated"] = clk->isGenerated();
      o["is_virtual"] = clk->isVirtual();
      o["is_propagated"] = clk->isPropagated();
      // create_clock -add: when true, this clock was added to existing pins
      // rather than replacing them. Surfaced so users can spot multi-clock
      // pins.
      o["add_to_pins"] = clk->addToPins();
      // Generated-clock diagnostics (omitted/null on primary clocks).
      if (clk->isGenerated()) {
        o["combinational"] = clk->combinational();
        o["master_inferred"] = clk->masterClkInfered();
        o["generated_up_to_date"] = clk->waveformValid();
      } else {
        o["combinational"] = nullptr;
        o["master_inferred"] = nullptr;
        o["generated_up_to_date"] = nullptr;
      }
      o["comment"] = std::string(clk->comment());

      sta::Clock* master = clk->masterClk();
      if (master) {
        o["master_clock"] = std::string(master->name());
      } else {
        o["master_clock"] = nullptr;
      }

      if (clk->isGenerated()) {
        int div = clk->divideBy();
        int mul = clk->multiplyBy();
        if (div > 0) {
          o["divide_by"] = div;
        } else {
          o["divide_by"] = nullptr;
        }
        if (mul > 0) {
          o["multiply_by"] = mul;
        } else {
          o["multiply_by"] = nullptr;
        }
        o["invert"] = clk->invert();

        const sta::IntSeq& edges = clk->edges();
        if (!edges.empty()) {
          boost::json::array arr;
          for (int e : edges) {
            arr.push_back(e);
          }
          o["edges"] = std::move(arr);
        } else {
          o["edges"] = nullptr;
        }

        const sta::FloatSeq& shifts = clk->edgeShifts();
        if (!shifts.empty()) {
          boost::json::array arr;
          for (float s : shifts) {
            arr.push_back(scaleTime(s, time_scale));
          }
          o["edge_shifts"] = std::move(arr);
        } else {
          o["edge_shifts"] = nullptr;
        }

        sta::Pin* src = clk->srcPin();
        if (src && network) {
          o["src_pin"] = std::string(network->pathName(src));
          emitPinOdbRef(o, "src_pin", src, db_network);
        } else {
          o["src_pin"] = nullptr;
        }
      } else {
        o["divide_by"] = nullptr;
        o["multiply_by"] = nullptr;
        o["invert"] = false;
        o["edges"] = nullptr;
        o["edge_shifts"] = nullptr;
        o["src_pin"] = nullptr;
      }

      // Source pins.
      {
        boost::json::array sources;
        if (network) {
          for (const sta::Pin* pin : clk->pins()) {
            if (pin) {
              sources.push_back(boost::json::value(network->pathName(pin)));
            }
          }
        }
        o["sources"] = std::move(sources);
      }
      {
        boost::json::array sources_odb;
        if (network) {
          for (const sta::Pin* pin : clk->pins()) {
            if (!pin) {
              continue;
            }
            boost::json::object inner;
            emitPinOdbBare(inner, pin, db_network);
            sources_odb.push_back(std::move(inner));
          }
        }
        o["sources_odb"] = std::move(sources_odb);
      }

      emitUncertainty(o,
                      "uncertainty_setup",
                      "uncertainty_hold",
                      extractUncertainty(&clk->uncertainties()),
                      time_scale);

      clocks_arr.push_back(std::move(o));
    }
    root["clocks"] = std::move(clocks_arr);
  }

  // ── "clock_tree" array — root clocks with recursive generated children ──
  {
    boost::json::array tree;
    for (sta::Clock* clk : clocks) {
      if (!clk) {
        continue;
      }
      if (!clk->masterClk()) {
        tree.push_back(buildTreeNode(clk, childrenMap));
      }
    }
    root["clock_tree"] = std::move(tree);
  }

  return jsonResponse(req.id, boost::json::serialize(root));
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

  boost::json::object root;
  root["current_mode"] = mode_name;

  {
    boost::json::array scene_names;
    scene_names.push_back(boost::json::value(scene->name()));
    root["scene_names"] = std::move(scene_names);
  }

  {
    boost::json::array case_analysis;
    for (const auto& [pin, val] : sdc->caseLogicValues()) {
      if (!pin) {
        continue;
      }
      boost::json::object o;
      if (network) {
        o["pin"] = std::string(network->pathName(pin));
      } else {
        o["pin"] = "";
      }
      o["value"] = std::string(logicValueStr(val));
      emitPinOdbRef(o, "pin", pin, db_network);
      case_analysis.push_back(std::move(o));
    }
    root["case_analysis"] = std::move(case_analysis);
  }

  {
    boost::json::array logic_values;
    for (const auto& [pin, val] : sdc->logicValues()) {
      if (!pin) {
        continue;
      }
      boost::json::object o;
      if (network) {
        o["pin"] = std::string(network->pathName(pin));
      } else {
        o["pin"] = "";
      }
      emitPinOdbRef(o, "pin", pin, db_network);
      o["value"] = std::string(logicValueStr(val));
      logic_values.push_back(std::move(o));
    }
    root["logic_values"] = std::move(logic_values);
  }

  return jsonResponse(req.id, boost::json::serialize(root));
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
  // STA scenario for clockDomains() lookups — used to expose the
  // clocks STA actually propagated to each port pin so the frontend
  // can flag mismatches against the constraint clock.
  sta::Mode* mode = ctx->sta ? ctx->sta->cmdMode() : nullptr;

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

  struct RawDelay
  {
    sta::PortDelay* pd;
    const sta::Pin* pin;
    bool is_input;
    std::string port_name;
    int order;
  };
  std::vector<RawDelay> raw;
  std::set<std::string> emitted_names;
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

    std::set<std::pair<std::string, int>> seen_port_exc;
    auto noteExcPin = [&](sta::ExceptionPath* exc, const sta::Pin* pin) {
      if (!pin || !network->isTopLevelPort(pin)) {
        return;
      }
      const std::string nm = std::string(network->pathName(pin));
      const int id = exc ? static_cast<int>(exc->id()) : 0;
      if (!seen_port_exc.insert({nm, id}).second) {
        return;
      }
      ++port_exc_count[nm];
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

  const int total = static_cast<int>(raw.size());
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

  std::map<std::string, int> clocks_total;
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

  boost::json::object root;
  root["time_unit"] = time_suffix;
  root["cap_unit"] = cap_suffix;
  root["total"] = total;
  root["offset"] = offset;
  {
    boost::json::object o;
    for (const auto& [name, count] : clocks_total) {
      o[name] = count;
    }
    root["clocks_total"] = std::move(o);
  }
  {
    boost::json::object o;
    int all_count = 0;
    for (const auto& [dir, ports] : dir_total_ports) {
      o[dir] = static_cast<int>(ports.size());
      all_count += static_cast<int>(ports.size());
    }
    o["all"] = all_count;
    root["directions_total"] = std::move(o);
  }

  // Lightweight per-port metadata for the WHOLE port list (every
  // entry, not just the requested page). Frontend uses this for
  // filter-chip counts so "input (N)" / "output (N)" / per-clock
  // counts honour the active pattern + clock filter against the
  // full design, not just what's been paginated in. Without this,
  // chip counts grow as the user scrolls and read wrong under a
  // pattern filter that excludes most of the loaded subset.
  {
    boost::json::array port_meta;
    for (const RawDelay& rd : raw) {
      boost::json::object pm;
      pm["port"] = rd.port_name;
      pm["direction"]
          = std::string(portDir.count(rd.port_name)
                            ? portDir.at(rd.port_name)
                            : (rd.is_input ? "input" : "output"));
      sta::Clock* clk = rd.pd ? rd.pd->clock() : nullptr;
      if (clk) {
        pm["clock"] = std::string(clk->name());
      } else {
        pm["clock"] = nullptr;
      }
      pm["is_input"] = rd.is_input;
      port_meta.push_back(std::move(pm));
    }
    root["port_meta"] = std::move(port_meta);
  }

  boost::json::array port_delays;

  auto emitDelay = [&](const RawDelay& rd) {
    sta::PortDelay* pd = rd.pd;
    bool is_input = rd.is_input;
    const sta::Pin* pin = rd.pin;
    if (!pin) {
      return;
    }
    const bool is_exception_only = (pd == nullptr);

    boost::json::object o;
    o["port"] = rd.port_name;
    emitPinOdbRef(o, "port", pin, db_network);
    o["is_input"] = is_input;
    o["direction"]
        = std::string(portDir.count(rd.port_name)
                          ? portDir.at(rd.port_name)
                          : (is_input ? "input" : "output"));
    o["exception_only"] = is_exception_only;
    const bool is_clock_port = sdc && pin && sdc->isClock(pin);
    o["is_clock_port"] = is_clock_port;
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
    o["is_unconstrained"] = unconstrained;
    o["exception_count"]
        = port_exc_count.count(rd.port_name) ? port_exc_count.at(rd.port_name)
                                             : 0;

    // Clocks STA propagated to this port pin (whatever the timing
    // graph actually carries here, regardless of the SDC constraint
    // clock). The frontend compares this to `clock` below and flags
    // a mismatch when they diverge — often intentional cross-clock
    // IO, but the user should see it. Empty when timing graph is
    // unavailable or the port has no propagated clock.
    {
      boost::json::array actual_clocks;
      if (mode && pin) {
        sta::ClockSet domains = ctx->sta->clockDomains(pin, mode);
        for (const sta::Clock* c : domains) {
          if (c) {
            actual_clocks.push_back(boost::json::value(c->name()));
          }
        }
      }
      o["actual_clocks"] = std::move(actual_clocks);
    }

    if (is_exception_only) {
      o["source_latency_included"] = nullptr;
      o["network_latency_included"] = nullptr;
      o["ref_pin"] = nullptr;
      o["clock"] = nullptr;
      o["clk_period"] = nullptr;
      o["uncertainty_setup"] = nullptr;
      o["uncertainty_hold"] = nullptr;
      o["clk_edge"] = nullptr;
      o["rise_max"] = nullptr;
      o["rise_min"] = nullptr;
      o["fall_max"] = nullptr;
      o["fall_min"] = nullptr;
    } else {
      o["source_latency_included"] = pd->sourceLatencyIncluded();
      o["network_latency_included"] = pd->networkLatencyIncluded();
      if (const sta::Pin* ref = pd->refPin()) {
        o["ref_pin"]
            = std::string(network ? network->pathName(ref) : "");
        emitPinOdbRef(o, "ref_pin", ref, db_network);
      } else {
        o["ref_pin"] = nullptr;
      }

      sta::Clock* clk = pd->clock();
      if (clk) {
        o["clock"] = std::string(clk->name());
        o["clk_period"] = scaleTime(clk->period(), time_scale);
        emitUncertainty(o,
                        "uncertainty_setup",
                        "uncertainty_hold",
                        extractUncertainty(&clk->uncertainties()),
                        time_scale);
      } else {
        o["clock"] = nullptr;
        o["clk_period"] = nullptr;
        o["uncertainty_setup"] = nullptr;
        o["uncertainty_hold"] = nullptr;
      }

      const sta::ClockEdge* clk_edge = pd->clkEdge();
      if (clk_edge) {
        o["clk_edge"] = std::string(clk_edge->transition()->name());
      } else {
        o["clk_edge"] = nullptr;
      }

      emitRiseFallMinMax(o, pd->delays(), time_scale);
    }

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
          o["driving_cell"] = std::string(cell->name());
          if (to_port) {
            o["driving_pin"] = std::string(to_port->name());
          } else {
            o["driving_pin"] = nullptr;
          }
        } else {
          o["driving_cell"] = nullptr;
          o["driving_pin"] = nullptr;
        }
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
              o[key] = static_cast<double>(val) / scale;
            } else {
              o[key] = nullptr;
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
            1.0);
      } else {
        o["driving_cell"] = nullptr;
        o["driving_pin"] = nullptr;
        for (const char* k : {"drive_slew_rise_max",
                              "drive_slew_rise_min",
                              "drive_slew_fall_max",
                              "drive_slew_fall_min",
                              "drive_res_rise_max",
                              "drive_res_rise_min",
                              "drive_res_fall_max",
                              "drive_res_fall_min"}) {
          o[k] = nullptr;
        }
      }
    } else {
      o["driving_cell"] = nullptr;
      o["driving_pin"] = nullptr;
      for (const char* k : {"drive_slew_rise_max",
                            "drive_slew_rise_min",
                            "drive_slew_fall_max",
                            "drive_slew_fall_min",
                            "drive_res_rise_max",
                            "drive_res_rise_min",
                            "drive_res_fall_max",
                            "drive_res_fall_min"}) {
        o[k] = nullptr;
      }
    }

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
        o["load_cap"] = static_cast<double>(pinc / cap_scale);
      } else {
        o["load_cap"] = nullptr;
      }
      if (wirec_ex) {
        o["wire_cap"] = static_cast<double>(wirec / cap_scale);
      } else {
        o["wire_cap"] = nullptr;
      }
      if (fan_ex) {
        o["fanout_load"] = fan;
      } else {
        o["fanout_load"] = nullptr;
      }
    } else {
      o["load_cap"] = nullptr;
      o["wire_cap"] = nullptr;
      o["fanout_load"] = nullptr;
    }

    port_delays.push_back(std::move(o));
  };

  for (int i = offset; i < end; ++i) {
    emitDelay(raw[i]);
  }

  root["port_delays"] = std::move(port_delays);
  return jsonResponse(req.id, boost::json::serialize(root));
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

  auto buildPins = [&](const sta::PinSet* pins) {
    boost::json::array arr;
    if (pins && network) {
      for (const sta::Pin* pin : *pins) {
        if (!pin) {
          continue;
        }
        boost::json::object o;
        o["name"] = std::string(network->pathName(pin));
        emitPinOdbBare(o, pin, db_network);
        arr.push_back(std::move(o));
      }
    }
    return arr;
  };

  auto buildClocks = [&](const sta::ClockSet* clks) {
    boost::json::array arr;
    if (clks) {
      for (sta::Clock* clk : *clks) {
        if (clk) {
          arr.push_back(boost::json::value(clk->name()));
        }
      }
    }
    return arr;
  };

  auto buildInsts = [&](const sta::InstanceSet* insts) {
    boost::json::array arr;
    if (insts && network) {
      for (const sta::Instance* inst : *insts) {
        if (!inst) {
          continue;
        }
        boost::json::object o;
        o["name"] = std::string(network->pathName(inst));
        emitInstanceOdbBare(o, inst, db_network);
        arr.push_back(std::move(o));
      }
    }
    return arr;
  };

  auto buildNets = [&](const sta::NetSet* nets) {
    boost::json::array arr;
    if (nets && network) {
      for (const sta::Net* net : *nets) {
        if (net) {
          arr.push_back(boost::json::value(network->pathName(net)));
        }
      }
    }
    return arr;
  };

  auto setTransition = [&](boost::json::object& obj,
                           const char* key,
                           const sta::RiseFallBoth* rf) {
    if (rf) {
      obj[key] = std::string(rf->name());
    } else {
      obj[key] = nullptr;
    }
  };

  boost::json::object root;
  root["time_unit"] = time_suffix;
  boost::json::array exceptions;

  for (sta::ExceptionPath* exc : sdc->exceptions()) {
    if (!exc) {
      continue;
    }
    if (exc->isFilter() || exc->isLoop()) {
      continue;
    }

    boost::json::object o;
    o["type"] = std::string(exceptionTypeStr(exc));

    if (exc->isGroupPath()) {
      o["name"] = std::string(exc->name());
      o["is_default"] = exc->isDefault();
    } else {
      o["name"] = nullptr;
      o["is_default"] = false;
    }

    const sta::MinMaxAll* mm = exc->minMax();
    o["min_max"] = mm ? mm->to_string() : std::string("max");

    o["multiplier"] = exc->pathMultiplier();

    if (exc->isPathDelay()) {
      o["delay"] = scaleTime(exc->delay(), time_scale);
    } else {
      o["delay"] = nullptr;
    }

    o["ignore_clk_latency"] = exc->ignoreClkLatency();
    o["break_path"] = exc->breakPath();
    o["use_end_clk"] = exc->useEndClk();

    sta::ExceptionFrom* from = exc->from();
    o["from_pins"] = buildPins(from ? from->pins() : nullptr);
    o["from_clocks"] = buildClocks(from ? from->clks() : nullptr);
    o["from_insts"] = buildInsts(from ? from->instances() : nullptr);
    setTransition(o, "from_transition", from ? from->transition() : nullptr);

    {
      boost::json::array thrus;
      if (exc->thrus()) {
        for (sta::ExceptionThru* thru : *exc->thrus()) {
          if (!thru) {
            continue;
          }
          boost::json::object t;
          t["pins"] = buildPins(thru->pins());
          t["clocks"] = buildClocks(thru->clks());
          t["insts"] = buildInsts(thru->instances());
          t["nets"] = buildNets(thru->nets());
          setTransition(t, "transition", thru->transition());
          thrus.push_back(std::move(t));
        }
      }
      o["thrus"] = std::move(thrus);
    }

    sta::ExceptionTo* to = exc->to();
    o["to_pins"] = buildPins(to ? to->pins() : nullptr);
    o["to_clocks"] = buildClocks(to ? to->clks() : nullptr);
    o["to_insts"] = buildInsts(to ? to->instances() : nullptr);
    setTransition(o, "to_transition", to ? to->endTransition() : nullptr);

    o["id"] = static_cast<int>(exc->id());
    o["comment"] = std::string(exc->comment());

    exceptions.push_back(std::move(o));
  }

  root["exceptions"] = std::move(exceptions);
  return jsonResponse(req.id, boost::json::serialize(root));
}

// ── Network latencies (set_clock_latency without -source). Source
//    latencies emit below as clock_insertions; is_source discriminates.
static void emitClockLatencies(boost::json::object& root, const SdcContext& ctx)
{
  boost::json::array arr;
  for (sta::ClockLatency* lat : *ctx.sdc->clockLatencies()) {
    if (!lat) {
      continue;
    }
    boost::json::object o;
    const sta::Clock* clk = lat->clock();
    if (clk) {
      o["clock"] = std::string(clk->name());
    } else {
      o["clock"] = nullptr;
    }
    const sta::Pin* pin = lat->pin();
    if (pin && ctx.network) {
      o["pin"] = std::string(ctx.network->pathName(pin));
    } else {
      o["pin"] = nullptr;
    }
    o["is_source"] = false;
    emitRiseFallMinMax(o, lat->delays(), ctx.time_scale);
    arr.push_back(std::move(o));
  }
  root["clock_latencies"] = std::move(arr);
}

// ── Clock insertion delays (set_clock_latency -source) ──
static void emitClockInsertions(boost::json::object& root,
                                const SdcContext& ctx)
{
  boost::json::array arr;
  for (sta::ClockInsertion* ins : ctx.sdc->clockInsertions()) {
    if (!ins) {
      continue;
    }
    boost::json::object o;
    const sta::Clock* clk = ins->clock();
    if (clk) {
      o["clock"] = std::string(clk->name());
    } else {
      o["clock"] = nullptr;
    }
    const sta::Pin* pin = ins->pin();
    if (pin && ctx.network) {
      o["pin"] = std::string(ctx.network->pathName(pin));
    } else {
      o["pin"] = nullptr;
    }
    o["is_source"] = true;
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
        o[s.key] = scaleTime(val, ctx.time_scale);
      } else {
        o[s.key] = nullptr;
      }
    }
    arr.push_back(std::move(o));
  }
  root["clock_insertions"] = std::move(arr);
}

// ── Per-clock uncertainties (set_clock_uncertainty -clock) ──────────────
static void emitClockUncertainties(boost::json::object& root,
                                   const SdcContext& ctx)
{
  boost::json::array arr;
  for (const sta::Clock* clk : ctx.sdc->clocks()) {
    if (!clk) {
      continue;
    }
    UncertaintyValues u = extractUncertainty(&clk->uncertainties());
    if (!u.any()) {
      continue;
    }
    boost::json::object o;
    o["clock"] = std::string(clk->name());
    emitUncertainty(o, "setup", "hold", u, ctx.time_scale);
    arr.push_back(std::move(o));
  }
  root["clock_uncertainties"] = std::move(arr);
}

// ── Top-level port loads and limits ─────────────────────────────────────
static void emitPortLoads(boost::json::object& root, const SdcContext& ctx)
{
  boost::json::array arr;
  if (ctx.network) {
    const sta::Instance* top = ctx.network->topInstance();
    if (top) {
      const sta::Cell* top_cell = ctx.network->cell(top);
      std::unique_ptr<sta::CellPortIterator> it(
          ctx.network->portIterator(top_cell));
      while (it->hasNext()) {
        sta::Port* port = it->next();
        if (!port) {
          continue;
        }

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

        boost::json::object o;
        o["port"] = std::string(ctx.network->name(port));

        if (ext_cap) {
          const sta::RiseFallMinMax* pin_cap = ext_cap->pinCap();
          const sta::RiseFallMinMax* wire_cap = ext_cap->wireCap();
          emitRiseFallMinMax(o, pin_cap, ctx.cap_scale);
          float wc;
          bool wc_ex;
          if (wire_cap) {
            wire_cap->value(
                sta::RiseFall::rise(), sta::MinMax::max(), wc, wc_ex);
          } else {
            wc_ex = false;
          }
          if (wc_ex) {
            o["wire_cap_max"] = static_cast<double>(wc / ctx.cap_scale);
          } else {
            o["wire_cap_max"] = nullptr;
          }
        } else {
          o["rise_max"] = nullptr;
          o["rise_min"] = nullptr;
          o["fall_max"] = nullptr;
          o["fall_min"] = nullptr;
          o["wire_cap_max"] = nullptr;
        }

        if (slew_max_ex) {
          o["slew_max"] = scaleTime(slew_max, ctx.time_scale);
        } else {
          o["slew_max"] = nullptr;
        }
        if (slew_min_ex) {
          o["slew_min"] = scaleTime(slew_min, ctx.time_scale);
        } else {
          o["slew_min"] = nullptr;
        }

        if (cap_max_ex) {
          o["cap_limit"] = static_cast<double>(cap_max / ctx.cap_scale);
        } else {
          o["cap_limit"] = nullptr;
        }

        if (fanout_max_ex) {
          o["fanout_limit"] = static_cast<int>(fanout_max);
        } else {
          o["fanout_limit"] = nullptr;
        }

        arr.push_back(std::move(o));
      }
    }
  }
  root["port_loads"] = std::move(arr);
}

// ── Disabled timing (set_disable_timing) ────────────────────────────────
static void emitDisabledTiming(boost::json::object& root,
                               const SdcContext& ctx)
{
  boost::json::array arr;
  auto buildPortList = [&](const sta::LibertyPortSet* ports) {
    boost::json::array a;
    if (ports) {
      for (const sta::LibertyPort* lp : *ports) {
        if (lp) {
          a.push_back(boost::json::value(lp->name()));
        }
      }
    }
    return a;
  };
  auto buildFromToPairs = [&](const sta::LibertyPortPairSet* pairs) {
    boost::json::array a;
    if (pairs) {
      for (const sta::LibertyPortPair& p : *pairs) {
        boost::json::object pp;
        pp["from"]
            = std::string(p.first ? p.first->name() : "");
        pp["to"] = std::string(p.second ? p.second->name() : "");
        a.push_back(std::move(pp));
      }
    }
    return a;
  };
  auto setDisabledPorts
      = [&](boost::json::object& obj, const sta::DisabledPorts* dp) {
          obj["all"] = dp ? dp->all() : false;
          obj["from"] = buildPortList(dp ? dp->from() : nullptr);
          obj["to"] = buildPortList(dp ? dp->to() : nullptr);
          obj["from_to"] = buildFromToPairs(dp ? dp->fromTo() : nullptr);
        };

  if (const sta::PinSet* pins = ctx.sdc->disabledPins()) {
    for (const sta::Pin* pin : *pins) {
      if (!pin) {
        continue;
      }
      boost::json::object o;
      o["scope"] = std::string("pin");
      if (ctx.network) {
        o["name"] = std::string(ctx.network->pathName(pin));
      } else {
        o["name"] = std::string("");
      }
      arr.push_back(std::move(o));
    }
  }
  if (const sta::PortSet* ports = ctx.sdc->disabledPorts()) {
    for (const sta::Port* port : *ports) {
      if (!port) {
        continue;
      }
      boost::json::object o;
      o["scope"] = std::string("port");
      if (ctx.network) {
        o["name"] = std::string(ctx.network->name(port));
      } else {
        o["name"] = std::string("");
      }
      arr.push_back(std::move(o));
    }
  }
  if (const sta::LibertyPortSet* lports = ctx.sdc->disabledLibPorts()) {
    for (const sta::LibertyPort* lp : *lports) {
      if (!lp) {
        continue;
      }
      boost::json::object o;
      o["scope"] = std::string("lib_port");
      o["name"] = std::string(lp->name());
      arr.push_back(std::move(o));
    }
  }
  auto buildTimingArcSets = [&](const sta::DisabledCellPorts* dcp) {
    const sta::TimingArcSetSet* sets = dcp ? dcp->timingArcSets() : nullptr;
    boost::json::array a;
    if (sets) {
      for (sta::TimingArcSet* arcset : *sets) {
        if (arcset) {
          a.push_back(boost::json::value(arcset->to_string()));
        }
      }
    }
    return a;
  };
  if (const sta::DisabledCellPortsMap* cm = ctx.sdc->disabledCellPorts()) {
    for (const auto& [cell, dcp] : *cm) {
      if (!cell || !dcp) {
        continue;
      }
      boost::json::object o;
      o["scope"] = std::string("cell_port");
      o["name"] = std::string(cell->name());
      setDisabledPorts(o, dcp);
      o["timing_arc_sets"] = buildTimingArcSets(dcp);
      arr.push_back(std::move(o));
    }
  }
  if (const sta::DisabledInstancePortsMap* im
      = ctx.sdc->disabledInstancePorts()) {
    for (const auto& [inst, dip] : *im) {
      if (!inst || !dip) {
        continue;
      }
      boost::json::object o;
      o["scope"] = std::string("inst_port");
      if (ctx.network) {
        o["name"] = std::string(ctx.network->pathName(inst));
      } else {
        o["name"] = std::string("");
      }
      setDisabledPorts(o, dip);
      o["timing_arc_sets"] = boost::json::array();
      arr.push_back(std::move(o));
    }
  }
  root["disabled_timing"] = std::move(arr);
}

// ── Design-wide cell limits ───────────────────────────────────────────
static void emitCellLimits(boost::json::object& root, const SdcContext& ctx)
{
  boost::json::array arr;
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
        boost::json::object o;
        o["scope"] = std::string("design");
        o["name"] = std::string(ctx.network->name(top_cell));
        if (slew_max_ex) {
          o["slew_max"] = scaleTime(slew_max, ctx.time_scale);
        } else {
          o["slew_max"] = nullptr;
        }
        if (slew_min_ex) {
          o["slew_min"] = scaleTime(slew_min, ctx.time_scale);
        } else {
          o["slew_min"] = nullptr;
        }
        if (cap_max_ex) {
          o["cap_limit"] = static_cast<double>(cap_max / ctx.cap_scale);
        } else {
          o["cap_limit"] = nullptr;
        }
        if (fanout_max_ex) {
          o["fanout_limit"] = static_cast<int>(fanout_max);
        } else {
          o["fanout_limit"] = nullptr;
        }
        arr.push_back(std::move(o));
      }
    }
  }
  root["cell_limits"] = std::move(arr);
}

// ── Clock-level slew limits ───────────────────────────────────────────
static void emitClockSlewLimits(boost::json::object& root,
                                const SdcContext& ctx)
{
  boost::json::array arr;
  if (ctx.sdc->haveClkSlewLimits()) {
    const sta::PathClkOrData clk_only = sta::PathClkOrData::clk;
    const sta::PathClkOrData data_only = sta::PathClkOrData::data;
    for (const sta::Clock* clk : ctx.sdc->clocks()) {
      if (!clk) {
        continue;
      }
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
      boost::json::object o;
      o["clock"] = std::string(clk->name());
      for (int i = 0; i < 8; ++i) {
        if (ex[i]) {
          o[slots[i].key] = scaleTime(vals[i], ctx.time_scale);
        } else {
          o[slots[i].key] = nullptr;
        }
      }
      arr.push_back(std::move(o));
    }
  }
  root["clock_slew_limits"] = std::move(arr);
}

// ── Pin-anchored clock uncertainty ───────────────────────────────────
static void emitPinClockUncertainties(boost::json::object& root,
                                      const SdcContext& ctx)
{
  boost::json::array arr;
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
        boost::json::object o;
        o["pin"] = std::string(ctx.network->pathName(pin));
        emitPinOdbRef(o, "pin", pin, ctx.db_network);
        emitUncertainty(o, "setup", "hold", u, ctx.time_scale);
        arr.push_back(std::move(o));
      }
    }
  }
  root["pin_clock_uncertainties"] = std::move(arr);
}

// ── Pin-scope cap limits ──────────────────────────────────────────
static void emitPinCapLimits(boost::json::object& root, const SdcContext& ctx)
{
  boost::json::array arr;
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
        sta::Pin* mut_pin = const_cast<sta::Pin*>(pin);
        ctx.sdc->capacitanceLimit(
            mut_pin, sta::MinMax::max(), cap_max, cap_max_ex);
        ctx.sdc->capacitanceLimit(
            mut_pin, sta::MinMax::min(), cap_min, cap_min_ex);
        if (!cap_max_ex && !cap_min_ex) {
          continue;
        }
        boost::json::object o;
        o["pin"] = std::string(ctx.network->pathName(pin));
        emitPinOdbRef(o, "pin", pin, ctx.db_network);
        if (cap_max_ex) {
          o["cap_max"] = static_cast<double>(cap_max / ctx.cap_scale);
        } else {
          o["cap_max"] = nullptr;
        }
        if (cap_min_ex) {
          o["cap_min"] = static_cast<double>(cap_min / ctx.cap_scale);
        } else {
          o["cap_min"] = nullptr;
        }
        arr.push_back(std::move(o));
      }
    }
  }
  root["pin_cap_limits"] = std::move(arr);
}

WebSocketResponse SdcHandler::handleSdcLimits(const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return emptyLimits(req.id);
  }

  boost::json::object root;
  root["time_unit"] = ctx->time_suffix;
  root["cap_unit"] = ctx->cap_suffix;

  emitClockLatencies(root, *ctx);
  emitClockInsertions(root, *ctx);
  emitClockUncertainties(root, *ctx);
  emitPortLoads(root, *ctx);
  emitDisabledTiming(root, *ctx);
  emitCellLimits(root, *ctx);
  emitClockSlewLimits(root, *ctx);
  emitPinClockUncertainties(root, *ctx);
  emitPinCapLimits(root, *ctx);

  return jsonResponse(req.id, boost::json::serialize(root));
}

WebSocketResponse SdcHandler::handleSdcClockGroups(const WebSocketRequest& req)
{
  auto ctx = makeSdcContext(gen_);
  if (!ctx) {
    return jsonResponse(req.id, R"({"groups":[]})");
  }
  sta::Sdc* sdc = ctx->sdc;

  boost::json::object root;
  boost::json::array groups;

  for (const auto& [name, cg] : sdc->clockGroupsNameMap()) {
    if (!cg) {
      continue;
    }
    boost::json::object o;
    o["name"] = name;

    const char* type_str = cg->asynchronous()          ? "asynchronous"
                           : cg->logicallyExclusive()  ? "logically_exclusive"
                           : cg->physicallyExclusive() ? "physically_exclusive"
                                                       : "unknown";
    o["type"] = std::string(type_str);
    o["allow_paths"] = cg->allowPaths();
    o["comment"] = std::string(cg->comment());

    boost::json::array clk_sets;
    for (const sta::ClockGroup* clk_set : *cg->groups()) {
      if (!clk_set) {
        continue;
      }
      boost::json::array set_arr;
      for (const sta::Clock* clk : *clk_set) {
        if (clk) {
          set_arr.push_back(boost::json::value(clk->name()));
        }
      }
      clk_sets.push_back(std::move(set_arr));
    }
    o["clk_sets"] = std::move(clk_sets);

    groups.push_back(std::move(o));
  }

  root["groups"] = std::move(groups);
  return jsonResponse(req.id, boost::json::serialize(root));
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

  // Emit all SDC constraint data for one resolved pin into the open object.
  auto emitPinData = [&](boost::json::object& obj, const sta::Pin* pin) {
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
      obj["direction"] = std::string(dir_str);
    }
    obj["is_clock_pin"] = lp ? lp->isRegClk() : false;

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
            if (!capture_edge && arc->fromEdge()) {
              capture_edge = arc->fromEdge();
            }
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
      obj["capture_edge"] = std::string("rise");
    } else if (capture_edge == sta::Transition::fall()) {
      obj["capture_edge"] = std::string("fall");
    } else {
      obj["capture_edge"] = nullptr;
    }

    double clkq[2][2] = {{0, 0}, {0, 0}};
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
      boost::json::object clkq_obj;
      auto setClkq = [&](const char* key, int rfi, int mmi) {
        if (has_clkq[rfi][mmi]) {
          clkq_obj[key] = clkq[rfi][mmi];
        } else {
          clkq_obj[key] = nullptr;
        }
      };
      setClkq("rise_max", 0, 0);
      setClkq("rise_min", 0, 1);
      setClkq("fall_max", 1, 0);
      setClkq("fall_min", 1, 1);
      obj["clk_to_q"] = std::move(clkq_obj);
    } else {
      obj["clk_to_q"] = nullptr;
    }

    const bool is_latch_data = lp && lp->isLatchData();
    const sta::Pin* enable_pin_resolved = nullptr;
    if (is_latch_data && enable_port) {
      sta::Instance* inst = network->instance(pin);
      if (inst) {
        enable_pin_resolved = network->findPin(inst, enable_port);
      }
    }
    {
      boost::json::array clocks_arr;
      if (mode) {
        sta::ClockSet domains = sta->clockDomains(pin, mode);
        for (const sta::Clock* clk : domains) {
          if (!clk) {
            continue;
          }
          boost::json::object co;
          co["name"] = std::string(clk->name());
          co["period"] = scaleTime(clk->period(), time_scale);
          const sta::FloatSeq& wf = clk->waveform();
          boost::json::array wf_arr;
          if (!wf.empty()) {
            for (float t : wf) {
              wf_arr.push_back(scaleTime(t, time_scale));
            }
          } else {
            wf_arr.push_back(0.0);
            wf_arr.push_back(scaleTime(clk->period(), time_scale) / 2.0);
          }
          co["waveform"] = std::move(wf_arr);
          emitUncertainty(co,
                          "uncertainty_setup",
                          "uncertainty_hold",
                          extractUncertainty(&clk->uncertainties()),
                          time_scale);
          if (has_lib_setup) {
            co["library_setup"] = scaleTime(lib_setup, time_scale);
          } else {
            co["library_setup"] = nullptr;
          }
          if (has_lib_hold) {
            co["library_hold"] = scaleTime(lib_hold, time_scale);
          } else {
            co["library_hold"] = nullptr;
          }
          if (is_latch_data) {
            float limit = 0.0f;
            bool has_limit = false;
            sdc->latchBorrowLimit(pin,
                                  enable_pin_resolved,
                                  const_cast<sta::Clock*>(clk),
                                  limit,
                                  has_limit);
            if (has_limit) {
              co["time_borrow_limit"] = scaleTime(limit, time_scale);
            } else {
              co["time_borrow_limit"] = nullptr;
            }
          } else {
            co["time_borrow_limit"] = nullptr;
          }
          clocks_arr.push_back(std::move(co));
        }
      }
      obj["clocks"] = std::move(clocks_arr);
    }

    {
      boost::json::array exc_arr;
      // Resolved MCP fields. Best-effort: pick the FIRST applicable
      // setup MCP and the FIRST applicable hold MCP per pin. Counts
      // track how many MCPs match each direction so the frontend can
      // surface a "+N more constraints apply" hint when multiple
      // exceptions reach the same pin.
      //
      // SDC `min_max` semantics:
      //   "max" → setup-side (default for set_multicycle_path)
      //   "min" → hold-side
      //   "all" → applies to BOTH (rare; matches both buckets)
      // We do NOT attempt to resolve scope-precedence here (from-pin
      // > from-clock > global) — STA's resolution is path-specific
      // and the visualization only needs a representative cycle
      // count. If a more-specific MCP exists, it lands first in
      // pin_exceptions because that's the order STA emits them.
      // The "+N more" indicator is the user-facing escape hatch
      // when the chosen MCP disagrees with their intent.
      int mcp_setup_cycles = -1;     // -1 = unset (no MCP applies)
      int mcp_hold_cycles  = -1;
      int mcp_setup_count  = 0;
      int mcp_hold_count   = 0;
      auto eit = pin_exceptions.find(pin);
      if (eit != pin_exceptions.end()) {
        for (sta::ExceptionPath* exc : eit->second) {
          boost::json::object eo;
          eo["type"] = std::string(exceptionTypeStr(exc));
          const sta::MinMaxAll* mm = exc->minMax();
          const std::string mm_str
              = mm ? mm->to_string() : std::string("max");
          eo["min_max"] = mm_str;
          const int mult = exc->pathMultiplier();
          eo["multiplier"] = mult;
          if (exc->isPathDelay()) {
            eo["delay"] = scaleTime(exc->delay(), time_scale);
          } else {
            eo["delay"] = nullptr;
          }
          if (exc->isMultiCycle()) {
            const bool applies_setup
                = (mm_str == "max" || mm_str == "all");
            const bool applies_hold
                = (mm_str == "min" || mm_str == "all");
            if (applies_setup) {
              ++mcp_setup_count;
              if (mcp_setup_cycles < 0) {
                mcp_setup_cycles = mult;
              }
            }
            if (applies_hold) {
              ++mcp_hold_count;
              if (mcp_hold_cycles < 0) {
                mcp_hold_cycles = mult;
              }
            }
          }
          exc_arr.push_back(std::move(eo));
        }
      }
      obj["exceptions"] = std::move(exc_arr);
      // Sparse emission: only set when an MCP actually applies.
      // Absent fields = no MCP = default (setup=1 cycle, hold=0
      // cycles in SDC semantics). Lets the frontend render a
      // "no MCP" lane without an explicit boolean.
      if (mcp_setup_cycles >= 0) {
        obj["mcp_setup_cycles"] = mcp_setup_cycles;
        obj["mcp_setup_count"] = mcp_setup_count;
      }
      if (mcp_hold_cycles >= 0) {
        obj["mcp_hold_cycles"] = mcp_hold_cycles;
        obj["mcp_hold_count"] = mcp_hold_count;
      }
    }

    {
      boost::json::array pd_arr;
      if (network->isTopLevelPort(pin)) {
        auto emitPortDelay = [&](sta::PortDelay* pd, bool is_input) {
          if (!pd) {
            return;
          }
          boost::json::object po;
          po["is_input"] = is_input;
          sta::Clock* clk = pd->clock();
          if (clk) {
            po["clock"] = std::string(clk->name());
            po["clk_period"] = scaleTime(clk->period(), time_scale);
            emitUncertainty(po,
                            "uncertainty_setup",
                            "uncertainty_hold",
                            extractUncertainty(&clk->uncertainties()),
                            time_scale);
          } else {
            po["clock"] = nullptr;
            po["clk_period"] = nullptr;
            po["uncertainty_setup"] = nullptr;
            po["uncertainty_hold"] = nullptr;
          }
          const sta::ClockEdge* clk_edge = pd->clkEdge();
          if (clk_edge) {
            po["clk_edge"] = std::string(clk_edge->transition()->name());
          } else {
            po["clk_edge"] = nullptr;
          }
          emitRiseFallMinMax(po, pd->delays(), time_scale);
          pd_arr.push_back(std::move(po));
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
      obj["port_delays"] = std::move(pd_arr);
    }
  };

  const std::string pat = extract_string(req.json, "pin");
  std::vector<const sta::Pin*> matched;
  if (sta::patternWildcards(pat)) {
    sta::PatternMatch pattern(pat);
    sta::Network* sdc_net = sta->sdcNetwork();
    if (sdc_net) {
      sta::PinSeq pins
          = sdc_net->findPinsMatching(sdc_net->topInstance(), &pattern);
      matched.assign(pins.begin(), pins.end());
    }
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

  const int total = static_cast<int>(matched.size());
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
  root["time_unit"] = time_suffix;
  root["found"] = !matched.empty();
  root["multi"] = total > 1;
  root["total"] = total;
  root["offset"] = offset;
  root["truncated"] = false;
  boost::json::array pins_arr;
  for (int i = offset; i < end; ++i) {
    const sta::Pin* p = matched[i];
    boost::json::object o;
    o["name"] = std::string(network->pathName(p));
    emitPinOdbRef(o, "name", p, db_network);
    o["is_port"] = network->isTopLevelPort(p);
    emitPinData(o, p);
    pins_arr.push_back(std::move(o));
  }
  root["pins"] = std::move(pins_arr);
  return jsonResponse(req.id, boost::json::serialize(root));
}

WebSocketResponse SdcHandler::handleSdcEndpointCounts(
    const WebSocketRequest& req)
{
  static constexpr const char* kEmpty
      = R"({"time_unit":"ns","clocks_total":{}})";
  auto ctx = makeSdcContext(gen_);
  if (!ctx || !ctx->network || !ctx->network->isLinked()) {
    return jsonResponse(req.id, kEmpty);
  }
  sta::dbSta* sta = ctx->sta;
  sta::Network* network = ctx->network;
  sta::Mode* mode = sta->cmdMode();
  sta->ensureGraph();
  sta::Search* search = sta->search();
  if (!search) {
    return jsonResponse(req.id, kEmpty);
  }

  std::map<std::string, int> clocks_total;
  sta::VertexSet& endpoints = search->endpoints();
  for (sta::Vertex* v : endpoints) {
    if (!v) {
      continue;
    }
    const sta::Pin* pin = v->pin();
    if (!pin) {
      continue;
    }
    if (network->isTopLevelPort(pin)) {
      continue;
    }
    if (!mode) {
      continue;
    }
    sta::ClockSet domains = sta->clockDomains(pin, mode);
    for (const sta::Clock* c : domains) {
      if (c) {
        clocks_total[c->name()] += 1;
      }
    }
  }

  boost::json::object root;
  root["time_unit"] = ctx->time_suffix;
  {
    boost::json::object o;
    for (const auto& [name, count] : clocks_total) {
      o[name] = count;
    }
    root["clocks_total"] = std::move(o);
  }
  return jsonResponse(req.id, boost::json::serialize(root));
}

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
  sta->ensureGraph();
  sta::Search* search = sta->search();
  if (!search) {
    return jsonResponse(req.id, kEmptyList);
  }
  const std::string& time_suffix = ctx->time_suffix;
  const float time_scale = ctx->time_scale;

  auto classify = [&](const sta::Pin* pin) -> const char* {
    sta::Instance* inst = network->instance(pin);
    if (!inst) {
      return "stdcell";
    }
    sta::LibertyCell* lc = network->libertyCell(inst);
    if (!lc) {
      return "stdcell";
    }
    if (lc->isClockGate()) {
      return "clock_gate";
    }
    if (lc->hasSequentials()) {
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
    }
    if (lc->isMacro() || lc->isMemory()) {
      return "macro";
    }
    return "stdcell";
  };

  const std::string pattern = extract_string(req.json, "pattern");
  std::unique_ptr<sta::PatternMatch> pat;
  if (!pattern.empty()) {
    pat = std::make_unique<sta::PatternMatch>(pattern);
  }
  const std::string kind_filter = extract_string(req.json, "kind");
  auto kindMatches = [&](const char* k) {
    if (kind_filter.empty() || kind_filter == "all") {
      return true;
    }
    return kind_filter == k;
  };
  std::set<std::string> clock_whitelist;
  {
    const std::string cf = extract_string(req.json, "clock");
    if (!cf.empty() && cf != "all") {
      std::string::size_type i = 0;
      while (i <= cf.size()) {
        std::string::size_type j = cf.find(',', i);
        if (j == std::string::npos) {
          j = cf.size();
        }
        std::string tok = cf.substr(i, j - i);
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
    std::vector<std::string> clocks;
  };
  std::vector<Hit> hits;
  std::set<const sta::Instance*> seen_flop;
  std::set<const sta::Instance*> seen_latch;
  std::set<const sta::Instance*> seen_macro;
  std::set<const sta::Instance*> seen_stdcell;
  std::set<const sta::Instance*> seen_clock_gate;
  std::map<std::string, std::set<const sta::Instance*>> clocks_total_insts;
  sta::VertexSet& endpoints = search->endpoints();
  hits.reserve(endpoints.size() / 4);
  for (sta::Vertex* v : endpoints) {
    if (!v) {
      continue;
    }
    const sta::Pin* pin = v->pin();
    if (!pin) {
      continue;
    }
    if (network->isTopLevelPort(pin)) {
      continue;
    }
    const char* kind = classify(pin);
    sta::Instance* inst = network->instance(pin);
    std::vector<std::string> clk_names;
    if (mode) {
      sta::ClockSet domains = sta->clockDomains(pin, mode);
      for (const sta::Clock* c : domains) {
        if (c) {
          clk_names.emplace_back(c->name());
        }
      }
    }
    std::string name = network->pathName(pin);
    const bool name_match = !pat || pat->match(name.c_str());
    const bool clock_match = clockMatches(clk_names);

    if (name_match && clock_match) {
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
    }
    if (name_match && kindMatches(kind)) {
      for (const auto& cn : clk_names) {
        clocks_total_insts[cn].insert(inst);
      }
    }
    if (!kindMatches(kind)) {
      continue;
    }
    if (!name_match) {
      continue;
    }
    if (!clock_match) {
      continue;
    }
    hits.push_back({pin, kind, std::move(name), std::move(clk_names)});
  }

  std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) {
    return a.name < b.name;
  });

  const int total = static_cast<int>(hits.size());
  std::set<sta::Instance*> hit_insts_unique;
  for (const Hit& h : hits) {
    sta::Instance* inst = network->instance(h.pin);
    if (inst) {
      hit_insts_unique.insert(inst);
    }
  }
  const int total_instances = static_cast<int>(hit_insts_unique.size());

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
  root["time_unit"] = time_suffix;
  root["total"] = total;
  root["total_instances"] = total_instances;
  root["offset"] = offset;
  {
    boost::json::object o;
    o["flipflop"] = static_cast<int>(seen_flop.size());
    o["latch"] = static_cast<int>(seen_latch.size());
    o["macro"] = static_cast<int>(seen_macro.size());
    o["stdcell"] = static_cast<int>(seen_stdcell.size());
    o["clock_gate"] = static_cast<int>(seen_clock_gate.size());
    root["kinds_total"] = std::move(o);
  }
  {
    boost::json::object o;
    for (const auto& [name, insts] : clocks_total_insts) {
      o[name] = static_cast<int>(insts.size());
    }
    root["clocks_total"] = std::move(o);
  }
  boost::json::array endpoints_arr;
  for (int i = offset; i < end; ++i) {
    const Hit& h = hits[i];
    boost::json::object o;
    o["name"] = h.name;
    o["kind"] = std::string(h.kind);
    emitPinOdbBare(o, h.pin, db_network);
    sta::Instance* inst = network->instance(h.pin);
    sta::LibertyCell* lc_for_emit = nullptr;
    if (inst && !network->isTopLevelPort(h.pin)) {
      o["instance"] = std::string(network->pathName(inst));
      lc_for_emit = network->libertyCell(inst);
      if (lc_for_emit) {
        o["cell"] = std::string(lc_for_emit->name());
      } else {
        o["cell"] = nullptr;
      }
    } else {
      o["instance"] = nullptr;
      o["cell"] = nullptr;
    }
    if (lc_for_emit && lc_for_emit->isClockGate()) {
      const char* flavor = "other";
      if (lc_for_emit->isClockGateLatchPosedge()) {
        flavor = "latch_posedge";
      } else if (lc_for_emit->isClockGateLatchNegedge()) {
        flavor = "latch_negedge";
      }
      o["clock_gate_flavor"] = std::string(flavor);
      auto namedPin = [&](const char* role,
                          bool (sta::LibertyPort::*pred)() const) {
        for (sta::LibertyCellPortIterator it(lc_for_emit); it.hasNext();) {
          sta::LibertyPort* lp = it.next();
          if (lp && (lp->*pred)()) {
            const sta::Pin* p = network->findPin(inst, lp->name());
            if (p) {
              o[std::string(role) + "_pin"]
                  = std::string(network->pathName(p));
              const char* tp = nullptr;
              int id = 0;
              if (resolvePinOdb(p, db_network, tp, id)) {
                o[std::string(role) + "_pin_odb_type"] = std::string(tp);
                o[std::string(role) + "_pin_odb_id"] = id;
              }
              return;
            }
          }
        }
        o[std::string(role) + "_pin"] = nullptr;
      };
      namedPin("clock_gate_ck", &sta::LibertyPort::isClockGateClock);
      namedPin("clock_gate_en", &sta::LibertyPort::isClockGateEnable);
      namedPin("clock_gate_out", &sta::LibertyPort::isClockGateOut);
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
        o["clock_gate_setup"] = nullptr;
      } else {
        o["clock_gate_setup"] = setup_v;
      }
      if (std::isnan(hold_v)) {
        o["clock_gate_hold"] = nullptr;
      } else {
        o["clock_gate_hold"] = hold_v;
      }
    }
    boost::json::array clk_arr;
    for (const auto& cn : h.clocks) {
      clk_arr.push_back(boost::json::value(cn));
    }
    o["clocks"] = std::move(clk_arr);
    endpoints_arr.push_back(std::move(o));
  }
  root["endpoints"] = std::move(endpoints_arr);
  return jsonResponse(req.id, boost::json::serialize(root));
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

  boost::json::object root;
  root["current"] = current;
  boost::json::array modes;
  for (sta::Mode* m : sta->modes()) {
    if (!m) {
      continue;
    }
    modes.push_back(boost::json::value(m->name()));
  }
  root["modes"] = std::move(modes);
  return jsonResponse(req.id, boost::json::serialize(root));
}

WebSocketResponse SdcHandler::handleSdcSetMode(const WebSocketRequest& req)
{
  sta::dbSta* sta = gen_ ? gen_->getSta() : nullptr;
  if (!sta) {
    return jsonResponse(req.id,
                        R"({"ok":false,"error":"no sta","current":""})");
  }

  const std::string target = extract_string(req.json, "mode");
  if (target.empty()) {
    return jsonResponse(
        req.id, R"({"ok":false,"error":"empty mode name","current":""})");
  }

  sta::Mode* mode = sta->findMode(target);
  if (!mode) {
    boost::json::object root;
    root["ok"] = false;
    root["error"] = std::string("mode not found: ") + target;
    std::string cur;
    if (sta::Mode* cm = sta->cmdMode()) {
      cur = cm->name();
    }
    root["current"] = cur;
    return jsonResponse(req.id, boost::json::serialize(root));
  }

  if (sta::Mode* cur = sta->cmdMode(); cur == mode) {
    boost::json::object root;
    root["ok"] = true;
    root["current"] = std::string(mode->name());
    return jsonResponse(req.id, boost::json::serialize(root));
  }

  const sta::SceneSeq& scenes = mode->scenes();
  if (scenes.empty()) {
    boost::json::object root;
    root["ok"] = false;
    root["error"] = std::string("no scene for mode: ") + target;
    std::string cur;
    if (sta::Mode* cm = sta->cmdMode()) {
      cur = cm->name();
    }
    root["current"] = cur;
    return jsonResponse(req.id, boost::json::serialize(root));
  }

  sta->setCmdScene(scenes[0]);

  boost::json::object root;
  root["ok"] = true;
  root["current"] = std::string(mode->name());
  return jsonResponse(req.id, boost::json::serialize(root));
}

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

  boost::json::object root;
  root["ok"] = error.empty();
  if (!error.empty()) {
    root["error"] = error;
  }
  root["resolved"] = before - after;
  root["remaining"] = after;
  return jsonResponse(req.id, boost::json::serialize(root));
}

// Registration entry point — called from web.cpp at session
// construction to wire every kSdc* type into the dispatcher. Each
// handler reads its own fields from req.json.
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
  d.add("sdc_endpoint_counts",
        WebSocketRequest::kSdcEndpointCounts,
        [this](const WebSocketRequest& req, SessionState&) {
          return handleSdcEndpointCounts(req);
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
