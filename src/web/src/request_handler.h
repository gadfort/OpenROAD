// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "boost/json/object.hpp"
#include "boost/json/value.hpp"
#include "boost/json/value_to.hpp"
#include "color.h"
#include "gui/gui.h"
#include "odb/db.h"
#include "odb/geom.h"
#include "tcl.h"
#include "tile_generator.h"
#include "utl/Logger.h"

namespace web {

class RequestDispatcher;
class TimingReport;
class ClockTreeReport;

// Sentinel string set as the Tcl result by WebServer::tclExitHandler
// when the browser-side Tcl `exit`/`quit` is invoked.  TclHandler
// detects this in handleTclEval and converts the response to a clean
// shutdown signal for the browser.
inline constexpr const char* kExitResultMsg = "_WEB_EXITING_";

// Thread-safe Tcl command evaluation.  Log output emitted while the
// command runs is captured by WebLogSink (registered on the logger via
// addSink) and pushed to clients as {"type":"log",...} messages — do
// NOT redirect the logger to a string here.  redirectStringBegin clears
// the entire sink list, which would unhook WebLogSink (and any other
// sink) for the duration of the command and break log streaming.  After
// each eval the optional drain_output hook is invoked so any buffered
// log output reaches clients before the eval response is sent.
struct TclEvaluator
{
  Tcl_Interp* interp;
  utl::Logger* logger;
  std::mutex mutex;
  std::function<void()> drain_output;

  struct Result
  {
    std::string result;
    bool is_error;
  };

  TclEvaluator(Tcl_Interp* interp, utl::Logger* logger)
      : interp(interp), logger(logger)
  {
  }

  Result eval(const std::string& cmd)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const int rc = Tcl_Eval(interp, cmd.c_str());
    Result r;
    r.result = Tcl_GetStringResult(interp);
    r.is_error = (rc != TCL_OK);
    if (drain_output) {
      drain_output();
    }
    return r;
  }
};

struct WebSocketRequest
{
  enum Type
  {
    kTile,
    kBounds,
    kTech,
    kSelect,
    kInspect,
    kInspectByOdb,
    kInspectBack,
    kHover,
    kTclEval,
    kTclComplete,
    kTimingReport,
    kTimingHighlight,
    kClockTree,
    kClockTreeHighlight,
    kSlackHistogram,
    kChartFilters,
    kModuleHierarchy,
    kSetModuleColors,
    kSetFocusNets,
    kSetRouteGuides,
    kHeatmaps,
    kSetActiveHeatmap,
    kSetHeatmap,
    kHeatmapTile,
    kListDir,
    kSnap,
    kSchematicCone,
    kSchematicFull,
    kSchematicInspect,
    kDrcCategories,
    kDrcMarkers,
    kDrcLoadReport,
    kDrcUpdateMarker,
    kDrcUpdateCategoryVisibility,
    kDrcHighlight,
    kSdcClocks,
    kSdcClockModes,
    kSdcPortDelays,
    kSdcExceptions,
    kSdcLimits,
    kSdcClockGroups,
    kSdcEndpoint,
    kSdcEndpointList,
    kSdcEndpointCounts,
    kSdcListModes,
    kSdcSetMode,
    kSdcResolveGenClocks,
    kCdcOverview,
    kCdcPaths,
    kCdcPathDetail,
    kCdcPinFanIn,
    kCdcClockMixTrace,
    kCdcMixEndpoints,
    kCdcSetWhitelist,
    kCdcGetWhitelist,
    kDebugContinue,
    kDebugCharts,
    kUnknown
  };

  uint32_t id = 0;
  Type type = kUnknown;
  boost::json::object json;  // parsed payload; empty on parse failure
  // Original `"type"` string from the JSON, even when not registered.
  // Used by the kUnknown error path for diagnosability.  Empty when
  // the message was malformed (parse threw) or had no `type` field.
  std::string raw_type;
  // Set to the boost::json exception message when JSON parsing or one
  // of the required envelope reads (id/type) failed.  Surfaced in the
  // kUnknown error payload so WEB-0043 names the actual parse error.
  std::string parse_error;
};

struct WebSocketResponse
{
  enum PayloadType : uint8_t
  {
    kJson = 0,
    kPng = 1,
    kError = 2
  };

  uint32_t id = 0;
  PayloadType type = kJson;
  std::vector<unsigned char> payload;
  // Original `"type"` string from the request, used by the kError
  // logging path for diagnosability.  Annotated by WebSocketSession::on_read
  // after the handler returns; handlers do not need to set it.
  std::string request_type;
};

// Shared mutable state for a WebSocket session.
// Handlers receive a reference; WebSocketSession owns the instance.
struct SessionState
{
  std::mutex selection_mutex;
  std::vector<odb::Rect> highlight_rects;
  std::vector<odb::Polygon> highlight_polys;
  std::vector<odb::Rect> hover_rects;
  std::vector<ColoredRect> timing_rects;
  std::vector<FlightLine> timing_lines;

  std::mutex selectables_mutex;
  std::vector<gui::Selected> selectables;

  gui::Selected current_inspected;
  std::vector<gui::Selected> navigation_history;

  std::mutex module_colors_mutex;
  std::map<uint32_t, Color> module_colors;  // odb module id → RGBA color

  std::mutex focus_nets_mutex;
  std::set<uint32_t> focus_net_ids;  // dbNet ODB IDs

  std::mutex route_guides_mutex;
  std::set<uint32_t> route_guide_net_ids;  // dbNet ODB IDs

  std::mutex drc_mutex;
  std::string active_drc_category;     // name of active top-level category
  std::vector<ColoredRect> drc_rects;  // filled rect shapes for overlay
  std::vector<FlightLine> drc_lines;   // line/X shapes for overlay

  std::mutex heatmap_mutex;
  std::map<std::string, std::shared_ptr<gui::HeatMapDataSource>> heatmaps;
  std::string active_heatmap;
};

// Optional-field accessor: returns the JSON value at `key` converted to T,
// or `default_val` when the key is missing.  Throws
// (boost::system::system_error) when the key is present but the JSON type
// doesn't convert to T — that's a frontend/backend contract violation, surface
// it.
//
// For required fields, prefer the bare boost::json idiom
// `obj.at(key).as_int64()` / `as_string()` / `as_bool()` / `as_double()`,
// which throws on either missing or wrong-typed input.
template <class T>
T jsonOr(const boost::json::object& obj, std::string_view key, T default_val)
{
  if (auto* v = obj.if_contains(key)) {
    return boost::json::value_to<T>(*v);
  }
  return default_val;
}

// Lenient extractors used by the SDC / CDC handlers that pre-date the
// upstream boost::json migration. They return a default-constructed
// value when the key is missing OR when the value's JSON type doesn't
// match — handlers built on these expect "field is optional, treat
// type mismatch as absent" semantics throughout. Newer call sites
// should prefer `jsonOr<T>()` (above) or the bare `.at().as_*()`
// idiom for required fields.
inline std::string extract_string(const boost::json::object& obj,
                                  std::string_view key)
{
  if (auto* v = obj.if_contains(key)) {
    if (v->is_string()) {
      return std::string(v->as_string());
    }
  }
  return {};
}

inline int extract_int(const boost::json::object& obj, std::string_view key)
{
  if (auto* v = obj.if_contains(key)) {
    if (v->is_int64()) {
      return static_cast<int>(v->as_int64());
    }
    if (v->is_double()) {
      return static_cast<int>(v->as_double());
    }
  }
  return 0;
}

inline int extract_int_or(const boost::json::object& obj,
                          std::string_view key,
                          int default_val)
{
  if (auto* v = obj.if_contains(key)) {
    if (v->is_int64()) {
      return static_cast<int>(v->as_int64());
    }
    if (v->is_double()) {
      return static_cast<int>(v->as_double());
    }
  }
  return default_val;
}

inline std::set<std::string> extract_string_array(
    const boost::json::object& obj,
    std::string_view key)
{
  std::set<std::string> result;
  if (auto* v = obj.if_contains(key)) {
    if (v->is_array()) {
      for (const auto& e : v->as_array()) {
        if (e.is_string()) {
          result.insert(std::string(e.as_string()));
        }
      }
    }
  }
  return result;
}

// Handles SELECT, INSPECT, and HOVER requests.
class SelectHandler
{
 public:
  SelectHandler(std::shared_ptr<TileGenerator> gen,
                std::shared_ptr<TclEvaluator> tcl_eval);
  void registerRequests(RequestDispatcher& dispatcher);

  WebSocketResponse handleSelect(const WebSocketRequest& req,
                                 SessionState& state);
  WebSocketResponse handleInspect(const WebSocketRequest& req,
                                  SessionState& state);
  // Variant of handleInspect for callers that already hold an ODB ref
  // (e.g. the SDC widget clicking a pin name) — bypasses the
  // selectables[] lookup since the target is identified directly by
  // {odb_type, odb_id} in the request.
  WebSocketResponse handleInspectByOdb(const WebSocketRequest& req,
                                       SessionState& state);
  WebSocketResponse handleInspectBack(const WebSocketRequest& req,
                                      SessionState& state);
  WebSocketResponse handleHover(const WebSocketRequest& req,
                                SessionState& state);
  WebSocketResponse handleSetFocusNets(const WebSocketRequest& req,
                                       SessionState& state);
  WebSocketResponse handleSetRouteGuides(const WebSocketRequest& req,
                                         SessionState& state);
  WebSocketResponse handleSnap(const WebSocketRequest& req);
  WebSocketResponse handleSchematicCone(const WebSocketRequest& req);
  WebSocketResponse handleSchematicFull(const WebSocketRequest& req);
  WebSocketResponse handleSchematicInspect(const WebSocketRequest& req,
                                           SessionState& state);

 private:
  // Shared inspect pipeline used by both handleInspect (resolves the
  // target via state.selectables[select_id]) and handleInspectByOdb
  // (resolves via {odb_type, odb_id}). Owns highlight collection,
  // navigation-history bookkeeping, JSON envelope emission, and the
  // selectables[] replacement.
  WebSocketResponse buildInspectResponse(uint32_t req_id,
                                         const gui::Selected& sel,
                                         SessionState& state);

  std::shared_ptr<TileGenerator> gen_;
  std::shared_ptr<TclEvaluator> tcl_eval_;
};

// Handles TCL_EVAL requests.
class TclHandler
{
 public:
  explicit TclHandler(std::shared_ptr<TclEvaluator> tcl_eval);
  void registerRequests(RequestDispatcher& dispatcher);

  WebSocketResponse handleTclEval(const WebSocketRequest& req);
  WebSocketResponse handleTclComplete(const WebSocketRequest& req);

 private:
  std::shared_ptr<TclEvaluator> tcl_eval_;
};

// Handles TIMING_REPORT and TIMING_HIGHLIGHT requests.
class TimingHandler
{
 public:
  TimingHandler(std::shared_ptr<TileGenerator> gen,
                std::shared_ptr<TimingReport> timing_report,
                std::shared_ptr<TclEvaluator> tcl_eval);
  void registerRequests(RequestDispatcher& dispatcher);

  WebSocketResponse handleTimingReport(const WebSocketRequest& req);
  WebSocketResponse handleTimingHighlight(const WebSocketRequest& req,
                                          SessionState& state);
  WebSocketResponse handleSlackHistogram(const WebSocketRequest& req);
  WebSocketResponse handleChartFilters(const WebSocketRequest& req);

 private:
  std::shared_ptr<TileGenerator> gen_;
  std::shared_ptr<TimingReport> timing_report_;
  std::shared_ptr<TclEvaluator> tcl_eval_;
};

// Handles CLOCK_TREE and CLOCK_TREE_HIGHLIGHT requests.
class ClockTreeHandler
{
 public:
  ClockTreeHandler(std::shared_ptr<TileGenerator> gen,
                   std::shared_ptr<ClockTreeReport> clock_report,
                   std::shared_ptr<TclEvaluator> tcl_eval);
  void registerRequests(RequestDispatcher& dispatcher);

  WebSocketResponse handleClockTree(const WebSocketRequest& req);
  WebSocketResponse handleClockTreeHighlight(const WebSocketRequest& req,
                                             SessionState& state);

 private:
  std::shared_ptr<TileGenerator> gen_;
  std::shared_ptr<ClockTreeReport> clock_report_;
  std::shared_ptr<TclEvaluator> tcl_eval_;
};

// Handles TILE/BOUNDS/TECH requests.
class TileHandler
{
 public:
  explicit TileHandler(std::shared_ptr<TileGenerator> gen);
  void registerRequests(RequestDispatcher& dispatcher);

  void initializeHeatMaps(SessionState& state);
  WebSocketResponse handleTile(const WebSocketRequest& req,
                               SessionState& state);
  WebSocketResponse handleModuleHierarchy(const WebSocketRequest& req);
  WebSocketResponse handleSetModuleColors(const WebSocketRequest& req,
                                          SessionState& state);
  WebSocketResponse handleHeatMaps(const WebSocketRequest& req,
                                   SessionState& state);
  WebSocketResponse handleSetActiveHeatMap(const WebSocketRequest& req,
                                           SessionState& state);
  WebSocketResponse handleSetHeatMap(const WebSocketRequest& req,
                                     SessionState& state);
  WebSocketResponse handleHeatMapTile(const WebSocketRequest& req,
                                      SessionState& state);

 private:
  static WebSocketResponse serializeBounds(uint32_t id,
                                           const TileGenerator& gen);
  static WebSocketResponse serializeTech(uint32_t id, const TileGenerator& gen);
  static WebSocketResponse renderTile(
      uint32_t id,
      const std::string& layer,
      int z,
      int x,
      int y,
      const TileVisibility& vis,
      const TileGenerator& gen,
      const std::vector<odb::Rect>& highlight_rects,
      const std::vector<odb::Polygon>& highlight_polys,
      const std::vector<ColoredRect>& colored_rects,
      const std::vector<FlightLine>& flight_lines,
      const std::map<uint32_t, Color>* module_colors,
      const std::set<uint32_t>* focus_net_ids,
      const std::set<uint32_t>* route_guide_net_ids);

  std::shared_ptr<TileGenerator> gen_;
};

// Handles DRC_CATEGORIES, DRC_MARKERS, DRC_LOAD_REPORT,
// DRC_UPDATE_MARKER, and DRC_HIGHLIGHT requests.
class DRCHandler
{
 public:
  explicit DRCHandler(std::shared_ptr<TileGenerator> gen);
  void registerRequests(RequestDispatcher& dispatcher);

  WebSocketResponse handleDRCCategories(const WebSocketRequest& req);
  WebSocketResponse handleDRCMarkers(const WebSocketRequest& req,
                                     SessionState& state);
  WebSocketResponse handleDRCLoadReport(const WebSocketRequest& req,
                                        SessionState& state);
  WebSocketResponse handleDRCUpdateMarker(const WebSocketRequest& req,
                                          SessionState& state);
  WebSocketResponse handleDRCUpdateCategoryVisibility(
      const WebSocketRequest& req,
      SessionState& state);
  WebSocketResponse handleDRCHighlight(const WebSocketRequest& req,
                                       SessionState& state);

 private:
  std::shared_ptr<TileGenerator> gen_;
  int min_box_ = -1;  // cached tech pitch for marker rendering threshold

  // Returns block and chip, throwing if either is null.
  std::pair<odb::dbBlock*, odb::dbChip*> getBlockAndChip();

  // Find a marker by ID in the active category. Returns nullptr if not found.
  odb::dbMarker* findMarkerById(SessionState& state,
                                odb::dbChip* chip,
                                int marker_id);

  // Recompute DRC overlay rects from the active category's visible markers.
  void refreshDRCOverlay(SessionState& state);
};

// Handles LIST_DIR requests (server-side file browsing).
WebSocketResponse handleListDir(const WebSocketRequest& req);

// Handles every kSdc* request — clocks, clock modes, port delays,
// exceptions, limits, clock groups, endpoint browse + detail, mode
// switching, and generated-clock resolution. Each is registered with
// the dispatcher in registerRequests().
class SdcHandler
{
 public:
  explicit SdcHandler(std::shared_ptr<TileGenerator> gen);
  void registerRequests(RequestDispatcher& dispatcher);

  WebSocketResponse handleSdcClocks(const WebSocketRequest& req);
  WebSocketResponse handleSdcClockModes(const WebSocketRequest& req);
  WebSocketResponse handleSdcPortDelays(const WebSocketRequest& req);
  WebSocketResponse handleSdcExceptions(const WebSocketRequest& req);
  WebSocketResponse handleSdcLimits(const WebSocketRequest& req);
  WebSocketResponse handleSdcClockGroups(const WebSocketRequest& req);
  WebSocketResponse handleSdcEndpoint(const WebSocketRequest& req);
  WebSocketResponse handleSdcEndpointList(const WebSocketRequest& req);
  WebSocketResponse handleSdcEndpointCounts(const WebSocketRequest& req);
  WebSocketResponse handleSdcListModes(const WebSocketRequest& req);
  WebSocketResponse handleSdcSetMode(const WebSocketRequest& req);
  WebSocketResponse handleSdcResolveGenClocks(const WebSocketRequest& req);

 private:
  std::shared_ptr<TileGenerator> gen_;
};

// Handles every kCdc* request — the cross-domain crossing visualizer
// tab. Owns the session-scoped synchroniser whitelist (instance and
// master pattern lists) so re-classification doesn't need to round-trip
// through SDC. Each request type is registered with the dispatcher in
// registerRequests().
class CdcHandler
{
 public:
  CdcHandler(std::shared_ptr<TileGenerator> gen);
  ~CdcHandler();
  void registerRequests(RequestDispatcher& dispatcher);

  WebSocketResponse handleCdcOverview(const WebSocketRequest& req);
  WebSocketResponse handleCdcPaths(const WebSocketRequest& req);
  WebSocketResponse handleCdcPathDetail(const WebSocketRequest& req);
  WebSocketResponse handleCdcPinFanIn(const WebSocketRequest& req);
  WebSocketResponse handleCdcClockMixTrace(const WebSocketRequest& req);
  WebSocketResponse handleCdcMixEndpoints(const WebSocketRequest& req);
  WebSocketResponse handleCdcSetWhitelist(const WebSocketRequest& req);
  WebSocketResponse handleCdcGetWhitelist(const WebSocketRequest& req);

 private:
  std::shared_ptr<TileGenerator> gen_;

  // pImpl-style cache of the per-mode CdcPair lists. The structure is
  // defined inside cdc_handler.cpp because CdcPair carries STA pointer
  // types we don't want to leak into this header. cdc_overview
  // populates it for every mode in one walk; cdc_paths reads from it
  // on every per-cell drill-in (so paginating + filtering by category
  // is just an O(N) scan of an in-memory vector instead of a fresh
  // endpoint walk per click). cdc_set_whitelist invalidates it.
  struct PairCache;
  std::unique_ptr<PairCache> pair_cache_;

  // Session-scoped synchroniser whitelist. Two glob-pattern lists,
  // each independently maintained:
  //   instance_patterns_ — matched against network->pathName(capture_inst)
  //   master_patterns_   — matched against LibertyCell::name() of the
  //                        capture flop's master cell
  // A path is tagged "whitelisted" when *either* list matches. Empty
  // by default; populated via cdc_set_whitelist; never persisted.
  std::mutex whitelist_mutex_;
  std::vector<std::string> instance_patterns_;
  std::vector<std::string> master_patterns_;
};

}  // namespace web
