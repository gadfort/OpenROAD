// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

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

// Thread-safe Tcl command evaluation with output capture.
struct TclEvaluator
{
  Tcl_Interp* interp;
  utl::Logger* logger;
  std::mutex mutex;

  struct Result
  {
    std::string output;
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
    logger->redirectStringBegin();
    const int rc = Tcl_Eval(interp, cmd.c_str());
    Result r;
    r.output = logger->redirectStringEnd();
    r.result = Tcl_GetStringResult(interp);
    r.is_error = (rc != TCL_OK);
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
    kSdcListModes,
    kSdcSetMode,
    kSdcResolveGenClocks,
    kCdcOverview,
    kCdcPaths,
    kCdcPathDetail,
    kCdcPinFanIn,
    kCdcSetWhitelist,
    kCdcGetWhitelist,
    kDebugContinue,
    kDebugCharts,
    kUnknown
  };

  uint32_t id = 0;
  Type type = kUnknown;
  std::string raw_json;  // original JSON message for field extraction
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

// Minimal JSON field extraction (no JSON library dependency).
std::string extract_string(const std::string& json, const std::string& key);
int extract_int(const std::string& json, const std::string& key);
int extract_int_or(const std::string& json,
                   const std::string& key,
                   int default_val);
float extract_float_or(const std::string& json,
                       const std::string& key,
                       float default_val);
std::set<std::string> extract_string_array(const std::string& json,
                                           const std::string& key);

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
