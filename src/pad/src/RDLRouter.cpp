/////////////////////////////////////////////////////////////////////////////
//
// BSD 3-Clause License
//
// Copyright (c) 2023, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include "RDLRouter.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/polygon/polygon.hpp>
#include <limits>
#include <list>
#include <queue>
#include <set>

#include "Utilities.h"
#include "odb/db.h"
#include "odb/dbTransform.h"
#include "pad/ICeWall.h"
#include "utl/Logger.h"

namespace pad {

class RDLRouterDistanceHeuristic
    : public boost::astar_heuristic<RDLRouter::GridGraph, int64_t>
{
 public:
  RDLRouterDistanceHeuristic(
      const std::map<RDLRouter::grid_vertex, odb::Point>& vertex_map,
      const std::vector<RDLRouter::grid_vertex>& predecessor,
      const RDLRouter::grid_vertex& start_vertex,
      const odb::Point& goal,
      float turn_penalty)
      : vertex_map_(vertex_map),
        predecessor_(predecessor),
        start_vertex_(start_vertex),
        goal_(goal),
        turn_penalty_(turn_penalty)
  {
  }
  int64_t operator()(RDLRouter::grid_vertex vt_next)
  {
    const auto& pt_next = vertex_map_.at(vt_next);

    const int64_t distance = RDLRouter::distance(goal_, pt_next);

    const auto& vt_curr = predecessor_[vt_next];
    if (start_vertex_ == vt_curr) {
      return distance;
    }

    const auto& vt_prev = predecessor_[vt_curr];
    if (start_vertex_ == vt_prev) {
      return distance;
    }

    const auto& pt_curr = vertex_map_.at(vt_curr);
    const auto& pt_prev = vertex_map_.at(vt_prev);

    const odb::Point incoming_vec(pt_curr.x() - pt_prev.x(),
                                  pt_curr.y() - pt_prev.y());
    const odb::Point outgoing_vec(pt_next.x() - pt_curr.x(),
                                  pt_next.y() - pt_curr.y());

    int64_t penalty = 0;
    if (incoming_vec != outgoing_vec) {
      penalty = turn_penalty_ * RDLRouter::distance(pt_prev, pt_curr);
    }

    return distance + penalty;
  }

 private:
  const std::map<RDLRouter::grid_vertex, odb::Point>& vertex_map_;
  const std::vector<RDLRouter::grid_vertex>& predecessor_;
  const RDLRouter::grid_vertex& start_vertex_;
  odb::Point goal_;
  const float turn_penalty_;
};

struct RDLRouterGoalFound
{
};  // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class RDLRouterGoalVisitor : public boost::default_astar_visitor
{
 public:
  explicit RDLRouterGoalVisitor(RDLRouter::grid_vertex goal) : goal_(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g)
  {
    if (u == goal_) {
      throw RDLRouterGoalFound();
    }
  }

 private:
  RDLRouter::grid_vertex goal_;
};

RDLRouter::RDLRouter(utl::Logger* logger,
                     odb::dbBlock* block,
                     odb::dbTechLayer* layer,
                     odb::dbTechVia* bump_via,
                     odb::dbTechVia* pad_via,
                     const std::map<odb::dbITerm*, odb::dbITerm*>& routing_map,
                     int width,
                     int spacing,
                     bool allow45,
                     float turn_penalty)
    : logger_(logger),
      block_(block),
      layer_(layer),
      bump_accessvia_(bump_via),
      pad_accessvia_(pad_via),
      width_(width),
      spacing_(spacing),
      allow45_(allow45),
      turn_penalty_(turn_penalty),
      routing_map_(routing_map),
      gui_(nullptr)
{
  if (width_ == 0) {
    width_ = layer_->getWidth();
  }
  if (width_ < layer_->getWidth()) {
    const double dbus = block_->getDbUnitsPerMicron();
    logger_->warn(
        utl::PAD,
        3,
        "{:.3f}um is below the minimum width for {}, changing to {:.3f}um",
        width_ / dbus,
        layer_->getName(),
        layer_->getWidth() / dbus);
    width_ = layer_->getWidth();
  }
  if (spacing_ == 0) {
    spacing_ = layer_->getSpacing(width_);
  }
  if (spacing_ < layer_->getSpacing(width_)) {
    const double dbus = block_->getDbUnitsPerMicron();
    logger_->warn(
        utl::PAD,
        4,
        "{:.3f}um is below the minimum spacing for {}, changing to {:.3f}um",
        spacing_ / dbus,
        layer_->getName(),
        layer_->getSpacing(width_) / dbus);
    spacing_ = layer_->getSpacing(width_);
  }
}

RDLRouter::~RDLRouter()
{
  if (gui_ != nullptr) {
    gui_->setRouter(nullptr);
  }
}

void RDLRouter::route(const std::vector<odb::dbNet*>& nets)
{
  const double dbus = block_->getDbUnitsPerMicron();
  // Build list of routing targets
  for (auto* net : nets) {
    routing_targets_[net] = generateRoutingTargets(net);
  }

  // Build obstructions
  populateObstructions(nets);

  // build graph
  makeGraph();

  if (gui_ != nullptr) {
    gui_->pause();
  }

  std::vector<odb::dbITerm*> cover_terms;
  for (const auto& [net, iterm_targets] : routing_targets_) {
    for (const auto& [iterm, targets] : iterm_targets) {
      if (!isCoverTerm(iterm)) {
        // only collect cover terms
        continue;
      }

      std::vector<odb::dbITerm*> iterm_pairs;
      auto assigned_route = routing_map_.find(iterm);
      if (assigned_route != routing_map_.end()) {
        if (assigned_route->second != nullptr) {
          iterm_pairs.push_back(assigned_route->second);
        }
      } else {
        for (const auto& [piterm, targets] : iterm_targets) {
          if (iterm->getInst() != piterm->getInst()) {
            iterm_pairs.push_back(piterm);
          }
        }
      }
      if (!iterm_pairs.empty()) {
        cover_terms.push_back(iterm);
      }

      const odb::Point iterm_center = iterm->getBBox().center();
      // shortest distance to longest
      std::stable_sort(
          iterm_pairs.begin(),
          iterm_pairs.end(),
          [this, &iterm_center](odb::dbITerm* lhs, odb::dbITerm* rhs) {
            const bool lhs_cover = isCoverTerm(lhs);
            const bool rhs_cover = isCoverTerm(rhs);
            // sort non-cover terms first
            if (lhs_cover == rhs_cover) {
              return distance(iterm_center, lhs->getBBox().center())
                     < distance(iterm_center, rhs->getBBox().center());
            }
            if (rhs_cover) {
              return true;
            }
            return false;
          });

      auto& iterm_info = routing_iterm_map_[iterm];
      iterm_info.terminals = iterm_pairs;
      iterm_info.next = iterm_info.terminals.begin();
    }
  }

  // create priority queue
  auto iterm_cmp = [this](odb::dbITerm* lhs, odb::dbITerm* rhs) {
    const auto& lhs_shortest = *routing_iterm_map_[lhs].next;
    const auto& rhs_shortest = *routing_iterm_map_[rhs].next;
    const auto lhs_dist
        = distance(lhs->getBBox().center(), lhs_shortest->getBBox().center());
    const auto rhs_dist
        = distance(rhs->getBBox().center(), rhs_shortest->getBBox().center());

    return lhs_dist > rhs_dist;
  };

  std::priority_queue route_queue(
      cover_terms.begin(), cover_terms.end(), iterm_cmp);

  logger_->info(utl::PAD, 5, "Routing {} nets", nets.size());

  // track destinations routed to, so we don't attempt to route to the same
  // iterm more than once.
  std::set<odb::dbITerm*> dst_items_routed;
  // track sets of routes, so we don't route the reverse by accident
  std::map<odb::dbITerm*, odb::dbITerm*> routed_pairs;
  // track cover instances we dont route the same one twice
  std::set<odb::dbInst*> routed_covers;
  while (!route_queue.empty()) {
    odb::dbITerm* src = route_queue.top();
    route_queue.pop();

    odb::dbNet* net = src->getNet();
    auto& net_targets = routing_targets_[net];

    auto& status = routing_iterm_map_[src];

    if (routed_covers.find(src->getInst()) != routed_covers.end()) {
      // we've already routed this cover once (indicates the cover has multiple
      // iterms)
      routed_pairs[src] = nullptr;
      status.next = status.terminals.end();
      continue;
    }

    odb::dbITerm* dst = nullptr;
    do {
      if (status.next == status.terminals.end()) {
        dst = nullptr;
        break;
      }
      dst = *status.next;
      status.next++;
    } while (dst_items_routed.find(dst) != dst_items_routed.end()
             || routed_pairs[src] == dst);

    if (dst == nullptr) {
      failed_.insert(src);
      continue;
    }

    debugPrint(
        logger_,
        utl::PAD,
        "Router",
        2,
        "Routing {} -> {} : ({:.3f}um)",
        src->getName(),
        dst->getName(),
        distance(src->getBBox().center(), dst->getBBox().center()) / dbus);

    // create ordered set of iterm targets
    std::vector<TargetPair> targets;
    for (const auto& src_target : net_targets[src]) {
      for (const auto& dst_target : net_targets[dst]) {
        targets.push_back(TargetPair{&src_target, &dst_target});
      }
    }

    std::stable_sort(
        targets.begin(), targets.end(), [](const auto& lhs, const auto& rhs) {
          return distance(lhs) < distance(rhs);
        });

    bool routed = false;
    for (auto& points : targets) {
      debugPrint(logger_,
                 utl::PAD,
                 "Router",
                 3,
                 "Routing {} ({:.3f}um, {:.3f}um) -> ({:.3f}um, {:.3f}um) : "
                 "({:.3f}um)",
                 net->getName(),
                 points.target0->center.x() / dbus,
                 points.target0->center.y() / dbus,
                 points.target1->center.x() / dbus,
                 points.target1->center.y() / dbus,
                 distance(points) / dbus);
    }

    for (auto& points : targets) {
      const auto added_edges0
          = insertTerminalVertex(*points.target0, *points.target1);
      const auto added_edges1
          = insertTerminalVertex(*points.target1, *points.target0);

      auto route = run(points.target0->center, points.target1->center);

      if (!route.empty()) {
        debugPrint(
            logger_, utl::PAD, "Router", 3, "Route segments {}", route.size());
        routes_[net].push_back({route, points.target0, points.target1});
        commitRoute(route);

        // house keeping
        routed = true;

        // record destination to avoid picking it again as a destination
        dst_items_routed.insert(dst);

        // record cover instance
        routed_covers.insert(src->getInst());

        // record routed pair (forward and reverse) to avoid routing this segment again
        routed_pairs[src] = dst;
        routed_pairs[dst] = src;

        // delete previously mark failures if they share an instance
        failed_.erase(std::find_if(failed_.begin(),
                                   failed_.end(),
                                   [src](odb::dbITerm* term) {
                                     return src->getInst() == term->getInst();
                                   }),
                      failed_.end());
      }

      removeTerminalEdges(added_edges0);
      removeTerminalEdges(added_edges1);

      if (routed) {
        break;
      }
    }

    if (!routed) {
      if (status.next != status.terminals.end()) {
        route_queue.push(src);
      } else {
        failed_.insert(src);
      }
    }

    if (gui_ != nullptr && logger_->debugCheck(utl::PAD, "Router", 2)) {
      gui_->pause();
    }
  }

  std::map<odb::dbNet*, std::set<odb::dbITerm*>> failed;
  for (auto* fail : failed_) {
    failed[fail->getNet()].insert(fail);
  }
  if (!failed.empty()) {
    logger_->warn(
        utl::PAD, 6, "Failed to route the following {} nets:", failed.size());
    for (const auto& [net, iterms] : failed) {
      logger_->report("  {}", net->getName());
      for (auto* iterm : iterms) {
        const auto& route_info = routing_iterm_map_[iterm];
        std::vector<odb::dbITerm*> no_routes_to;
        for (auto* dst_iterm : route_info.terminals) {
          if (routed_pairs[iterm] == dst_iterm) {
            continue;
          }
          no_routes_to.push_back(dst_iterm);
        }

        const size_t max_print_length = 5;
        std::string terms = "";
        for (size_t i = 0; i < no_routes_to.size() && i < max_print_length;
             i++) {
          if (!terms.empty()) {
            terms += ", ";
          }
          terms += no_routes_to[i]->getName();
        }
        if (no_routes_to.size() < max_print_length) {
          logger_->report("    {} -> {}", iterm->getName(), terms);
        } else {
          logger_->report("    {} -> {}, ... ({} possible terminals)",
                          iterm->getName(),
                          terms,
                          no_routes_to.size());
        }
      }
    }
  }

  // smooth wire
  // write to DB
  for (const auto& [net, net_routes] : routes_) {
    net->destroySWires();
    for (const auto& [route, source, target] : net_routes) {
      writeToDb(net, route, *source, *target);
    }
  }

  if (!failed_.empty()) {
    logger_->error(utl::PAD, 7, "Failed to route {} nets.", failed_.size());
  }
}

void RDLRouter::removeTerminalEdges(const std::vector<Edge>& edges)
{
  for (const auto& [p0, p1] : edges) {
    boost::remove_edge(point_vertex_map_[p0], point_vertex_map_[p1], graph_);
  }
}

std::vector<RDLRouter::Edge> RDLRouter::insertTerminalVertex(
    const RouteTarget& target,
    const RouteTarget& source)
{
  struct GridSnap
  {
    int pos;
    int index;
  };

  const double dbus = block_->getDbUnitsPerMicron();

  auto snap = [](const int pos, const std::vector<int>& grid) -> GridSnap {
    int dist = std::numeric_limits<int>::max();
    for (size_t i = 0; i < grid.size(); i++) {
      const int p = grid[i];
      const int new_dist = std::abs(p - pos);
      if (new_dist < dist) {
        dist = new_dist;
      } else {
        return {grid[i - 1], static_cast<int>(i - 1)};
      }
    }

    return {-1, -1};
  };

  const GridSnap x_snap = snap(target.center.x(), x_grid_);
  const GridSnap y_snap = snap(target.center.y(), y_grid_);

  const odb::Point snapped(x_snap.pos, y_snap.pos);

  debugPrint(logger_,
             utl::PAD,
             "Router_snap",
             1,
             "Snap ({}, {}) -> ({}, {})",
             target.center.x(),
             target.center.y(),
             snapped.x(),
             snapped.y());

  if (x_snap.index == -1 || y_snap.index == -1) {
    logger_->error(utl::PAD,
                   8,
                   "Unable to snap ({:.3f}um, {:.3f}um) to routing grid.",
                   target.center.x() / dbus,
                   target.center.y() / dbus);
  }

  addGraphVertex(snapped);

  odb::Rect iterm_box;
  target.shape.bloat(getBloatFactor(), iterm_box);

  const float route_dist = distance(target.center, source.center);

  auto add_snap_edge
      = [this, &source, &route_dist, &snapped](const std::vector<int>& grid,
                                               const int index,
                                               const int const_pos,
                                               const int dindex,
                                               const bool is_x,
                                               const int term_boundary,
                                               odb::Point& edge_snap) -> bool {
    int idx = index + dindex;
    bool found = false;
    while (0 <= idx && idx < grid.size()) {
      const int grid_pt = grid[idx];
      bool inside_terminal;
      if (dindex > 0) {
        inside_terminal = grid_pt <= term_boundary;
      } else {
        inside_terminal = term_boundary <= grid_pt;
      }

      if (!inside_terminal) {
        odb::Point pt;
        if (is_x) {
          pt = odb::Point(grid_pt, const_pos);
        } else {
          pt = odb::Point(const_pos, grid_pt);
        }

        auto find_vertex = point_vertex_map_.find(pt);
        if (find_vertex != point_vertex_map_.end()) {
          edge_snap = pt;
          found = true;
          break;
        }
      }
      idx += dindex;
    }

    debugPrint(logger_,
               utl::PAD,
               "Router_snap",
               2,
               "Finding snap point ({}, {}) with must be outside {} and is "
               "searching in {} direction by {} and start was {}.",
               snapped.x(),
               snapped.y(),
               term_boundary,
               is_x ? "x" : "y",
               dindex,
               grid[index]);

    if (!found) {
      return false;
    }

    const float weight_scale = std::min(distance(edge_snap, source.center),
                                        distance(snapped, source.center))
                               / route_dist;

    debugPrint(logger_,
               utl::PAD,
               "Router_snap",
               2,
               "Adding edge ({}, {}) -> ({}, {}) with scale {}",
               snapped.x(),
               snapped.y(),
               edge_snap.x(),
               edge_snap.y(),
               weight_scale);

    return addGraphEdge(snapped, edge_snap, weight_scale, false);
  };
  std::set<odb::Point> edge_points;
  odb::Point edge_pt;
  if (add_snap_edge(x_grid_,
                    x_snap.index,
                    snapped.y(),
                    -1,
                    true,
                    iterm_box.xMin(),
                    edge_pt)) {
    edge_points.insert(edge_pt);
  }
  if (add_snap_edge(x_grid_,
                    x_snap.index,
                    snapped.y(),
                    1,
                    true,
                    iterm_box.xMax(),
                    edge_pt)) {
    edge_points.insert(edge_pt);
  }
  if (add_snap_edge(y_grid_,
                    y_snap.index,
                    snapped.x(),
                    -1,
                    false,
                    iterm_box.yMin(),
                    edge_pt)) {
    edge_points.insert(edge_pt);
  }
  if (add_snap_edge(y_grid_,
                    y_snap.index,
                    snapped.x(),
                    1,
                    false,
                    iterm_box.yMax(),
                    edge_pt)) {
    edge_points.insert(edge_pt);
  }

  if (edge_points.empty()) {
    logger_->error(
        utl::PAD,
        9,
        "No edges added to routing grid to access ({:.3f}um, {:.3f}um).",
        target.center.x() / dbus,
        target.center.y() / dbus);
  }

  const odb::Point& target_pt = target.center;
  const auto& snap_v = point_vertex_map_[snapped];

  std::vector<Edge> added_edges;

  auto get_weight = [&source, &route_dist](const odb::Point& p0,
                                           const odb::Point& p1) -> float {
    return std::min(distance(p0, source.center), distance(p1, source.center))
           / route_dist;
  };

  addGraphVertex(target_pt);
  // Add edges to hit center
  for (const auto& pt : edge_points) {
    added_edges.push_back({pt, snapped});
    const odb::Rect edge_shape(pt, snapped);
    if (edge_shape.xMin() <= target_pt.x()
        && target_pt.x() <= edge_shape.xMax()) {
      // Remove horizontal edge
      const auto& vh = point_vertex_map_[pt];
      boost::remove_edge(snap_v, vh, graph_);
      // Add middle point vertex
      const odb::Point new_pt(target_pt.x(), pt.y());
      addGraphVertex(new_pt);
      // Add two new horizontal edges
      if (addGraphEdge(pt, new_pt, get_weight(pt, new_pt), false)) {
        added_edges.push_back({pt, new_pt});
      }
      if (addGraphEdge(new_pt, snapped, get_weight(new_pt, snapped), false)) {
        added_edges.push_back({new_pt, snapped});
      }
      // Add edge to target
      if (addGraphEdge(
              new_pt, target_pt, get_weight(new_pt, target_pt), false)) {
        added_edges.push_back({new_pt, target_pt});
      }
    } else if (edge_shape.yMin() <= target_pt.y()
               && target_pt.y() <= edge_shape.yMax()) {
      // Remove vertical edge
      const auto& vv = point_vertex_map_[pt];
      boost::remove_edge(snap_v, vv, graph_);
      // Add middle point vertex
      const odb::Point new_pt(pt.x(), target_pt.y());
      addGraphVertex(new_pt);
      // Add two new vertical edges
      if (addGraphEdge(pt, new_pt, get_weight(pt, new_pt), false)) {
        added_edges.push_back({pt, new_pt});
      }
      if (addGraphEdge(new_pt, snapped, get_weight(new_pt, snapped), false)) {
        added_edges.push_back({new_pt, snapped});
      }
      // Add edge to target
      if (addGraphEdge(
              new_pt, target_pt, get_weight(new_pt, target_pt), false)) {
        added_edges.push_back({new_pt, target_pt});
      }
    }
  }

  return added_edges;
}

void RDLRouter::uncommitRoute(
    const std::set<std::pair<odb::Point, odb::Point>>& route)
{
  for (const auto& [p0, p1] : route) {
    addGraphEdge(p0, p1);
  }
}

std::set<std::pair<odb::Point, odb::Point>> RDLRouter::commitRoute(
    const std::vector<grid_vertex>& route)
{
  std::set<grid_edge> edges;
  for (const auto& v : route) {
    GridGraph::out_edge_iterator oit, oend;
    std::tie(oit, oend) = boost::out_edges(v, graph_);
    for (; oit != oend; oit++) {
      edges.insert(*oit);
    }
    GridGraph::in_edge_iterator iit, iend;
    std::tie(iit, iend) = boost::in_edges(v, graph_);
    for (; iit != iend; iit++) {
      edges.insert(*iit);
    }
  }

  std::set<std::pair<odb::Point, odb::Point>> removed_edges;
  for (const auto& edge : edges) {
    removed_edges.emplace(vertex_point_map_[edge.m_source],
                          vertex_point_map_[edge.m_target]);
    boost::remove_edge(edge, graph_);
  }
  return removed_edges;
}

std::vector<RDLRouter::grid_vertex> RDLRouter::run(const odb::Point& source,
                                                   const odb::Point& dest)
{
  const int N = boost::num_vertices(graph_);
  std::vector<grid_vertex> p(N);
  std::vector<int64_t> d(N);

  const grid_vertex& start = point_vertex_map_[source];
  const grid_vertex& goal = point_vertex_map_[dest];

  debugPrint(logger_,
             utl::PAD,
             "Router",
             1,
             "Route ({}, {}) -> ({}, {})",
             source.x(),
             source.y(),
             dest.x(),
             dest.y());

  try {
    // call astar named parameter interface
    boost::astar_search_tree(
        graph_,
        start,
        RDLRouterDistanceHeuristic(
            vertex_point_map_, p, start, dest, turn_penalty_),
        boost::predecessor_map(
            boost::make_iterator_property_map(
                p.begin(), boost::get(boost::vertex_index, graph_)))
            .distance_map(boost::make_iterator_property_map(
                d.begin(), boost::get(boost::vertex_index, graph_)))
            .visitor(RDLRouterGoalVisitor<grid_vertex>(goal)));
  } catch (const RDLRouterGoalFound&) {  // found a path to the goal
    std::list<grid_vertex> shortest_path;
    for (grid_vertex v = goal;; v = p[v]) {
      shortest_path.push_front(v);
      if (p[v] == v) {
        break;
      }
    }

    std::vector<grid_vertex> route(shortest_path.begin(), shortest_path.end());
    return route;
  }

  return {};
}

void RDLRouter::makeGraph()
{
  point_vertex_map_.clear();
  vertex_point_map_.clear();
  graph_.clear();

  graph_weight_ = boost::get(boost::edge_weight, graph_);

  std::vector<int> x_grid;
  std::vector<int> y_grid;

  odb::dbTrackGrid* tracks = block_->findTrackGrid(layer_);
  tracks->getGridX(x_grid);
  tracks->getGridY(y_grid);

  // filter grid points based on spacing requirements
  const int pitch = width_ + spacing_ - 1;
  x_grid_.clear();
  for (const auto& x : x_grid) {
    bool add = false;
    if (x_grid_.empty()) {
      add = true;
    } else {
      if (*x_grid_.rbegin() + pitch < x) {
        add = true;
      }
    }

    if (add) {
      x_grid_.push_back(x);
    }
  }
  y_grid_.clear();
  for (const auto& y : y_grid) {
    bool add = false;
    if (y_grid_.empty()) {
      add = true;
    } else {
      if (*y_grid_.rbegin() + pitch < y) {
        add = true;
      }
    }

    if (add) {
      y_grid_.push_back(y);
    }
  }

  for (const auto& x : x_grid_) {
    for (const auto& y : y_grid_) {
      addGraphVertex(odb::Point(x, y));
    }
  }
  debugPrint(logger_,
             utl::PAD,
             "Router",
             1,
             "Added {} vertices to graph",
             boost::num_vertices(graph_));

  for (size_t i = 0; i < x_grid_.size(); i++) {
    for (size_t j = 0; j < y_grid_.size(); j++) {
      const odb::Point center(x_grid_[i], y_grid_[j]);

      if (j + 1 < y_grid_.size()) {
        addGraphEdge(center, {x_grid_[i], y_grid_[j + 1]});
      }
      if (j != 0) {
        addGraphEdge(center, {x_grid_[i], y_grid_[j - 1]});
      }
      if (i != 0) {
        addGraphEdge(center, {x_grid_[i - 1], y_grid_[j]});
      }
      if (i + 1 < x_grid_.size()) {
        addGraphEdge(center, {x_grid_[i + 1], y_grid_[j]});
      }

      if (allow45_) {
        if (i % 2 == 1 || j % 2 == 1) {
          // only do every other position
          continue;
        }
        if (i + 1 < x_grid_.size() && j + 1 < y_grid_.size()) {
          addGraphEdge(center, {x_grid_[i + 1], y_grid_[j + 1]});
        }
        if (i + 1 < x_grid_.size() && j != 0) {
          addGraphEdge(center, {x_grid_[i + 1], y_grid_[j - 1]});
        }
        if (i != 0 && j + 1 < y_grid_.size()) {
          addGraphEdge(center, {x_grid_[i - 1], y_grid_[j + 1]});
        }
        if (i != 0 && j != 0) {
          addGraphEdge(center, {x_grid_[i - 1], y_grid_[j - 1]});
        }
      }
    }
  }

  debugPrint(logger_,
             utl::PAD,
             "Router",
             1,
             "Added {} edges to graph",
             boost::num_edges(graph_));
}

bool RDLRouter::isObstructed(const odb::Point& pt) const
{
  return obstructions_.qbegin(
             boost::geometry::index::intersects(Point(pt.x(), pt.y())))
         != obstructions_.qend();
}

void RDLRouter::addGraphVertex(const odb::Point& point)
{
  auto idx = boost::add_vertex(graph_);
  debugPrint(logger_,
             utl::PAD,
             "Router_vertex",
             1,
             "Adding point ({}, {}) as vertex {}",
             point.x(),
             point.y(),
             idx);
  point_vertex_map_[point] = idx;
  vertex_point_map_[idx] = point;
}

bool RDLRouter::addGraphEdge(const odb::Point& point0,
                             const odb::Point& point1,
                             float edge_weight_scale,
                             bool check_obstructions)
{
  auto point0check = point_vertex_map_.find(point0);
  if (point0check == point_vertex_map_.end()) {
    debugPrint(logger_,
               utl::PAD,
               "Router_edge",
               1,
               "Failed to find vertex at ({}, {})",
               point0.x(),
               point0.y());
    return false;
  }
  auto point1check = point_vertex_map_.find(point1);
  if (point1check == point_vertex_map_.end()) {
    debugPrint(logger_,
               utl::PAD,
               "Router_edge",
               1,
               "Failed to find vertex at ({}, {})",
               point1.x(),
               point1.y());
    return false;
  }
  grid_vertex v0 = point0check->second;
  grid_vertex v1 = point1check->second;
  if (v0 == v1) {
    return false;
  }

  using Line = boost::geometry::model::segment<Point>;
  if (check_obstructions
      && obstructions_.qbegin(boost::geometry::index::intersects(Line(
             Point(point0.x(), point0.y()), Point(point1.x(), point1.y()))))
             != obstructions_.qend()) {
    debugPrint(logger_,
               utl::PAD,
               "Router_edge",
               1,
               "Failed to add edge ({}, {}) -> ({}, {}) intersects obstruction",
               point0.x(),
               point0.y(),
               point1.x(),
               point1.y());
    return false;
  }

  bool added;
  grid_edge edge;

  bool exists;
  boost::tie(edge, exists) = boost::lookup_edge(v0, v1, graph_);
  if (exists) {
    return true;
  }
  boost::tie(edge, exists) = boost::lookup_edge(v1, v0, graph_);
  if (exists) {
    return true;
  }

  boost::tie(edge, added) = boost::add_edge(v0, v1, graph_);
  if (!added) {
    return false;
  }

  const int64_t weight = edge_weight_scale * distance(point0, point1);

  debugPrint(logger_,
             utl::PAD,
             "Router_edge",
             1,
             "Adding edge from ({}, {}) to ({}, {}) with weight {}",
             point0.x(),
             point0.y(),
             point1.x(),
             point1.y(),
             weight);
  graph_weight_[edge] = weight;

  return true;
}

std::vector<std::pair<odb::Point, odb::Point>> RDLRouter::simplifyRoute(
    const std::vector<grid_vertex>& route) const
{
  std::vector<std::pair<odb::Point, odb::Point>> wire;

  enum class Direction
  {
    UNSET,
    HORIZONTAL,
    VERTICAL,
    ANGLE45,
    ANGLE135
  };

  auto get_direction
      = [](const odb::Point& s, const odb::Point& t) -> Direction {
    if (s.y() == t.y()) {
      return Direction::HORIZONTAL;
    }
    if (s.x() == t.x()) {
      return Direction::VERTICAL;
    }
    if (s.x() < t.x() && s.y() < t.y()) {
      return Direction::ANGLE45;
    }
    if (s.x() > t.x() && s.y() > t.y()) {
      return Direction::ANGLE45;
    }
    return Direction::ANGLE135;
  };

  wire.emplace_back(vertex_point_map_.at(route[0]),
                    vertex_point_map_.at(route[1]));
  Direction direction
      = get_direction(wire.begin()->first, wire.begin()->second);
  for (size_t i = 2; i < route.size(); i++) {
    odb::Point s = wire.rbegin()->second;
    odb::Point t = vertex_point_map_.at(route[i]);

    Direction segment_direction = get_direction(s, t);
    if (direction == segment_direction) {
      // Extend segment
      wire.rbegin()->second = t;
    } else {
      // Determine if extentions are needed
      int extention = width_ / 2;
      if (direction == Direction::HORIZONTAL
          && segment_direction == Direction::VERTICAL) {
        const odb::Point& prev_s = wire.rbegin()->first;
        if (prev_s.x() < s.x()) {
          wire.rbegin()->second.setX(s.x() + extention);
        } else {
          wire.rbegin()->second.setX(s.x() - extention);
        }
        if (s.y() < t.y()) {
          s.setY(s.y() - extention);
        } else {
          s.setY(s.y() + extention);
        }
      } else if (direction == Direction::VERTICAL
                 && segment_direction == Direction::HORIZONTAL) {
        const odb::Point& prev_s = wire.rbegin()->first;
        if (prev_s.y() < s.y()) {
          wire.rbegin()->second.setY(s.y() + extention);
        } else {
          wire.rbegin()->second.setY(s.y() - extention);
        }
        if (s.x() < t.x()) {
          s.setX(s.x() - extention);
        } else {
          s.setX(s.x() + extention);
        }
      }

      // Start new segment
      wire.emplace_back(s, t);

      direction = segment_direction;
    }
  }

  return wire;
}

odb::Rect RDLRouter::correctEndPoint(const odb::Rect& route,
                                     const bool is_horizontal,
                                     const odb::Rect& target) const
{
  const int route_width = is_horizontal ? route.dy() : route.dx();
  const int target_width = is_horizontal ? target.dy() : target.dx();

  if (route_width <= target_width) {
    // shape is already fully covered
    return route;
  }

  odb::Rect new_route = route;
  new_route.merge(target);

  return new_route;
}

void RDLRouter::writeToDb(odb::dbNet* net,
                          const std::vector<grid_vertex>& route,
                          const RouteTarget& source,
                          const RouteTarget& target)
{
  Utilities::makeSpecial(net);

  auto* swire = odb::dbSWire::create(net, odb::dbWireType::ROUTED);
  const auto simplified_route = simplifyRoute(route);
  for (size_t i = 0; i < simplified_route.size(); i++) {
    const auto& [s, t] = simplified_route[i];
    odb::Rect shape(s, t);
    shape.bloat(width_ / 2, shape);
    odb::dbSBox::Direction dir;
    if (s.x() == t.x()) {
      shape.set_ylo(shape.yMin() + width_ / 2);
      shape.set_yhi(shape.yMax() - width_ / 2);
      dir = odb::dbSBox::VERTICAL;
    } else if (s.y() == t.y()) {
      shape.set_xlo(shape.xMin() + width_ / 2);
      shape.set_xhi(shape.xMax() - width_ / 2);
      dir = odb::dbSBox::HORIZONTAL;
    } else {
      dir = odb::dbSBox::OCTILINEAR;
    }

    if (dir != odb::dbSBox::OCTILINEAR) {
      if (i == 0) {
        shape = correctEndPoint(shape, s.y() == t.y(), source.shape);
      } else if (i + 1 == simplified_route.size()) {
        shape = correctEndPoint(shape, s.y() == t.y(), target.shape);
      }
    }

    if (dir != odb::dbSBox::OCTILINEAR) {
      odb::dbSBox::create(swire,
                          layer_,
                          shape.xMin(),
                          shape.yMin(),
                          shape.xMax(),
                          shape.yMax(),
                          odb::dbWireShapeType::IOWIRE);
    } else {
      odb::dbSBox::create(swire,
                          layer_,
                          s.x(),
                          s.y(),
                          t.x(),
                          t.y(),
                          odb::dbWireShapeType::IOWIRE,
                          odb::dbSBox::OCTILINEAR,
                          width_);
    }
  }

  if (source.layer != layer_) {
    odb::dbTechVia* via = pad_accessvia_;
    if (isCoverTerm(source.terminal)) {
      via = bump_accessvia_;
    }
    odb::dbSBox::create(swire,
                        via,
                        source.center.x(),
                        source.center.y(),
                        odb::dbWireShapeType::IOWIRE);
  }
  if (target.layer != layer_) {
    odb::dbTechVia* via = pad_accessvia_;
    if (isCoverTerm(target.terminal)) {
      via = bump_accessvia_;
    }
    odb::dbSBox::create(swire,
                        via,
                        target.center.x(),
                        target.center.y(),
                        odb::dbWireShapeType::IOWIRE);
  }
}

int RDLRouter::getBloatFactor() const
{
  return width_ / 2 + spacing_;
}

std::set<odb::Rect> RDLRouter::getITermShapes(odb::dbITerm* iterm) const
{
  std::set<odb::Rect> rects;

  const odb::dbTransform xform = iterm->getInst()->getTransform();

  for (auto* mpin : iterm->getMTerm()->getMPins()) {
    for (auto* geom : mpin->getGeometry()) {
      if (geom->getTechLayer() != layer_) {
        continue;
      }

      odb::Rect rect = geom->getBox();
      xform.apply(rect);
      rects.insert(rect);
    }
  }

  return rects;
}

void RDLRouter::populateObstructions(const std::vector<odb::dbNet*>& nets)
{
  obstructions_.clear();

  const int bloat = getBloatFactor();
  auto rect_to_poly = [bloat](const odb::Rect& rect) -> Box {
    odb::Rect bloated;
    rect.bloat(bloat, bloated);
    return Box(Point(bloated.xMin(), bloated.yMin()),
               Point(bloated.xMax(), bloated.yMax()));
  };

  // Get placed instanced obstructions
  for (auto* inst : block_->getInsts()) {
    if (!inst->isPlaced()) {
      continue;
    }

    const odb::dbTransform xform = inst->getTransform();

    auto* master = inst->getMaster();
    for (auto* obs : master->getObstructions()) {
      if (obs->getTechLayer() != layer_) {
        continue;
      }

      odb::Rect rect = obs->getBox();
      xform.apply(rect);
      obstructions_.insert({rect_to_poly(rect), nullptr});
    }

    for (auto* mterm : master->getMTerms()) {
      auto* iterm = inst->getITerm(mterm);
      auto* net = iterm->getNet();
      for (const auto& rect : getITermShapes(iterm)) {
        obstructions_.insert({rect_to_poly(rect), net});
      }
    }
  }

  // Get already routed nets obstructions, excluding those that will be routed
  // now
  for (auto* net : block_->getNets()) {
    if (std::find(nets.begin(), nets.end(), net) != nets.end()) {
      continue;
    }

    for (auto* swire : net->getSWires()) {
      for (auto* box : swire->getWires()) {
        if (box->getTechLayer() != layer_) {
          continue;
        }

        obstructions_.insert({rect_to_poly(box->getBox()), net});
      }
    }
  }

  // Get routing obstructions
  for (auto* obs : block_->getObstructions()) {
    auto* box = obs->getBBox();
    if (box->getTechLayer() != layer_) {
      continue;
    }

    obstructions_.insert({rect_to_poly(box->getBox()), nullptr});
  }

  // Add via obstructions when using access vias
  for (const auto& [net, routing_pairs] : routing_targets_) {
    for (const auto& [iterm, targets] : routing_pairs) {
      for (const auto& target : targets) {
        if (target.layer != layer_) {
          obstructions_.insert({rect_to_poly(target.shape), net});
        }
      }
    }
  }
}

int64_t RDLRouter::distance(const odb::Point& p0, const odb::Point& p1)
{
  const int64_t dx = p0.x() - p1.x();
  const int64_t dy = p0.y() - p1.y();
  return std::sqrt(dx * dx + dy * dy);
}

int64_t RDLRouter::distance(const TargetPair& pair)
{
  return distance(pair.target0->center, pair.target1->center);
}

bool RDLRouter::isCoverTerm(odb::dbITerm* term) const
{
  return term->getMTerm()->getMaster()->getType().isCover();
}

odb::dbTechLayer* RDLRouter::getOtherLayer(odb::dbTechVia* via) const
{
  if (via != nullptr) {
    if (via->getBottomLayer() != layer_) {
      return via->getBottomLayer();
    }
    if (via->getTopLayer() != layer_) {
      return via->getTopLayer();
    }
  }
  return nullptr;
}

std::map<odb::dbITerm*, std::vector<RDLRouter::RouteTarget>>
RDLRouter::generateRoutingTargets(odb::dbNet* net) const
{
  std::map<odb::dbITerm*, std::vector<RDLRouter::RouteTarget>> targets;
  odb::dbTechLayer* bump_pin_layer = getOtherLayer(bump_accessvia_);
  odb::dbTechLayer* pad_pin_layer = getOtherLayer(pad_accessvia_);

  for (auto* iterm : net->getITerms()) {
    if (!iterm->getInst()->isPlaced()) {
      continue;
    }

    odb::dbTechLayer* other_layer;
    odb::dbTechVia* via;
    if (isCoverTerm(iterm)) {
      other_layer = bump_pin_layer;
      via = bump_accessvia_;
    } else {
      other_layer = pad_pin_layer;
      via = pad_accessvia_;
    }

    const odb::dbTransform xform = iterm->getInst()->getTransform();

    for (auto* mpin : iterm->getMTerm()->getMPins()) {
      for (auto* geom : mpin->getGeometry()) {
        odb::dbTechLayer* found_layer = geom->getTechLayer();
        if (found_layer != layer_ && found_layer != other_layer) {
          continue;
        }

        odb::Rect box = geom->getBox();
        if (found_layer == other_layer) {
          for (const auto& viabox : via->getBoxes()) {
            if (viabox->getTechLayer() == other_layer) {
              odb::Rect via_encl = viabox->getBox();
              via_encl.moveDelta(box.xCenter(), box.yCenter());
              box = via_encl;
              break;
            }
          }
        }
        xform.apply(box);

        targets[iterm].push_back({box.center(), box, iterm, found_layer});
      }
    }
  }

  if (targets.size() < 2) {
    logger_->error(utl::PAD,
                   10,
                   "{} only has one iterm on {} layer",
                   net->getName(),
                   layer_->getName());
  }

  debugPrint(logger_,
             utl::PAD,
             "Router",
             1,
             "{} has {} targets",
             net->getName(),
             targets.size());

  return targets;
}

/////////////////////////////////////

RDLGui::RDLGui()
{
  addDisplayControl(draw_vertex_, true);
  addDisplayControl(draw_edge_, true);
  addDisplayControl(draw_obs_, true);
  addDisplayControl(draw_targets_, true);
  addDisplayControl(draw_fly_wires_, true);
  addDisplayControl(draw_routes_, true);
}

RDLGui::~RDLGui()
{
  if (router_ != nullptr) {
    router_->setRDLGui(nullptr);
  }
}

void RDLGui::drawObjects(gui::Painter& painter)
{
  if (router_ == nullptr) {
    return;
  }
  const bool draw_detail = painter.getPixelsPerDBU() * 1000 >= 1;

  const odb::Rect box = painter.getBounds();

  const auto& vertex_map = router_->getVertexMap();

  std::map<odb::dbITerm*, const RDLRouter::NetRoute*> routes;
  for (const auto& [net, net_routes] : router_->getRoutes()) {
    for (const auto& route : net_routes) {
      routes[route.source->terminal] = &route;
    }
  }

  const bool draw_obs = draw_detail && checkDisplayControl(draw_obs_);
  if (draw_obs) {
    gui::Painter::Color obs_color = gui::Painter::cyan;
    obs_color.a = 127;
    painter.setPenAndBrush(obs_color, true);

    for (const auto& [box, ptr] : router_->getObstructions()) {
      const odb::Rect rect(box.min_corner().x(),
                           box.min_corner().y(),
                           box.max_corner().x(),
                           box.max_corner().y());
      painter.drawRect(rect);
    }
  }

  const bool draw_vertex = draw_detail && checkDisplayControl(draw_vertex_);
  const bool draw_edge = draw_detail && checkDisplayControl(draw_edge_);

  std::vector<RDLRouter::GridGraph::vertex_descriptor> vertex;
  if (draw_vertex || draw_edge) {
    RDLRouter::GridGraph::vertex_iterator v, vend;
    for (boost::tie(v, vend) = boost::vertices(router_->getGraph()); v != vend;
         ++v) {
      const odb::Point& pt = vertex_map.at(*v);
      if (box.contains({pt, pt})) {
        vertex.push_back(*v);
      }
    }
  }

  if (draw_vertex) {
    painter.setPenAndBrush(gui::Painter::red, true);

    for (const auto& v : vertex) {
      const odb::Point& pt = vertex_map.at(v);
      painter.drawCircle(pt.x(), pt.y(), 100);
    }
  }

  if (draw_edge) {
    gui::Painter::Color edge_color = gui::Painter::green;
    edge_color.a = 127;
    painter.setPenAndBrush(edge_color, true);

    for (const auto& v : vertex) {
      RDLRouter::GridGraph::out_edge_iterator eit, eend;
      std::tie(eit, eend) = boost::out_edges(v, router_->getGraph());
      for (; eit != eend; eit++) {
        const odb::Point& pt0 = vertex_map.at(eit->m_source);
        const odb::Point& pt1 = vertex_map.at(eit->m_target);
        painter.drawLine(pt0, pt1);
      }
    }
  }

  const bool draw_flywires = checkDisplayControl(draw_fly_wires_);
  if (draw_flywires) {
    painter.setPenAndBrush(
        gui::Painter::yellow, true, gui::Painter::Brush::SOLID, 3);

    const auto& targets = router_->getRoutingTargets();
    const auto& route_map = router_->getRoutingMap();

    for (const auto& [iterm, status] : route_map) {
      if (status.next == status.terminals.end()) {
        continue;
      }

      if (routes.find(iterm) != routes.end()) {
        continue;
      }

      const auto& net_targets = targets.at(iterm->getNet());

      odb::dbITerm* dst_iterm = *status.next;
      const auto& src = net_targets.at(iterm);
      const auto& dst = net_targets.at(dst_iterm);
      painter.drawLine(src[0].center, dst[0].center);
    }

    painter.setPenAndBrush(
        gui::Painter::red, true, gui::Painter::Brush::SOLID, 3);
    for (auto* fail : router_->getFailed()) {
      for (auto* dst : route_map.at(fail).terminals) {
        painter.drawLine(fail->getBBox().center(), dst->getBBox().center());
      }
    }
  }

  if (checkDisplayControl(draw_routes_)) {
    painter.setPenAndBrush(
        gui::Painter::green, true, gui::Painter::Brush::SOLID, 3);

    for (const auto& [iterm, route] : routes) {
      for (size_t i = 1; i < route->route.size(); i++) {
        const odb::Point& src = vertex_map.at(route->route.at(i - 1));
        const odb::Point& dst = vertex_map.at(route->route.at(i));

        painter.drawLine(src, dst);
      }
    }
  }

  if (checkDisplayControl(draw_targets_)) {
    for (const auto& [net, iterm_targets] : router_->getRoutingTargets()) {
      for (const auto& [iterm, targets] : iterm_targets) {
        for (const auto& target : targets) {
          if (box.intersects(target.shape)) {
            painter.setPenAndBrush(
                gui::Painter::blue, true, gui::Painter::Brush::DIAGONAL);
            painter.drawRect(target.shape);
            painter.setPenAndBrush(gui::Painter::blue, true);
            painter.drawCircle(target.center.x(),
                               target.center.y(),
                               0.05 * target.shape.minDXDY());
          }
        }
      }
    }
  }
}

void RDLGui::setRouter(RDLRouter* router)
{
  router_ = router;
  router_->setRDLGui(this);
}

void RDLGui::pause()
{
  gui::Gui::get()->redraw();
  gui::Gui::get()->pause();
}

}  // namespace pad
