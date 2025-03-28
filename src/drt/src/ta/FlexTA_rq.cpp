/* Authors: Lutong Wang and Bangqi Xu */
/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>

#include "frRTree.h"
#include "ta/FlexTA.h"

namespace drt {

struct FlexTAWorkerRegionQuery::Impl
{
  FlexTAWorker* taWorker;
  std::vector<RTree<taPinFig*>> shapes_;  // resource map
  // fixed objs, owner:: nullptr or net, con = short
  std::vector<RTree<std::pair<frBlockObject*, frConstraint*>>> route_costs_;
  std::vector<RTree<std::pair<frBlockObject*, frConstraint*>>> via_costs_;
};

FlexTAWorkerRegionQuery::FlexTAWorkerRegionQuery(FlexTAWorker* in)
    : impl_(std::make_unique<Impl>())
{
  impl_->taWorker = in;
}

FlexTAWorkerRegionQuery::~FlexTAWorkerRegionQuery() = default;

FlexTAWorker* FlexTAWorkerRegionQuery::getTAWorker() const
{
  return impl_->taWorker;
}

frDesign* FlexTAWorkerRegionQuery::getDesign() const
{
  return impl_->taWorker->getDesign();
}

void FlexTAWorkerRegionQuery::add(taPinFig* fig)
{
  Rect box;
  if (fig->typeId() == tacPathSeg) {
    auto obj = static_cast<taPathSeg*>(fig);
    auto [bp, ep] = obj->getPoints();
    box = Rect(bp, ep);
    impl_->shapes_.at(obj->getLayerNum()).insert(std::make_pair(box, obj));
  } else if (fig->typeId() == tacVia) {
    auto obj = static_cast<taVia*>(fig);
    auto bp = obj->getOrigin();
    box = Rect(bp, bp);
    impl_->shapes_.at(obj->getViaDef()->getCutLayerNum())
        .insert(std::make_pair(box, obj));
  } else {
    std::cout << "Error: unsupported region query add" << std::endl;
  }
}

void FlexTAWorkerRegionQuery::remove(taPinFig* fig)
{
  Rect box;
  if (fig->typeId() == tacPathSeg) {
    auto obj = static_cast<taPathSeg*>(fig);
    auto [bp, ep] = obj->getPoints();
    box = Rect(bp, ep);
    impl_->shapes_.at(obj->getLayerNum()).remove(std::make_pair(box, obj));
  } else if (fig->typeId() == tacVia) {
    auto obj = static_cast<taVia*>(fig);
    auto bp = obj->getOrigin();
    box = Rect(bp, bp);
    impl_->shapes_.at(obj->getViaDef()->getCutLayerNum())
        .remove(std::make_pair(box, obj));
  } else {
    std::cout << "Error: unsupported region query add" << std::endl;
  }
}

void FlexTAWorkerRegionQuery::query(
    const Rect& box,
    const frLayerNum layerNum,
    std::set<taPin*, frBlockObjectComp>& result) const
{
  std::vector<rq_box_value_t<taPinFig*>> temp;
  auto& tree = impl_->shapes_.at(layerNum);
  transform(tree.qbegin(bgi::intersects(box)),
            tree.qend(),
            inserter(result, result.end()),
            [](const auto& box_fig) { return box_fig.second->getPin(); });
}

void FlexTAWorkerRegionQuery::init()
{
  int numLayers = getDesign()->getTech()->getLayers().size();
  impl_->shapes_.clear();
  impl_->shapes_.resize(numLayers);
  impl_->route_costs_.clear();
  impl_->route_costs_.resize(numLayers);
  impl_->via_costs_.clear();
  impl_->via_costs_.resize(numLayers);
}

void FlexTAWorkerRegionQuery::addCost(const Rect& box,
                                      const frLayerNum layerNum,
                                      frBlockObject* obj,
                                      frConstraint* con)
{
  impl_->route_costs_.at(layerNum).insert(
      std::make_pair(box, std::make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::removeCost(const Rect& box,
                                         const frLayerNum layerNum,
                                         frBlockObject* obj,
                                         frConstraint* con)
{
  impl_->route_costs_.at(layerNum).remove(
      std::make_pair(box, std::make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::queryCost(
    const Rect& box,
    const frLayerNum layerNum,
    std::vector<rq_box_value_t<std::pair<frBlockObject*, frConstraint*>>>&
        result) const
{
  impl_->route_costs_.at(layerNum).query(bgi::intersects(box),
                                         back_inserter(result));
}

void FlexTAWorkerRegionQuery::addViaCost(const Rect& box,
                                         const frLayerNum layerNum,
                                         frBlockObject* obj,
                                         frConstraint* con)
{
  impl_->via_costs_.at(layerNum).insert(
      std::make_pair(box, std::make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::removeViaCost(const Rect& box,
                                            const frLayerNum layerNum,
                                            frBlockObject* obj,
                                            frConstraint* con)
{
  impl_->via_costs_.at(layerNum).remove(
      std::make_pair(box, std::make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::queryViaCost(
    const Rect& box,
    const frLayerNum layerNum,
    std::vector<rq_box_value_t<std::pair<frBlockObject*, frConstraint*>>>&
        result) const
{
  impl_->via_costs_.at(layerNum).query(bgi::intersects(box),
                                       back_inserter(result));
}

}  // namespace drt
