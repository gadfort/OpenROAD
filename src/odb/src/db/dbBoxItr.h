// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include "dbCore.h"
#include "odb/dbId.h"
#include "odb/dbIterator.h"
#include "odb/odb.h"

namespace odb {

class _dbBox;
class _dbPolygon;

template <uint page_size>
class dbBoxItr : public dbIterator
{
 protected:
  dbTable<_dbBox, page_size>* _box_tbl;
  dbTable<_dbPolygon, page_size>* _pbox_tbl;

 public:
  dbBoxItr(dbTable<_dbBox, page_size>* box_tbl,
           dbTable<_dbPolygon, page_size>* pbox_tbl,
           bool include_polygons)
  {
    _box_tbl = box_tbl;
    _pbox_tbl = pbox_tbl;
    include_polygons_ = include_polygons;
  }

  bool reversible() override;
  bool orderReversed() override;
  void reverse(dbObject* parent) override;
  uint sequential() override;
  uint size(dbObject* parent) override;
  uint begin(dbObject* parent) override;
  uint end(dbObject* parent) override;
  uint next(uint id, ...) override;
  dbObject* getObject(uint id, ...) override;

 private:
  // include polygons in iterations
  bool include_polygons_;
};

}  // namespace odb
