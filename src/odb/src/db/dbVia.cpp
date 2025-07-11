// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "dbVia.h"

#include <string>
#include <vector>

#include "dbBlock.h"
#include "dbBox.h"
#include "dbBoxItr.h"
#include "dbChip.h"
#include "dbCommon.h"
#include "dbDatabase.h"
#include "dbTable.h"
#include "dbTable.hpp"
#include "dbTech.h"
#include "dbTechLayer.h"
#include "dbTechVia.h"
#include "dbTechViaGenerateRule.h"
#include "odb/db.h"
#include "odb/dbSet.h"

namespace odb {

template class dbTable<_dbVia>;
static void create_via_boxes(_dbVia* via, const dbViaParams& P);

////////////////////////////////////////////////////////////////////
//
// _dbVia - Methods
//
////////////////////////////////////////////////////////////////////

bool _dbVia::operator==(const _dbVia& rhs) const
{
  if (_flags._is_rotated != rhs._flags._is_rotated) {
    return false;
  }

  if (_flags._is_tech_via != rhs._flags._is_tech_via) {
    return false;
  }

  if (_flags._has_params != rhs._flags._has_params) {
    return false;
  }

  if (_flags._orient != rhs._flags._orient) {
    return false;
  }

  if (_flags.default_ != rhs._flags.default_) {
    return false;
  }

  if (_name && rhs._name) {
    if (strcmp(_name, rhs._name) != 0) {
      return false;
    }
  } else if (_name || rhs._name) {
    return false;
  }

  if (_pattern && rhs._pattern) {
    if (strcmp(_pattern, rhs._pattern) != 0) {
      return false;
    }
  } else if (_pattern || rhs._pattern) {
    return false;
  }

  if (_bbox != rhs._bbox) {
    return false;
  }

  if (_boxes != rhs._boxes) {
    return false;
  }

  if (_top != rhs._top) {
    return false;
  }

  if (_bottom != rhs._bottom) {
    return false;
  }

  if (_generate_rule != rhs._generate_rule) {
    return false;
  }

  if (_rotated_via_id != rhs._rotated_via_id) {
    return false;
  }

  if (_via_params != rhs._via_params) {
    return false;
  }

  return true;
}

_dbVia::_dbVia(_dbDatabase*, const _dbVia& v)
    : _flags(v._flags),
      _name(nullptr),
      _pattern(nullptr),
      _bbox(v._bbox),
      _boxes(v._boxes),
      _top(v._top),
      _bottom(v._bottom),
      _generate_rule(v._generate_rule),
      _rotated_via_id(v._rotated_via_id),
      _via_params(v._via_params)
{
  if (v._name) {
    _name = safe_strdup(v._name);
  }

  if (v._pattern) {
    _pattern = safe_strdup(v._pattern);
  }
}

_dbVia::_dbVia(_dbDatabase*)
{
  _flags._is_rotated = 0;
  _flags._is_tech_via = 0;
  _flags._has_params = 0;
  _flags._orient = dbOrientType::R0;
  _flags.default_ = false;
  _flags._spare_bits = 0;
  _name = nullptr;
  _pattern = nullptr;
}

_dbVia::~_dbVia()
{
  if (_name) {
    free((void*) _name);
  }

  if (_pattern) {
    free((void*) _pattern);
  }
}

_dbTech* _dbVia::getTech()
{
  _dbBlock* block = (_dbBlock*) getOwner();
  return block->getTech();
}

dbOStream& operator<<(dbOStream& stream, const _dbVia& v)
{
  uint* bit_field = (uint*) &v._flags;
  stream << *bit_field;
  stream << v._name;
  stream << v._pattern;
  stream << v._bbox;
  stream << v._boxes;
  stream << v._top;
  stream << v._bottom;
  stream << v._generate_rule;
  stream << v._rotated_via_id;
  stream << v._via_params;
  return stream;
}

dbIStream& operator>>(dbIStream& stream, _dbVia& v)
{
  uint* bit_field = (uint*) &v._flags;
  stream >> *bit_field;
  stream >> v._name;
  stream >> v._pattern;
  stream >> v._bbox;
  stream >> v._boxes;
  stream >> v._top;
  stream >> v._bottom;
  stream >> v._generate_rule;
  stream >> v._rotated_via_id;
  stream >> v._via_params;

  return stream;
}

////////////////////////////////////////////////////////////////////
//
// dbVia - Methods
//
////////////////////////////////////////////////////////////////////

std::string dbVia::getName()
{
  _dbVia* via = (_dbVia*) this;
  return via->_name;
}

const char* dbVia::getConstName()
{
  _dbVia* via = (_dbVia*) this;
  return via->_name;
}

std::string dbVia::getPattern()
{
  _dbVia* via = (_dbVia*) this;

  if (via->_pattern == nullptr) {
    return "";
  }

  return via->_pattern;
}

void dbVia::setPattern(const char* name)
{
  _dbVia* via = (_dbVia*) this;

  if (via->_pattern != nullptr) {
    return;
  }

  via->_pattern = safe_strdup(name);
}

dbBlock* dbVia::getBlock()
{
  return (dbBlock*) getImpl()->getOwner();
}

dbBox* dbVia::getBBox()
{
  _dbVia* via = (_dbVia*) this;

  if (via->_bbox == 0) {
    return nullptr;
  }

  _dbBlock* block = (_dbBlock*) via->getOwner();
  return (dbBox*) block->_box_tbl->getPtr(via->_bbox);
}

bool dbVia::isViaRotated()
{
  _dbVia* via = (_dbVia*) this;
  return via->_flags._is_rotated == 1;
}

dbOrientType dbVia::getOrient()
{
  _dbVia* via = (_dbVia*) this;
  dbOrientType o(via->_flags._orient);
  return o;
}

dbTechVia* dbVia::getTechVia()
{
  _dbVia* via = (_dbVia*) this;

  if ((via->_flags._is_rotated == 0) || (via->_flags._is_tech_via == 0)) {
    return nullptr;
  }

  _dbTech* tech = via->getTech();
  _dbTechVia* v = tech->_via_tbl->getPtr(via->_rotated_via_id);
  return (dbTechVia*) v;
}

dbVia* dbVia::getBlockVia()
{
  _dbVia* via = (_dbVia*) this;

  if ((via->_flags._is_rotated == 0) || (via->_flags._is_tech_via == 1)) {
    return nullptr;
  }

  _dbBlock* block = (_dbBlock*) via->getOwner();
  _dbVia* v = block->_via_tbl->getPtr(via->_rotated_via_id);
  return (dbVia*) v;
}

dbSet<dbBox> dbVia::getBoxes()
{
  _dbVia* via = (_dbVia*) this;
  _dbBlock* block = (_dbBlock*) via->getOwner();
  return dbSet<dbBox>(via, block->_box_itr);
}

dbTechLayer* dbVia::getTopLayer()
{
  _dbVia* via = (_dbVia*) this;

  if (via->_top == 0) {
    return nullptr;
  }

  _dbTech* tech = via->getTech();
  return (dbTechLayer*) tech->_layer_tbl->getPtr(via->_top);
}

dbTechLayer* dbVia::getBottomLayer()
{
  _dbVia* via = (_dbVia*) this;

  if (via->_bottom == 0) {
    return nullptr;
  }

  _dbTech* tech = via->getTech();
  return (dbTechLayer*) tech->_layer_tbl->getPtr(via->_bottom);
}

bool dbVia::hasParams()
{
  _dbVia* via = (_dbVia*) this;
  return via->_flags._has_params == 1;
}

void dbVia::setViaGenerateRule(dbTechViaGenerateRule* rule)
{
  _dbVia* via = (_dbVia*) this;
  via->_generate_rule = rule->getImpl()->getOID();
}

dbTechViaGenerateRule* dbVia::getViaGenerateRule()
{
  _dbVia* via = (_dbVia*) this;

  if (via->_generate_rule == 0) {
    return nullptr;
  }

  _dbTech* tech = via->getTech();
  auto rule = tech->_via_generate_rule_tbl->getPtr(via->_generate_rule);
  return (dbTechViaGenerateRule*) rule;
}

void dbVia::setViaParams(const dbViaParams& params)
{
  _dbVia* via = (_dbVia*) this;
  _dbBlock* block = (_dbBlock*) via->getOwner();
  via->_flags._has_params = 1;

  // Clear previous boxes
  dbSet<dbBox> boxes = getBoxes();
  dbSet<dbBox>::iterator itr;

  for (itr = boxes.begin(); itr != boxes.end();) {
    dbSet<dbBox>::iterator cur = itr++;
    _dbBox* box = (_dbBox*) *cur;
    dbProperty::destroyProperties(box);
    block->_box_tbl->destroy(box);
  }

  via->_boxes = 0U;
  via->_via_params = params;
  via->_top = params._top_layer;
  via->_bottom = params._bot_layer;
  create_via_boxes(via, params);
}

dbViaParams dbVia::getViaParams()
{
  dbViaParams params;

  _dbVia* via = (_dbVia*) this;

  if (via->_flags._has_params == 0) {
    params = dbViaParams();
  } else {
    params = via->_via_params;
    params._tech = (dbTech*) via->getTech();
  }

  return params;
}

void dbVia::setDefault(bool val)
{
  _dbVia* via = (_dbVia*) this;
  via->_flags.default_ = val;
}

bool dbVia::isDefault()
{
  _dbVia* via = (_dbVia*) this;
  return via->_flags.default_;
}

dbVia* dbVia::create(dbBlock* block_, const char* name_)
{
  if (block_->findVia(name_)) {
    return nullptr;
  }

  _dbBlock* block = (_dbBlock*) block_;
  _dbVia* via = block->_via_tbl->create();
  via->_name = safe_strdup(name_);
  return (dbVia*) via;
}

dbVia* dbVia::create(dbBlock* block,
                     const char* name,
                     dbVia* blk_via,
                     dbOrientType orient)
{
  _dbVia* via = (_dbVia*) dbVia::create(block, name);

  if (via == nullptr) {
    return nullptr;
  }

  via->_flags._is_rotated = 1;
  via->_flags._orient = orient;
  via->_rotated_via_id = blk_via->getId();

  dbTransform t(orient);

  for (dbBox* box : blk_via->getBoxes()) {
    dbTechLayer* l = box->getTechLayer();
    Rect r = ((_dbBox*) box)->_shape._rect;
    t.apply(r);
    dbBox::create((dbVia*) via, l, r.xMin(), r.yMin(), r.xMax(), r.yMax());
  }

  return (dbVia*) via;
}

dbVia* dbVia::create(dbBlock* block,
                     const char* name,
                     dbTechVia* tech_via,
                     dbOrientType orient)
{
  _dbVia* via = (_dbVia*) dbVia::create(block, name);

  if (via == nullptr) {
    return nullptr;
  }

  via->_flags._is_rotated = 1;
  via->_flags._is_tech_via = 1;
  via->_flags._orient = orient;
  via->_rotated_via_id = tech_via->getId();

  dbTransform t(orient);

  for (dbBox* box : tech_via->getBoxes()) {
    dbTechLayer* l = box->getTechLayer();
    Rect r = ((_dbBox*) box)->_shape._rect;
    t.apply(r);
    dbBox::create((dbVia*) via, l, r.xMin(), r.yMin(), r.xMax(), r.yMax());
  }

  return (dbVia*) via;
}

static dbVia* copyVia(dbBlock* block_, dbVia* via_, bool copyRotatedVia)
{
  _dbBlock* block = (_dbBlock*) block_;
  _dbVia* via = (_dbVia*) via_;

  _dbVia* cvia = block->_via_tbl->create();

  cvia->_flags = via->_flags;
  cvia->_name = safe_strdup(via->_name);

  if (via->_pattern) {
    cvia->_pattern = safe_strdup(via->_pattern);
  }

  cvia->_top = via->_top;
  cvia->_bottom = via->_bottom;

  for (dbBox* b : via_->getBoxes()) {
    dbTechLayer* l = b->getTechLayer();
    dbBox::create((dbVia*) cvia, l, b->xMin(), b->yMin(), b->xMax(), b->yMax());
  }

  if (via->_flags._is_rotated) {
    if (via->_flags._is_tech_via) {
      cvia->_rotated_via_id = via->_rotated_via_id;
    } else {
      _dbVia* bv = (_dbVia*) via_->getBlockVia();
      _dbVia* cbv = (_dbVia*) block_->findVia(bv->_name);

      if (copyRotatedVia && (cbv == nullptr)) {
        cbv = (_dbVia*) copyVia(block_, (dbVia*) bv, true);
      }

      assert(cbv);
      cvia->_rotated_via_id = cbv->getOID();
    }
  }

  return (dbVia*) cvia;
}

dbVia* dbVia::copy(dbBlock* dst, dbVia* src)
{
  return copyVia(dst, src, true);
}

bool dbVia::copy(dbBlock* dst_, dbBlock* src_)
{
  // copy non rotated via's first
  for (dbVia* v : src_->getVias()) {
    _dbVia* via = (_dbVia*) v;

    if (!v->isViaRotated()) {
      if (!dst_->findVia(via->_name)) {
        copyVia(dst_, v, false);
      }
    }
  }

  // copy rotated via's last
  for (dbVia* v : src_->getVias()) {
    _dbVia* via = (_dbVia*) v;

    if (v->isViaRotated()) {
      if (!dst_->findVia(via->_name)) {
        copyVia(dst_, v, false);
      }
    }
  }

  return true;
}

dbVia* dbVia::getVia(dbBlock* block_, uint dbid_)
{
  _dbBlock* block = (_dbBlock*) block_;
  return (dbVia*) block->_via_tbl->getPtr(dbid_);
}

void create_via_boxes(_dbVia* via, const dbViaParams& P)
{
  int rows = P.getNumCutRows();
  int cols = P.getNumCutCols();
  int row;
  int y = 0;
  int maxX = 0;
  int maxY = 0;
  std::vector<Rect> cutRects;

  for (row = 0; row < rows; ++row) {
    int col;
    int x = 0;

    for (col = 0; col < cols; ++col) {
      maxX = x + P.getXCutSize();
      maxY = y + P.getYCutSize();
      Rect r(x, y, maxX, maxY);
      cutRects.push_back(r);
      x = maxX;
      x += P.getXCutSpacing();
    }

    y = maxY;
    y += P.getYCutSpacing();
  }

  dbTechLayer* cut_layer = P.getCutLayer();

  int dx = maxX / 2;
  int dy = maxY / 2;
  std::vector<Rect>::iterator itr;

  for (itr = cutRects.begin(); itr != cutRects.end(); ++itr) {
    Rect& r = *itr;
    r.moveDelta(-dx, -dy);
    r.moveDelta(P.getXOrigin(), P.getYOrigin());
    dbBox::create(
        (dbVia*) via, cut_layer, r.xMin(), r.yMin(), r.xMax(), r.yMax());
  }

  int minX = -dx;
  int minY = -dy;
  maxX -= dx;
  maxY -= dy;

  int top_minX
      = minX - P.getXTopEnclosure() + P.getXOrigin() + P.getXTopOffset();
  int top_minY
      = minY - P.getYTopEnclosure() + P.getYOrigin() + P.getYTopOffset();
  int top_maxX
      = maxX + P.getXTopEnclosure() + P.getXOrigin() + P.getXTopOffset();
  int top_maxY
      = maxY + P.getYTopEnclosure() + P.getYOrigin() + P.getYTopOffset();
  dbBox::create(
      (dbVia*) via, P.getTopLayer(), top_minX, top_minY, top_maxX, top_maxY);

  int bot_minX
      = minX - P.getXBottomEnclosure() + P.getXOrigin() + P.getXBottomOffset();
  int bot_minY
      = minY - P.getYBottomEnclosure() + P.getYOrigin() + P.getYBottomOffset();
  int bot_maxX
      = maxX + P.getXBottomEnclosure() + P.getXOrigin() + P.getXBottomOffset();
  int bot_maxY
      = maxY + P.getYBottomEnclosure() + P.getYOrigin() + P.getYBottomOffset();
  dbBox::create(
      (dbVia*) via, P.getBottomLayer(), bot_minX, bot_minY, bot_maxX, bot_maxY);
}

void _dbVia::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);

  info.children_["name"].add(_name);
  info.children_["pattern"].add(_pattern);
}

}  // namespace odb
