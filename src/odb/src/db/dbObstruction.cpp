// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "dbObstruction.h"

#include "dbBlock.h"
#include "dbBox.h"
#include "dbDatabase.h"
#include "dbInst.h"
#include "dbTable.h"
#include "dbTable.hpp"
#include "dbTechLayer.h"
#include "odb/db.h"
#include "odb/dbBlockCallBackObj.h"
#include "odb/dbSet.h"

namespace odb {

template class dbTable<_dbObstruction>;

_dbObstruction::_dbObstruction(_dbDatabase*, const _dbObstruction& o)
    : _flags(o._flags),
      _inst(o._inst),
      _bbox(o._bbox),
      _min_spacing(o._min_spacing),
      _effective_width(o._effective_width)
{
}

_dbObstruction::_dbObstruction(_dbDatabase*)
{
  _flags._slot_obs = 0;
  _flags._fill_obs = 0;
  _flags._except_pg_nets = 0;
  _flags._pushed_down = 0;
  _flags._has_min_spacing = 0;
  _flags._has_effective_width = 0;
  _flags._spare_bits = 0;
  _flags._is_system_reserved = 0;
  _min_spacing = 0;
  _effective_width = 0;
}

_dbObstruction::~_dbObstruction()
{
}

dbOStream& operator<<(dbOStream& stream, const _dbObstruction& obs)
{
  uint* bit_field = (uint*) &obs._flags;
  stream << *bit_field;
  stream << obs._inst;
  stream << obs._bbox;
  stream << obs._min_spacing;
  stream << obs._effective_width;
  return stream;
}

dbIStream& operator>>(dbIStream& stream, _dbObstruction& obs)
{
  uint* bit_field = (uint*) &obs._flags;
  stream >> *bit_field;
  stream >> obs._inst;
  stream >> obs._bbox;
  stream >> obs._min_spacing;
  stream >> obs._effective_width;

  _dbDatabase* db = obs.getImpl()->getDatabase();
  if (!db->isSchema(db_schema_except_pg_nets_obstruction)) {
    // assume false for older databases
    obs._flags._except_pg_nets = false;
  }

  if (!db->isSchema(db_schema_die_area_is_polygon)) {
    // assume false for older databases
    obs._flags._is_system_reserved = false;
  }

  return stream;
}

bool _dbObstruction::operator==(const _dbObstruction& rhs) const
{
  if (_flags._slot_obs != rhs._flags._slot_obs) {
    return false;
  }

  if (_flags._fill_obs != rhs._flags._fill_obs) {
    return false;
  }

  if (_flags._except_pg_nets != rhs._flags._except_pg_nets) {
    return false;
  }

  if (_flags._pushed_down != rhs._flags._pushed_down) {
    return false;
  }

  if (_flags._has_min_spacing != rhs._flags._has_min_spacing) {
    return false;
  }

  if (_flags._has_effective_width != rhs._flags._has_effective_width) {
    return false;
  }

  if (_inst != rhs._inst) {
    return false;
  }

  if (_bbox != rhs._bbox) {
    return false;
  }

  return true;
}

bool _dbObstruction::operator<(const _dbObstruction& rhs) const
{
  _dbBlock* lhs_block = (_dbBlock*) getOwner();
  _dbBlock* rhs_block = (_dbBlock*) rhs.getOwner();
  _dbBox* lhs_box = lhs_block->_box_tbl->getPtr(_bbox);
  _dbBox* rhs_box = rhs_block->_box_tbl->getPtr(rhs._bbox);

  if (*lhs_box < *rhs_box) {
    return true;
  }

  if (lhs_box->equal(*rhs_box)) {
    if (_inst && rhs._inst) {
      _dbBlock* lhs_blk = (_dbBlock*) getOwner();
      _dbBlock* rhs_blk = (_dbBlock*) rhs.getOwner();
      _dbInst* lhs_inst = lhs_blk->_inst_tbl->getPtr(_inst);
      _dbInst* rhs_inst = rhs_blk->_inst_tbl->getPtr(rhs._inst);
      int r = strcmp(lhs_inst->_name, rhs_inst->_name);

      if (r < 0) {
        return true;
      }

      if (r > 0) {
        return false;
      }
    } else if (_inst) {
      return false;
    } else if (rhs._inst) {
      return true;
    }

    if (_flags._slot_obs < rhs._flags._slot_obs) {
      return true;
    }

    if (_flags._slot_obs > rhs._flags._slot_obs) {
      return false;
    }

    if (_flags._fill_obs < rhs._flags._fill_obs) {
      return true;
    }

    if (_flags._fill_obs > rhs._flags._fill_obs) {
      return false;
    }

    if (_flags._except_pg_nets < rhs._flags._except_pg_nets) {
      return true;
    }

    if (_flags._except_pg_nets > rhs._flags._except_pg_nets) {
      return false;
    }

    if (_flags._pushed_down < rhs._flags._pushed_down) {
      return true;
    }

    if (_flags._pushed_down > rhs._flags._pushed_down) {
      return false;
    }

    if (_flags._has_min_spacing < rhs._flags._has_min_spacing) {
      return true;
    }

    if (_flags._has_min_spacing > rhs._flags._has_min_spacing) {
      return false;
    }

    if (_flags._has_effective_width < rhs._flags._has_effective_width) {
      return true;
    }

    if (_flags._has_effective_width > rhs._flags._has_effective_width) {
      return false;
    }

    if (_min_spacing < rhs._min_spacing) {
      return true;
    }

    if (_min_spacing > rhs._min_spacing) {
      return false;
    }

    if (_effective_width < rhs._effective_width) {
      return true;
    }

    if (_effective_width > rhs._effective_width) {
      return false;
    }
  }

  return false;
}

////////////////////////////////////////////////////////////////////
//
// dbObstruction - Methods
//
////////////////////////////////////////////////////////////////////

dbBox* dbObstruction::getBBox()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  _dbBlock* block = (_dbBlock*) obs->getOwner();
  return (dbBox*) block->_box_tbl->getPtr(obs->_bbox);
}

dbInst* dbObstruction::getInstance()
{
  _dbObstruction* obs = (_dbObstruction*) this;

  if (obs->_inst == 0) {
    return nullptr;
  }

  _dbBlock* block = (_dbBlock*) obs->getOwner();
  return (dbInst*) block->_inst_tbl->getPtr(obs->_inst);
}

void dbObstruction::setSlotObstruction()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  obs->_flags._slot_obs = 1;
}

bool dbObstruction::isSlotObstruction()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_flags._slot_obs == 1;
}

void dbObstruction::setFillObstruction()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  obs->_flags._fill_obs = 1;
}

bool dbObstruction::isFillObstruction()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_flags._fill_obs == 1;
}

void dbObstruction::setExceptPGNetsObstruction()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  obs->_flags._except_pg_nets = 1;
}

bool dbObstruction::isExceptPGNetsObstruction()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_flags._except_pg_nets == 1;
}

void dbObstruction::setPushedDown()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  obs->_flags._pushed_down = 1;
}

bool dbObstruction::isPushedDown()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_flags._pushed_down == 1;
}

bool dbObstruction::hasEffectiveWidth()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_flags._has_effective_width == 1U;
}

void dbObstruction::setEffectiveWidth(int w)
{
  _dbObstruction* obs = (_dbObstruction*) this;
  obs->_flags._has_effective_width = 1U;
  obs->_effective_width = w;
}

int dbObstruction::getEffectiveWidth()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_effective_width;
}

bool dbObstruction::hasMinSpacing()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_flags._has_min_spacing == 1U;
}

void dbObstruction::setMinSpacing(int w)
{
  _dbObstruction* obs = (_dbObstruction*) this;
  obs->_flags._has_min_spacing = 1U;
  obs->_min_spacing = w;
}

int dbObstruction::getMinSpacing()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_min_spacing;
}

dbBlock* dbObstruction::getBlock()
{
  return (dbBlock*) getImpl()->getOwner();
}

bool dbObstruction::isSystemReserved()
{
  _dbObstruction* obs = (_dbObstruction*) this;
  return obs->_flags._is_system_reserved;
}

void dbObstruction::setIsSystemReserved(bool is_system_reserved)
{
  _dbObstruction* obs = (_dbObstruction*) this;
  obs->_flags._is_system_reserved = is_system_reserved;
}

dbObstruction* dbObstruction::create(dbBlock* block_,
                                     dbTechLayer* layer_,
                                     int x1,
                                     int y1,
                                     int x2,
                                     int y2,
                                     dbInst* inst_)
{
  _dbBlock* block = (_dbBlock*) block_;
  _dbTechLayer* layer = (_dbTechLayer*) layer_;
  _dbInst* inst = (_dbInst*) inst_;

  _dbObstruction* obs = block->_obstruction_tbl->create();

  if (inst) {
    obs->_inst = inst->getOID();
  }

  _dbBox* box = block->_box_tbl->create();
  box->_shape._rect.init(x1, y1, x2, y2);
  box->_flags._owner_type = dbBoxOwner::OBSTRUCTION;
  box->_owner = obs->getOID();
  box->_flags._layer_id = layer->getOID();
  obs->_bbox = box->getOID();

  // Update bounding box of block
  block->add_rect(box->_shape._rect);
  for (auto callback : block->_callbacks) {
    callback->inDbObstructionCreate((dbObstruction*) obs);
  }
  return (dbObstruction*) obs;
}

void dbObstruction::destroy(dbObstruction* obstruction)
{
  _dbObstruction* obs = (_dbObstruction*) obstruction;
  _dbBlock* block = (_dbBlock*) obs->getOwner();

  if (obstruction->isSystemReserved()) {
    utl::Logger* logger = block->getLogger();
    logger->error(
        utl::ODB,
        1111,
        "You cannot delete a system created obstruction (isSystemReserved).");
  }

  for (auto callback : block->_callbacks) {
    callback->inDbObstructionDestroy(obstruction);
  }
  dbProperty::destroyProperties(obs);
  block->_obstruction_tbl->destroy(obs);
}

dbSet<dbObstruction>::iterator dbObstruction::destroy(
    dbSet<dbObstruction>::iterator& itr)
{
  dbObstruction* bt = *itr;
  dbSet<dbObstruction>::iterator next = ++itr;
  destroy(bt);
  return next;
}

dbObstruction* dbObstruction::getObstruction(dbBlock* block_, uint dbid_)
{
  _dbBlock* block = (_dbBlock*) block_;
  return (dbObstruction*) block->_obstruction_tbl->getPtr(dbid_);
}

void _dbObstruction::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);
}
}  // namespace odb
