// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "dbBTerm.h"

#include <optional>
#include <string>

#include "dbArrayTable.h"
#include "dbBPinItr.h"
#include "dbBlock.h"
#include "dbBox.h"
#include "dbBoxItr.h"
#include "dbChip.h"
#include "dbCommon.h"
#include "dbDatabase.h"
#include "dbHier.h"
#include "dbITerm.h"
#include "dbInst.h"
#include "dbInstHdr.h"
#include "dbJournal.h"
#include "dbMTerm.h"
#include "dbMaster.h"
#include "dbModNet.h"
#include "dbNet.h"
#include "dbTable.h"
#include "dbTable.hpp"
#include "odb/db.h"
#include "odb/dbBlockCallBackObj.h"
#include "odb/dbShape.h"
#include "odb/dbTransform.h"
#include "utl/Logger.h"

namespace odb {

template class dbTable<_dbBTerm>;

_dbBTerm::_dbBTerm(_dbDatabase*)
{
  _flags._io_type = dbIoType::INPUT;
  _flags._sig_type = dbSigType::SIGNAL;
  _flags._orient = 0;
  _flags._status = 0;
  _flags._spef = 0;
  _flags._special = 0;
  _flags._mark = 0;
  _flags._spare_bits = 0;
  _ext_id = 0;
  _name = nullptr;
  _sta_vertex_id = 0;
  _constraint_region.mergeInit();
  _is_mirrored = false;
}

_dbBTerm::_dbBTerm(_dbDatabase*, const _dbBTerm& b)
    : _flags(b._flags),
      _ext_id(b._ext_id),
      _name(nullptr),
      _next_entry(b._next_entry),
      _net(b._net),
      _next_bterm(b._next_bterm),
      _prev_bterm(b._prev_bterm),
      _parent_block(b._parent_block),
      _parent_iterm(b._parent_iterm),
      _bpins(b._bpins),
      _ground_pin(b._ground_pin),
      _supply_pin(b._supply_pin),
      _sta_vertex_id(0),
      _constraint_region(b._constraint_region)
{
  if (b._name) {
    _name = safe_strdup(b._name);
  }
}

_dbBTerm::~_dbBTerm()
{
  if (_name) {
    free((void*) _name);
  }
}

bool _dbBTerm::operator<(const _dbBTerm& rhs) const
{
  return strcmp(_name, rhs._name) < 0;
}

bool _dbBTerm::operator==(const _dbBTerm& rhs) const
{
  if (_flags._io_type != rhs._flags._io_type) {
    return false;
  }

  if (_flags._sig_type != rhs._flags._sig_type) {
    return false;
  }

  if (_flags._spef != rhs._flags._spef) {
    return false;
  }

  if (_flags._special != rhs._flags._special) {
    return false;
  }

  if (_ext_id != rhs._ext_id) {
    return false;
  }

  if (_name && rhs._name) {
    if (strcmp(_name, rhs._name) != 0) {
      return false;
    }
  } else if (_name || rhs._name) {
    return false;
  }

  if (_next_entry != rhs._next_entry) {
    return false;
  }

  if (_net != rhs._net) {
    return false;
  }

  if (_next_bterm != rhs._next_bterm) {
    return false;
  }

  if (_prev_bterm != rhs._prev_bterm) {
    return false;
  }

  if (_parent_block != rhs._parent_block) {
    return false;
  }

  if (_parent_iterm != rhs._parent_iterm) {
    return false;
  }

  if (_bpins != rhs._bpins) {
    return false;
  }

  if (_ground_pin != rhs._ground_pin) {
    return false;
  }

  if (_supply_pin != rhs._supply_pin) {
    return false;
  }

  return true;
}

dbOStream& operator<<(dbOStream& stream, const _dbBTerm& bterm)
{
  uint* bit_field = (uint*) &bterm._flags;
  stream << *bit_field;
  stream << bterm._ext_id;
  stream << bterm._name;
  stream << bterm._next_entry;
  stream << bterm._net;
  stream << bterm._next_bterm;
  stream << bterm._prev_bterm;
  stream << bterm._mnet;
  stream << bterm._next_modnet_bterm;
  stream << bterm._prev_modnet_bterm;
  stream << bterm._parent_block;
  stream << bterm._parent_iterm;
  stream << bterm._bpins;
  stream << bterm._ground_pin;
  stream << bterm._supply_pin;
  stream << bterm._constraint_region;
  stream << bterm._mirrored_bterm;
  stream << bterm._is_mirrored;

  return stream;
}

dbIStream& operator>>(dbIStream& stream, _dbBTerm& bterm)
{
  dbBlock* block = (dbBlock*) (bterm.getOwner());
  _dbDatabase* db = (_dbDatabase*) (block->getDataBase());
  uint* bit_field = (uint*) &bterm._flags;
  stream >> *bit_field;
  stream >> bterm._ext_id;
  stream >> bterm._name;
  stream >> bterm._next_entry;
  stream >> bterm._net;
  stream >> bterm._next_bterm;
  stream >> bterm._prev_bterm;
  if (db->isSchema(db_schema_update_hierarchy)) {
    stream >> bterm._mnet;
    stream >> bterm._next_modnet_bterm;
    stream >> bterm._prev_modnet_bterm;
  }
  stream >> bterm._parent_block;
  stream >> bterm._parent_iterm;
  stream >> bterm._bpins;
  stream >> bterm._ground_pin;
  stream >> bterm._supply_pin;
  if (bterm.getDatabase()->isSchema(db_schema_bterm_constraint_region)) {
    stream >> bterm._constraint_region;
  }
  if (bterm.getDatabase()->isSchema(db_schema_bterm_mirrored_pin)) {
    stream >> bterm._mirrored_bterm;
  }
  if (bterm.getDatabase()->isSchema(db_schema_bterm_is_mirrored)) {
    stream >> bterm._is_mirrored;
  }

  return stream;
}

////////////////////////////////////////////////////////////////////
//
// dbBTerm - Methods
//
////////////////////////////////////////////////////////////////////

std::string dbBTerm::getName()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return bterm->_name;
}

const char* dbBTerm::getConstName()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return bterm->_name;
}

bool dbBTerm::rename(const char* name)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbBlock* block = (_dbBlock*) getBlock();

  if (block->_bterm_hash.hasMember(name)) {
    return false;
  }

  block->_bterm_hash.remove(bterm);
  free((void*) bterm->_name);
  bterm->_name = safe_strdup(name);
  block->_bterm_hash.insert(bterm);

  return true;
}

void dbBTerm::setSigType(dbSigType type)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbBlock* block = (_dbBlock*) getBlock();
  uint prev_flags = flagsToUInt(bterm);

  bterm->_flags._sig_type = type.getValue();

  if (block->_journal) {
    debugPrint(getImpl()->getLogger(),
               utl::ODB,
               "DB_ECO",
               1,
               "ECO: setSigType {}",
               type.getValue());
    block->_journal->updateField(
        this, _dbBTerm::FLAGS, prev_flags, flagsToUInt(bterm));
  }

  for (auto callback : block->_callbacks) {
    callback->inDbBTermSetSigType(this, type);
  }
}

dbSigType dbBTerm::getSigType()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return dbSigType(bterm->_flags._sig_type);
}

void dbBTerm::setIoType(dbIoType type)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbBlock* block = (_dbBlock*) getBlock();
  uint prev_flags = flagsToUInt(bterm);

  bterm->_flags._io_type = type.getValue();

  if (block->_journal) {
    debugPrint(getImpl()->getLogger(),
               utl::ODB,
               "DB_ECO",
               1,
               "ECO: setIoType {}",
               type.getValue());
    block->_journal->updateField(
        this, _dbBTerm::FLAGS, prev_flags, flagsToUInt(bterm));
  }

  for (auto callback : block->_callbacks) {
    callback->inDbBTermSetIoType(this, type);
  }
}

dbIoType dbBTerm::getIoType()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return dbIoType(bterm->_flags._io_type);
}

void dbBTerm::setSpefMark(uint v)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  bterm->_flags._spef = v;
}
bool dbBTerm::isSetSpefMark()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return bterm->_flags._spef > 0 ? true : false;
}
bool dbBTerm::isSpecial() const
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return bterm->_flags._special > 0 ? true : false;
}
void dbBTerm::setSpecial()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  bterm->_flags._special = 1;
}
void dbBTerm::setMark(uint v)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  bterm->_flags._mark = v;
}
bool dbBTerm::isSetMark()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return bterm->_flags._mark > 0 ? true : false;
}
void dbBTerm::setExtId(uint v)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  bterm->_ext_id = v;
}
uint dbBTerm::getExtId()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return bterm->_ext_id;
}

dbNet* dbBTerm::getNet()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  if (bterm->_net) {
    _dbBlock* block = (_dbBlock*) getBlock();
    _dbNet* net = block->_net_tbl->getPtr(bterm->_net);
    return (dbNet*) net;
  }
  return nullptr;
}

dbModNet* dbBTerm::getModNet()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  if (bterm->_mnet) {
    _dbBlock* block = (_dbBlock*) getBlock();
    _dbModNet* net = block->_modnet_tbl->getPtr(bterm->_mnet);
    return (dbModNet*) net;
  }
  return nullptr;
}

void dbBTerm::connect(dbModNet* mod_net)
{
  dbModule* parent_module = mod_net->getParent();
  _dbBlock* block = (_dbBlock*) (parent_module->getOwner());
  _dbModNet* _mod_net = (_dbModNet*) mod_net;
  _dbBTerm* bterm = (_dbBTerm*) this;
  if (bterm->_mnet == _mod_net->getId()) {
    return;
  }
  if (bterm->_mnet) {
    bterm->disconnectModNet(bterm, block);
  }
  bterm->connectModNet(_mod_net, block);
}

void dbBTerm::connect(dbNet* net_)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbNet* net = (_dbNet*) net_;
  _dbBlock* block = (_dbBlock*) net->getOwner();

  if (net->_flags._dont_touch) {
    net->getLogger()->error(utl::ODB,
                            377,
                            "Attempt to connect bterm to dont_touch net {}",
                            net->_name);
  }

  if (bterm->_net) {
    disconnectDbNet();
  }
  bterm->connectNet(net, block);
}

void dbBTerm::disconnect()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  if (bterm->_net) {
    _dbBlock* block = (_dbBlock*) bterm->getOwner();

    _dbNet* net = block->_net_tbl->getPtr(bterm->_net);
    if (net->_flags._dont_touch) {
      net->getLogger()->error(
          utl::ODB,
          375,
          "Attempt to disconnect bterm of dont_touch net {}",
          net->_name);
    }
    bterm->disconnectNet(bterm, block);
    if (bterm->_mnet) {
      bterm->disconnectModNet(bterm, block);
    }
  }
}

void dbBTerm::disconnectDbModNet()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbBlock* block = (_dbBlock*) bterm->getOwner();
  bterm->disconnectModNet(bterm, block);
}

void dbBTerm::disconnectDbNet()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  if (bterm->_net) {
    _dbBlock* block = (_dbBlock*) bterm->getOwner();

    _dbNet* net = block->_net_tbl->getPtr(bterm->_net);
    if (net->_flags._dont_touch) {
      net->getLogger()->error(
          utl::ODB,
          1106,
          "Attempt to disconnect bterm of dont_touch net {}",
          net->_name);
    }
    bterm->disconnectNet(bterm, block);
  }
}

dbSet<dbBPin> dbBTerm::getBPins()
{
  _dbBlock* block = (_dbBlock*) getBlock();
  return dbSet<dbBPin>(this, block->_bpin_itr);
}

dbITerm* dbBTerm::getITerm()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbBlock* block = (_dbBlock*) getBlock();

  if (bterm->_parent_block == 0) {
    return nullptr;
  }

  _dbChip* chip = (_dbChip*) block->getOwner();
  _dbBlock* parent = chip->_block_tbl->getPtr(bterm->_parent_block);
  return (dbITerm*) parent->_iterm_tbl->getPtr(bterm->_parent_iterm);
}

dbBlock* dbBTerm::getBlock() const
{
  return (dbBlock*) getImpl()->getOwner();
}

Rect dbBTerm::getBBox()
{
  Rect bbox;
  bbox.mergeInit();
  for (dbBPin* pin : getBPins()) {
    bbox.merge(pin->getBBox());
  }
  return bbox;
}

bool dbBTerm::getFirstPin(dbShape& shape)
{
  for (dbBPin* bpin : getBPins()) {
    for (dbBox* box : bpin->getBoxes()) {
      if (bpin->getPlacementStatus() == dbPlacementStatus::UNPLACED
          || bpin->getPlacementStatus() == dbPlacementStatus::NONE
          || box == nullptr) {
        continue;
      }

      if (box->isVia()) {  // This is not possible...
        continue;
      }

      Rect r = box->getBox();
      shape.setSegment(box->getTechLayer(), r);
      return true;
    }
  }

  return false;
}

dbPlacementStatus dbBTerm::getFirstPinPlacementStatus()
{
  dbSet<dbBPin> bpins = getBPins();
  if (bpins.empty()) {
    return dbPlacementStatus::NONE;
  }
  return bpins.begin()->getPlacementStatus();
}

bool dbBTerm::getFirstPinLocation(int& x, int& y)
{
  for (dbBPin* bpin : getBPins()) {
    for (dbBox* box : bpin->getBoxes()) {
      if (bpin->getPlacementStatus() == dbPlacementStatus::UNPLACED
          || bpin->getPlacementStatus() == dbPlacementStatus::NONE
          || box == nullptr) {
        continue;
      }

      if (box->isVia()) {  // This is not possible...
        continue;
      }

      Rect r = box->getBox();
      x = r.xMin() + (int) (r.dx() >> 1U);
      y = r.yMin() + (int) (r.dy() >> 1U);
      return true;
    }
  }

  x = 0;
  y = 0;
  return false;
}

dbBTerm* dbBTerm::getGroundPin()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbBlock* block = (_dbBlock*) getBlock();

  if (bterm->_ground_pin == 0) {
    return nullptr;
  }

  _dbBTerm* ground = block->_bterm_tbl->getPtr(bterm->_ground_pin);
  return (dbBTerm*) ground;
}

void dbBTerm::setGroundPin(dbBTerm* pin)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  bterm->_ground_pin = pin->getImpl()->getOID();
}

dbBTerm* dbBTerm::getSupplyPin()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbBlock* block = (_dbBlock*) getBlock();

  if (bterm->_supply_pin == 0) {
    return nullptr;
  }

  _dbBTerm* supply = block->_bterm_tbl->getPtr(bterm->_supply_pin);
  return (dbBTerm*) supply;
}

void dbBTerm::setSupplyPin(dbBTerm* pin)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  bterm->_supply_pin = pin->getImpl()->getOID();
}

dbBTerm* dbBTerm::create(dbNet* net_, const char* name)
{
  _dbNet* net = (_dbNet*) net_;
  _dbBlock* block = (_dbBlock*) net->getOwner();

  if (block->_bterm_hash.hasMember(name)) {
    return nullptr;
  }

  if (net->_flags._dont_touch) {
    net->getLogger()->error(utl::ODB,
                            376,
                            "Attempt to create bterm on dont_touch net {}",
                            net->_name);
  }

  if (block->_journal) {
    debugPrint(block->getImpl()->getLogger(),
               utl::ODB,
               "DB_ECO",
               1,
               "ECO: dbBTerm:create");
    block->_journal->beginAction(dbJournal::CREATE_OBJECT);
    block->_journal->pushParam(dbBTermObj);
    block->_journal->pushParam(net->getId());
    block->_journal->pushParam(name);
    block->_journal->endAction();
  }

  _dbBTerm* bterm = block->_bterm_tbl->create();
  bterm->_name = safe_strdup(name);
  block->_bterm_hash.insert(bterm);

  // If there is a parentInst then we need to update the dbMaster's
  // mterms, the parent dbInst's iterms, and the dbHier to match
  dbBlock* block_public = (dbBlock*) block;
  if (dbInst* inst = block_public->getParentInst()) {
    _dbBlock* parent_block = (_dbBlock*) inst->getBlock();
    dbMaster* master = inst->getMaster();
    _dbMaster* master_impl = (_dbMaster*) master;
    _dbInstHdr* inst_hdr = parent_block->_inst_hdr_hash.find(master_impl->_id);
    ZASSERT(inst_hdr->_inst_cnt == 1);

    master_impl->_flags._frozen = 0;  // allow the mterm creation
    auto mterm = (_dbMTerm*) dbMTerm::create(master, name, dbIoType::INOUT);
    master_impl->_flags._frozen = 1;
    mterm->_order_id = inst_hdr->_mterms.size();
    inst_hdr->_mterms.push_back(mterm->getOID());

    _dbInst* inst_impl = (_dbInst*) inst;
    _dbHier* hier = parent_block->_hier_tbl->getPtr(inst_impl->_hierarchy);
    hier->_child_bterms.push_back(bterm->getOID());

    _dbITerm* iterm = parent_block->_iterm_tbl->create();
    inst_impl->_iterms.push_back(iterm->getOID());
    iterm->_flags._mterm_idx = mterm->_order_id;
    iterm->_inst = inst_impl->getOID();

    bterm->_parent_block = parent_block->getOID();
    bterm->_parent_iterm = inst_impl->_iterms[mterm->_order_id];
  }

  for (auto callback : block->_callbacks) {
    callback->inDbBTermCreate((dbBTerm*) bterm);
  }

  bterm->connectNet(net, block);

  return (dbBTerm*) bterm;
}

void _dbBTerm::connectModNet(_dbModNet* mod_net, _dbBlock* block)
{
  _dbBTerm* bterm = (_dbBTerm*) this;

  _mnet = mod_net->getOID();

  if (block->_journal) {
    debugPrint(block->getImpl()->getLogger(),
               utl::ODB,
               "DB_ECO",
               1,
               "ECO: connect Bterm {} to modnet {}",
               bterm->getId(),
               mod_net->getId());

    block->_journal->beginAction(dbJournal::CONNECT_OBJECT);
    block->_journal->pushParam(dbBTermObj);
    block->_journal->pushParam(bterm->getId());
    // the flat net is left out
    block->_journal->pushParam(0U);
    // modnet
    block->_journal->pushParam(mod_net->getId());
    block->_journal->endAction();
  }

  if (mod_net->_bterms != 0) {
    _dbBTerm* head = block->_bterm_tbl->getPtr(mod_net->_bterms);
    _next_modnet_bterm = mod_net->_bterms;
    head->_prev_modnet_bterm = getOID();
  } else {
    _next_modnet_bterm = 0;
  }
  _prev_modnet_bterm = 0;
  mod_net->_bterms = getOID();
}

void _dbBTerm::connectNet(_dbNet* net, _dbBlock* block)
{
  _dbBTerm* bterm = (_dbBTerm*) this;

  if (block->_journal) {
    debugPrint(block->getImpl()->getLogger(),
               utl::ODB,
               "DB_ECO",
               1,
               "ECO: connect Bterm {} to net {}",
               bterm->getId(),
               net->getId());
    block->_journal->beginAction(dbJournal::CONNECT_OBJECT);
    block->_journal->pushParam(dbBTermObj);
    block->_journal->pushParam(bterm->getId());
    block->_journal->pushParam(net->getId());
    // modnet is left out, only flat net.
    block->_journal->pushParam(0U);
    block->_journal->endAction();
  }

  for (auto callback : block->_callbacks) {
    callback->inDbBTermPreConnect((dbBTerm*) this, (dbNet*) net);
  }
  _net = net->getOID();
  if (net->_bterms != 0) {
    _dbBTerm* tail = block->_bterm_tbl->getPtr(net->_bterms);
    _next_bterm = net->_bterms;
    tail->_prev_bterm = getOID();
  } else {
    _next_bterm = 0;
  }
  _prev_bterm = 0;
  net->_bterms = getOID();
  for (auto callback : block->_callbacks) {
    callback->inDbBTermPostConnect((dbBTerm*) this);
  }
}

void dbBTerm::destroy(dbBTerm* bterm_)
{
  _dbBTerm* bterm = (_dbBTerm*) bterm_;
  _dbBlock* block = (_dbBlock*) bterm->getOwner();

  if (bterm->_net) {
    _dbNet* net = block->_net_tbl->getPtr(bterm->_net);
    if (net->_flags._dont_touch) {
      net->getLogger()->error(utl::ODB,
                              374,
                              "Attempt to destroy bterm on dont_touch net {}",
                              net->_name);
    }
  }

  // delete bpins
  dbSet<dbBPin> bpins = bterm_->getBPins();
  dbSet<dbBPin>::iterator itr;

  for (itr = bpins.begin(); itr != bpins.end();) {
    itr = dbBPin::destroy(itr);
  }
  if (bterm->_net) {
    bterm->disconnectNet(bterm, block);
  }
  for (auto callback : block->_callbacks) {
    callback->inDbBTermDestroy(bterm_);
  }

  if (block->_journal) {
    debugPrint(block->getImpl()->getLogger(),
               utl::ODB,
               "DB_ECO",
               1,
               "ECO: dbBTerm:destroy");
    block->_journal->beginAction(dbJournal::DELETE_OBJECT);
    block->_journal->pushParam(dbBTermObj);
    block->_journal->pushParam(bterm_->getId());
    block->_journal->endAction();
  }

  block->_bterm_hash.remove(bterm);
  dbProperty::destroyProperties(bterm);
  block->_bterm_tbl->destroy(bterm);
}

void _dbBTerm::disconnectNet(_dbBTerm* bterm, _dbBlock* block)
{
  if (bterm->_net) {
    _dbNet* net = block->_net_tbl->getPtr(bterm->_net);

    // Journal
    if (block->_journal) {
      debugPrint(block->getImpl()->getLogger(),
                 utl::ODB,
                 "DB_ECO",
                 1,
                 "ECO: disconnect bterm {}",
                 bterm->getId());
      block->_journal->beginAction(dbJournal::DISCONNECT_OBJECT);
      block->_journal->pushParam(dbBTermObj);
      block->_journal->pushParam(bterm->getId());
      block->_journal->pushParam(net->getId());
      block->_journal->pushParam(0U);  // no modnet
      block->_journal->endAction();
    }

    // unlink bterm from the net
    for (auto callback : block->_callbacks) {
      callback->inDbBTermPreDisconnect((dbBTerm*) this);
    }

    uint id = bterm->getOID();

    if (net->_bterms == id) {
      net->_bterms = bterm->_next_bterm;

      if (net->_bterms != 0) {
        _dbBTerm* t = block->_bterm_tbl->getPtr(net->_bterms);
        t->_prev_bterm = 0;
      }
    } else {
      if (bterm->_next_bterm != 0) {
        _dbBTerm* next = block->_bterm_tbl->getPtr(bterm->_next_bterm);
        next->_prev_bterm = bterm->_prev_bterm;
      }

      if (bterm->_prev_bterm != 0) {
        _dbBTerm* prev = block->_bterm_tbl->getPtr(bterm->_prev_bterm);
        prev->_next_bterm = bterm->_next_bterm;
      }
    }
    _net = 0;
    for (auto callback : block->_callbacks) {
      callback->inDbBTermPostDisConnect((dbBTerm*) this, (dbNet*) net);
    }
  }
}

void _dbBTerm::disconnectModNet(_dbBTerm* bterm, _dbBlock* block)
{
  if (bterm->_mnet) {
    _dbModNet* mod_net = block->_modnet_tbl->getPtr(bterm->_mnet);

    if (block->_journal) {
      debugPrint(block->getImpl()->getLogger(),
                 utl::ODB,
                 "DB_ECO",
                 1,
                 "ECO: disconnect bterm {}",
                 bterm->getId());
      block->_journal->beginAction(dbJournal::DISCONNECT_OBJECT);
      block->_journal->pushParam(dbBTermObj);
      block->_journal->pushParam(bterm->getId());
      // we are not considering the dbNet
      block->_journal->pushParam(0U);
      block->_journal->pushParam(mod_net->getId());
      block->_journal->endAction();
    }

    uint id = bterm->getOID();
    if (mod_net->_bterms == id) {
      mod_net->_bterms = bterm->_next_modnet_bterm;
      if (mod_net->_bterms != 0) {
        _dbBTerm* t = block->_bterm_tbl->getPtr(mod_net->_bterms);
        t->_prev_modnet_bterm = 0;
      }
    } else {
      if (bterm->_next_modnet_bterm != 0) {
        _dbBTerm* next = block->_bterm_tbl->getPtr(bterm->_next_modnet_bterm);
        next->_prev_modnet_bterm = bterm->_prev_modnet_bterm;
      }
      if (bterm->_prev_modnet_bterm != 0) {
        _dbBTerm* prev = block->_bterm_tbl->getPtr(bterm->_prev_modnet_bterm);
        prev->_next_modnet_bterm = bterm->_next_modnet_bterm;
      }
    }
    _mnet = 0;
  }
}

void _dbBTerm::setMirroredConstraintRegion(const Rect& region, _dbBlock* block)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  const Rect& die_bounds = ((dbBlock*) block)->getDieArea();
  int begin = region.dx() == 0 ? region.yMin() : region.xMin();
  int end = region.dx() == 0 ? region.yMax() : region.xMax();
  Direction2D edge;
  if (region.dx() == 0) {
    edge = region.xMin() == die_bounds.xMin() ? west : east;
  } else {
    edge = region.yMin() == die_bounds.yMin() ? south : north;
  }
  const Rect mirrored_region
      = ((dbBlock*) block)->findConstraintRegion(edge, begin, end);
  bterm->_constraint_region = mirrored_region;
}

void _dbBTerm::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);

  info.children_["name"].add(_name);
}

dbSet<dbBTerm>::iterator dbBTerm::destroy(dbSet<dbBTerm>::iterator& itr)
{
  dbBTerm* bt = *itr;
  dbSet<dbBTerm>::iterator next = ++itr;
  destroy(bt);
  return next;
}

dbBTerm* dbBTerm::getBTerm(dbBlock* block_, uint oid)
{
  _dbBlock* block = (_dbBlock*) block_;
  return (dbBTerm*) block->_bterm_tbl->getPtr(oid);
}

uint32_t dbBTerm::staVertexId()
{
  _dbBTerm* iterm = (_dbBTerm*) this;
  return iterm->_sta_vertex_id;
}

void dbBTerm::staSetVertexId(uint32_t id)
{
  _dbBTerm* iterm = (_dbBTerm*) this;
  iterm->_sta_vertex_id = id;
}

void dbBTerm::setConstraintRegion(const Rect& constraint_region)
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  bterm->_constraint_region = constraint_region;

  dbBTerm* mirrored_bterm = getMirroredBTerm();
  if (mirrored_bterm != nullptr && !bterm->_constraint_region.isInverted()
      && mirrored_bterm->getConstraintRegion() == std::nullopt) {
    _dbBlock* block = (_dbBlock*) getBlock();
    _dbBTerm* mirrored = (_dbBTerm*) mirrored_bterm;
    mirrored->setMirroredConstraintRegion(bterm->_constraint_region, block);
  }
}

std::optional<Rect> dbBTerm::getConstraintRegion()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  const auto& constraint_region = bterm->_constraint_region;
  if (constraint_region.isInverted()) {
    return std::nullopt;
  }

  return bterm->_constraint_region;
}

void dbBTerm::resetConstraintRegion()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  bterm->_constraint_region.mergeInit();
}

void dbBTerm::setMirroredBTerm(dbBTerm* mirrored_bterm)
{
  _dbBTerm* bterm = (_dbBTerm*) this;

  bterm->_mirrored_bterm = mirrored_bterm->getImpl()->getOID();
  _dbBTerm* mirrored = (_dbBTerm*) mirrored_bterm;
  mirrored->_is_mirrored = true;
  mirrored->_mirrored_bterm = bterm->getImpl()->getOID();

  if (!bterm->_constraint_region.isInverted()
      && mirrored_bterm->getConstraintRegion() == std::nullopt) {
    _dbBlock* block = (_dbBlock*) getBlock();
    mirrored->setMirroredConstraintRegion(bterm->_constraint_region, block);
  } else if (mirrored_bterm->getConstraintRegion() != std::nullopt) {
    getImpl()->getLogger()->warn(utl::ODB,
                                 26,
                                 "Pin {} is mirrored with another pin. The "
                                 "constraint for this pin will be dropped.",
                                 mirrored_bterm->getName());
  }
}

dbBTerm* dbBTerm::getMirroredBTerm()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  _dbBlock* block = (_dbBlock*) getBlock();

  if (bterm->_mirrored_bterm == 0) {
    return nullptr;
  }

  _dbBTerm* mirrored_bterm = block->_bterm_tbl->getPtr(bterm->_mirrored_bterm);
  return (dbBTerm*) mirrored_bterm;
}

bool dbBTerm::hasMirroredBTerm()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return bterm->_mirrored_bterm != 0 && !bterm->_is_mirrored;
}

bool dbBTerm::isMirrored()
{
  _dbBTerm* bterm = (_dbBTerm*) this;
  return bterm->_is_mirrored;
}

}  // namespace odb
