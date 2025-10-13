// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <cstdlib>

#include "dbBlock.h"
#include "dbCommon.h"
#include "dbCore.h"
#include "dbDatabase.h"
#include "odb/dbId.h"
#include "odb/dbTypes.h"
#include "odb/odb.h"

namespace odb {

class dbIStream;
class dbOStream;
class dbSite;
class dbLib;
class _dbSite;
class _dbLib;

struct dbRowFlags
{
  dbOrientType::Value _orient : 4;
  dbRowDir::Value _dir : 2;
  uint _spare_bits : 26;
};

class _dbRow : public _dbObject
{
 public:
  // PERSISTANT-MEMBERS
  dbRowFlags _flags;
  char* _name;
  dbId<_dbLib> _lib;
  dbId<_dbSite> _site;
  int _x;
  int _y;
  int _site_cnt;
  int _spacing;
  dbId<_dbRow> _next_entry;

  _dbRow(_dbDatabase*, const _dbRow& r);
  _dbRow(_dbDatabase*);
  ~_dbRow();

  bool operator==(const _dbRow& rhs) const;
  bool operator!=(const _dbRow& rhs) const { return !operator==(rhs); }
  bool operator<(const _dbRow& rhs) const;
  void collectMemInfo(MemInfo& info);
};

inline _dbRow::_dbRow(_dbDatabase*, const _dbRow& r)
    : _flags(r._flags),
      _name(nullptr),
      _lib(r._lib),
      _site(r._site),
      _x(r._x),
      _y(r._y),
      _site_cnt(r._site_cnt),
      _spacing(r._spacing),
      _next_entry(r._next_entry)
{
  if (r._name) {
    _name = safe_strdup(r._name);
  }
}

inline _dbRow::_dbRow(_dbDatabase*)
{
  _flags._orient = dbOrientType::R0;
  _flags._dir = dbRowDir::HORIZONTAL;
  _flags._spare_bits = 0;
  _name = nullptr;
  _x = 0;
  _y = 0;
  _site_cnt = 0;
  _spacing = 0;
  _next_entry = 0;
}

inline _dbRow::~_dbRow()
{
  if (_name) {
    free((void*) _name);
  }
}

inline dbOStream& operator<<(dbOStream& stream, const _dbRow& row)
{
  uint* bit_field = (uint*) &row._flags;
  stream << *bit_field;
  stream << row._name;
  stream << row._lib;
  stream << row._site;
  stream << row._x;
  stream << row._y;
  stream << row._site_cnt;
  stream << row._spacing;
  stream << row._next_entry;

  return stream;
}

inline dbIStream& operator>>(dbIStream& stream, _dbRow& row)
{
  _dbDatabase* db = row.getImpl()->getDatabase();

  uint* bit_field = (uint*) &row._flags;
  stream >> *bit_field;
  stream >> row._name;
  stream >> row._lib;
  stream >> row._site;
  stream >> row._x;
  stream >> row._y;
  stream >> row._site_cnt;
  stream >> row._spacing;
  if (db->isSchema(db_schema_row_hash_table)) {
    stream >> row._next_entry;
  } else {
    _dbBlock* owner = (_dbBlock*) row.getOwner();
    owner->_row_hash.insert(&row);
  }

  return stream;
}

}  // namespace odb
