// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

// Generator Code Begin Cpp
#include "dbPowerSwitch.h"

#include <map>
#include <string>

#include "dbBlock.h"
#include "dbDatabase.h"
#include "dbHashTable.hpp"
#include "dbMTerm.h"
#include "dbMaster.h"
#include "dbNet.h"
#include "dbPowerDomain.h"
#include "dbTable.h"
#include "dbTable.hpp"
#include "odb/db.h"
#include "utl/Logger.h"
namespace odb {
template class dbTable<_dbPowerSwitch>;

bool _dbPowerSwitch::operator==(const _dbPowerSwitch& rhs) const
{
  if (_name != rhs._name) {
    return false;
  }
  if (_next_entry != rhs._next_entry) {
    return false;
  }
  if (_lib_cell != rhs._lib_cell) {
    return false;
  }
  if (_lib != rhs._lib) {
    return false;
  }
  if (_power_domain != rhs._power_domain) {
    return false;
  }

  return true;
}

bool _dbPowerSwitch::operator<(const _dbPowerSwitch& rhs) const
{
  return true;
}

_dbPowerSwitch::_dbPowerSwitch(_dbDatabase* db)
{
  _name = nullptr;
}

dbIStream& operator>>(dbIStream& stream, dbPowerSwitch::UPFIOSupplyPort& obj)
{
  stream >> obj.port_name;
  stream >> obj.supply_net_name;
  return stream;
}
dbIStream& operator>>(dbIStream& stream, dbPowerSwitch::UPFControlPort& obj)
{
  stream >> obj.port_name;
  stream >> obj.net_name;
  return stream;
}
dbIStream& operator>>(dbIStream& stream, dbPowerSwitch::UPFAcknowledgePort& obj)
{
  stream >> obj.port_name;
  stream >> obj.net_name;
  stream >> obj.boolean_expression;
  return stream;
}
dbIStream& operator>>(dbIStream& stream, dbPowerSwitch::UPFOnState& obj)
{
  stream >> obj.state_name;
  stream >> obj.input_supply_port;
  stream >> obj.boolean_expression;
  return stream;
}
dbIStream& operator>>(dbIStream& stream, _dbPowerSwitch& obj)
{
  stream >> obj._name;
  stream >> obj._next_entry;
  // User Code Begin >>
  if (obj.getDatabase()->isSchema(db_schema_update_db_power_switch)) {
    stream >> obj._in_supply_port;
    stream >> obj._out_supply_port;
    stream >> obj._control_port;
    stream >> obj._acknowledge_port;
    stream >> obj._on_state;
  } else {
    dbVector<std::string> in_supply_port;
    stream >> in_supply_port;
    for (const auto& port : in_supply_port) {
      obj._in_supply_port.emplace_back(
          dbPowerSwitch::UPFIOSupplyPort{port, ""});
    }
    dbVector<std::string> out_supply_port;
    stream >> out_supply_port;
    if (!out_supply_port.empty()) {
      obj._out_supply_port.port_name = out_supply_port[0];
      obj._out_supply_port.supply_net_name = "";
    }
    dbVector<std::string> control_port;
    stream >> control_port;
    for (const auto& port : control_port) {
      obj._control_port.emplace_back(dbPowerSwitch::UPFControlPort{port, ""});
    }
    dbVector<std::string> on_state;
    stream >> on_state;
    for (const auto& state : on_state) {
      obj._on_state.emplace_back(dbPowerSwitch::UPFOnState{state, "", ""});
    }
    dbId<_dbNet> net;
    stream >> net;  // unused
  }
  stream >> obj._power_domain;
  if (obj.getDatabase()->isSchema(db_schema_upf_power_switch_mapping)) {
    stream >> obj._lib_cell;
    stream >> obj._lib;
    stream >> obj._port_map;
  }
  // User Code End >>
  return stream;
}

dbOStream& operator<<(dbOStream& stream,
                      const dbPowerSwitch::UPFIOSupplyPort& obj)
{
  stream << obj.port_name;
  stream << obj.supply_net_name;
  return stream;
}
dbOStream& operator<<(dbOStream& stream,
                      const dbPowerSwitch::UPFControlPort& obj)
{
  stream << obj.port_name;
  stream << obj.net_name;
  return stream;
}
dbOStream& operator<<(dbOStream& stream,
                      const dbPowerSwitch::UPFAcknowledgePort& obj)
{
  stream << obj.port_name;
  stream << obj.net_name;
  stream << obj.boolean_expression;
  return stream;
}
dbOStream& operator<<(dbOStream& stream, const dbPowerSwitch::UPFOnState& obj)
{
  stream << obj.state_name;
  stream << obj.input_supply_port;
  stream << obj.boolean_expression;
  return stream;
}
dbOStream& operator<<(dbOStream& stream, const _dbPowerSwitch& obj)
{
  stream << obj._name;
  stream << obj._next_entry;
  // User Code Begin <<
  stream << obj._in_supply_port;
  stream << obj._out_supply_port;
  stream << obj._control_port;
  stream << obj._acknowledge_port;
  stream << obj._on_state;
  stream << obj._power_domain;
  stream << obj._lib_cell;
  stream << obj._lib;
  stream << obj._port_map;
  // User Code End <<
  return stream;
}

void _dbPowerSwitch::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);

  // User Code Begin collectMemInfo
  info.children_["name"].add(_name);
  info.children_["in_supply_port"].add(_in_supply_port);
  info.children_["control_port"].add(_control_port);
  info.children_["acknowledge_port"].add(_acknowledge_port);
  info.children_["on_state"].add(_on_state);
  info.children_["port_map"].add(_port_map);
  // User Code End collectMemInfo
}

_dbPowerSwitch::~_dbPowerSwitch()
{
  if (_name) {
    free((void*) _name);
  }
}

////////////////////////////////////////////////////////////////////
//
// dbPowerSwitch - Methods
//
////////////////////////////////////////////////////////////////////

const char* dbPowerSwitch::getName() const
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  return obj->_name;
}

void dbPowerSwitch::setPowerDomain(dbPowerDomain* power_domain)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;

  obj->_power_domain = power_domain->getImpl()->getOID();
}

dbPowerDomain* dbPowerSwitch::getPowerDomain() const
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  if (obj->_power_domain == 0) {
    return nullptr;
  }
  _dbBlock* par = (_dbBlock*) obj->getOwner();
  return (dbPowerDomain*) par->_powerdomain_tbl->getPtr(obj->_power_domain);
}

// User Code Begin dbPowerSwitchPublicMethods
dbPowerSwitch* dbPowerSwitch::create(dbBlock* block, const char* name)
{
  _dbBlock* _block = (_dbBlock*) block;
  if (_block->_powerswitch_hash.hasMember(name)) {
    return nullptr;
  }
  _dbPowerSwitch* ps = _block->_powerswitch_tbl->create();
  ps->_name = safe_strdup(name);

  _block->_powerswitch_hash.insert(ps);
  return (dbPowerSwitch*) ps;
}

void dbPowerSwitch::destroy(dbPowerSwitch* ps)
{
  // TODO
}

void dbPowerSwitch::addInSupplyPort(const std::string& in_port,
                                    const std::string& net)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  obj->_in_supply_port.emplace_back(UPFIOSupplyPort{in_port, net});
}

void dbPowerSwitch::setOutSupplyPort(const std::string& out_port,
                                     const std::string& net)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  obj->_out_supply_port.port_name = out_port;
  obj->_out_supply_port.supply_net_name = net;
}

void dbPowerSwitch::addControlPort(const std::string& control_port,
                                   const std::string& control_net)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  obj->_control_port.emplace_back(UPFControlPort{control_port, control_net});
}

void dbPowerSwitch::addAcknowledgePort(const std::string& port_name,
                                       const std::string& net_name,
                                       const std::string& boolean_expression)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  obj->_acknowledge_port.emplace_back(
      UPFAcknowledgePort{port_name, net_name, boolean_expression});
}

void dbPowerSwitch::addOnState(const std::string& on_state,
                               const std::string& port_name,
                               const std::string& boolean_expression)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  obj->_on_state.emplace_back(
      UPFOnState{on_state, port_name, boolean_expression});
}

void dbPowerSwitch::setLibCell(dbMaster* master)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  obj->_lib_cell = master->getImpl()->getOID();
  obj->_lib = master->getLib()->getImpl()->getOID();
}

void dbPowerSwitch::addPortMap(const std::string& model_port,
                               const std::string& switch_port)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  dbMaster* master = getLibCell();

  auto logger = obj->getImpl()->getLogger();
  if (!master) {
    logger->error(utl::ODB,
                  153,
                  "Cannot map port {} to {} because no lib cell is added",
                  model_port,
                  switch_port);
    return;
  }

  dbMTerm* mterm = master->findMTerm(switch_port.c_str());
  if (mterm == nullptr) {
    logger->error(utl::ODB,
                  154,
                  "Cannot map port {} to {} because the mterm is not found",
                  model_port,
                  switch_port);
    return;
  }
  obj->_port_map[model_port] = mterm->getImpl()->getOID();
}

void dbPowerSwitch::addPortMap(const std::string& model_port, dbMTerm* mterm)
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;

  dbMaster* master = getLibCell();

  if (!master) {
    obj->getImpl()->getLogger()->error(
        utl::ODB,
        219,
        "Cannot map port {} to {} because no lib cell is added",
        model_port,
        mterm->getName());
    return;
  }

  if (master != mterm->getMaster()) {
    obj->getImpl()->getLogger()->error(
        utl::ODB,
        220,
        "Cannot map port {} to {} because the mterm is not in the same master",
        model_port,
        mterm->getName());
    return;
  }

  obj->_port_map[model_port] = mterm->getImpl()->getOID();
}

std::vector<dbPowerSwitch::UPFControlPort> dbPowerSwitch::getControlPorts()
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  return obj->_control_port;
}

std::vector<dbPowerSwitch::UPFIOSupplyPort> dbPowerSwitch::getInputSupplyPorts()
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  return obj->_in_supply_port;
}

dbPowerSwitch::UPFIOSupplyPort dbPowerSwitch::getOutputSupplyPort()
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  return UPFIOSupplyPort{obj->_out_supply_port};
}

std::vector<dbPowerSwitch::UPFAcknowledgePort>
dbPowerSwitch::getAcknowledgePorts()
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  return obj->_acknowledge_port;
}

std::vector<dbPowerSwitch::UPFOnState> dbPowerSwitch::getOnStates()
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  return obj->_on_state;
}

dbMaster* dbPowerSwitch::getLibCell()
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  return (dbMaster*) dbMaster::getMaster(
      dbLib::getLib((dbDatabase*) obj->getImpl()->getDatabase(), obj->_lib),
      obj->_lib_cell);
}

std::map<std::string, dbMTerm*> dbPowerSwitch::getPortMap()
{
  _dbPowerSwitch* obj = (_dbPowerSwitch*) this;
  std::map<std::string, dbMTerm*> port_mapping;

  dbMaster* cell = getLibCell();
  if (!cell) {
    return port_mapping;
  }

  for (auto const& [key, val] : obj->_port_map) {
    dbMTerm* mterm = (dbMTerm*) dbMTerm::getMTerm(cell, val);
    port_mapping[key] = mterm;
  }

  return port_mapping;
}
// User Code End dbPowerSwitchPublicMethods
}  // namespace odb
   // Generator Code End Cpp
