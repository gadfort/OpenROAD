// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "db_sta/dbReadVerilog.hh"

#include <odb/dbSet.h>

#include <cstddef>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "sta/ConcreteLibrary.hh"
#include "sta/ConcreteNetwork.hh"
#include "sta/NetworkCmp.hh"
#include "sta/PortDirection.hh"
#include "sta/Vector.hh"
#include "sta/VerilogReader.hh"
#include "utl/Logger.h"

namespace ord {

using odb::dbBlock;
using odb::dbBTerm;
using odb::dbBusPort;
using odb::dbChip;
using odb::dbDatabase;
using odb::dbInst;
using odb::dbIoType;
using odb::dbITerm;
using odb::dbMaster;
using odb::dbModBTerm;
using odb::dbModInst;
using odb::dbModITerm;
using odb::dbModNet;
using odb::dbModule;
using odb::dbMTerm;
using odb::dbNet;
using odb::dbTech;
using utl::ORD;

using sta::Cell;
using sta::CellPortBitIterator;
using sta::CellPortIterator;
using sta::ConcreteCell;
using sta::ConcreteCellPortIterator;
using sta::ConcretePort;
using sta::ConnectedPinIterator;
using sta::dbNetwork;
using sta::Instance;
using sta::InstanceChildIterator;
using sta::InstancePinIterator;
using sta::LeafInstanceIterator;
using sta::LibertyCell;
using sta::Net;
using sta::NetConnectedPinIterator;
using sta::NetIterator;
using sta::NetTermIterator;
using sta::Network;
using sta::Pin;
using sta::PinPathNameLess;
using sta::PinSeq;
using sta::Port;
using sta::PortDirection;
using sta::Term;
using sta::VerilogReader;
using utl::Logger;

dbVerilogNetwork::dbVerilogNetwork()
{
  report_ = nullptr;
  debug_ = nullptr;
}

void dbVerilogNetwork::init(dbNetwork* db_network)
{
  db_network_ = db_network;
  copyState(db_network_);
}

dbVerilogNetwork* makeDbVerilogNetwork()
{
  return new dbVerilogNetwork;
}

void initDbVerilogNetwork(dbVerilogNetwork* network, sta::dbSta* sta)
{
  network->init(sta->getDbNetwork());
}

void setDbNetworkLinkFunc(dbVerilogNetwork* network,
                          VerilogReader* verilog_reader)
{
  if (verilog_reader) {
    network->setLinkFunc(
        [=](const char* top_cell_name, bool make_black_boxes) -> Instance* {
          return verilog_reader->linkNetwork(
              top_cell_name,
              make_black_boxes,
              // don't delete modules after link so we can swap to
              // uninstantiated modules if needed
              false);
        });
  }
}

void deleteDbVerilogNetwork(dbVerilogNetwork* verilog_network)
{
  delete verilog_network;
}

// Facade that looks in the db network for a liberty cell if
// there isn't one in the verilog network.
Cell* dbVerilogNetwork::findAnyCell(const char* name)
{
  Cell* cell = ConcreteNetwork::findAnyCell(name);
  if (cell == nullptr) {
    cell = db_network_->findAnyCell(name);
  }
  return cell;
}

// Cell is a black box if all the ports have unknown port directions
bool dbVerilogNetwork::isBlackBox(ConcreteCell* cell)
{
  std::unique_ptr<ConcreteCellPortIterator> port_iter{cell->portIterator()};
  while (port_iter->hasNext()) {
    ConcretePort* port = port_iter->next();
    if (port->direction() != PortDirection::unknown()) {
      return false;
    }
  }
  return true;
}

class Verilog2db
{
 public:
  Verilog2db(Network* verilog_network,
             dbDatabase* db,
             Logger* logger,
             bool hierarchy,
             bool omit_filename_prop);
  void makeBlock();
  void makeUnusedBlock(const char* name);
  void makeDbNetlist();
  void makeUnusedDbNetlist();
  void processUnusedCells(const char* top_cell_name,
                          dbVerilogNetwork* verilog_network,
                          bool link_make_black_boxes);
  void restoreTopBlock(const char* orig_top_cell_name);

 private:
  struct LineInfo
  {
    std::string file_name;
    int line_number;
  };
  using InstPair = std::pair<const Instance*, dbModInst*>;
  using InstPairs = std::vector<InstPair>;
  void makeDbModule(Instance* inst, dbModule* parent, InstPairs& inst_pairs);
  void makeChildInsts(Instance* inst, dbModule* module, InstPairs& inst_pairs);
  void makeModBTerms(Cell* cell, dbModule* module);
  void makeModITerms(Instance* inst, dbModInst* modinst);
  dbIoType staToDb(PortDirection* dir);
  bool staToDb(dbModule* module,
               const Pin* pin,
               dbBTerm*& bterm,
               dbITerm*& iterm,
               dbModBTerm*& mod_bterm,
               dbModITerm*& mod_iterm);
  void recordBusPortsOrder();
  void makeDbNets(const Instance* inst);

  void makeVModNets(const Instance* inst, dbModInst* mod_inst);
  void makeVModNets(InstPairs& inst_pairs);
  dbModNet* constructModNet(Net* inst_pin_net, dbModule* module);

  bool hasTerminals(Net* net) const;
  dbMaster* getMaster(Cell* cell);
  void storeLineInfo(const std::string& attribute, dbInst* db_inst);
  void makeModNets(Instance* inst);

  Network* network_;
  dbDatabase* db_;
  dbBlock* block_ = nullptr;
  dbBlock* top_block_ = nullptr;
  Logger* logger_;
  std::map<Cell*, dbMaster*> master_map_;
  // Map file names to a unique id to avoid having to store the full file name
  // for each instance
  std::map<std::string, int> src_file_id_;
  // We have to store dont_touch instances and apply the attribute after
  // creating iterms; as iterms can't be added to a dont_touch inst
  std::vector<dbInst*> dont_touch_insts_;
  bool hierarchy_ = false;
  bool omit_filename_prop_ = false;
  static const std::regex kLineInfoRe;
  std::vector<ConcreteCell*> unused_cells_;
};

// Example: "./designs/src/gcd/gcd.v:571.3-577.6"
const std::regex Verilog2db::kLineInfoRe("^(.*):(\\d+)\\.\\d+-\\d+\\.\\d+$");

bool dbLinkDesign(const char* top_cell_name,
                  dbVerilogNetwork* verilog_network,
                  dbDatabase* db,
                  Logger* logger,
                  bool hierarchy,
                  bool omit_filename_prop)
{
  debugPrint(
      logger, utl::ODB, "dbReadVerilog", 1, "dbLinkDesign {}", top_cell_name);
  bool link_make_black_boxes = true;
  bool success = verilog_network->linkNetwork(
      top_cell_name, link_make_black_boxes, verilog_network->report());
  if (success) {
    Verilog2db v2db(verilog_network, db, logger, hierarchy, omit_filename_prop);
    v2db.makeBlock();
    v2db.makeDbNetlist();
    // Link unused modules in case if we want to swap to such modules later
    v2db.processUnusedCells(
        top_cell_name, verilog_network, link_make_black_boxes);
  }

  return success;
}

Verilog2db::Verilog2db(Network* network,
                       dbDatabase* db,
                       Logger* logger,
                       bool hierarchy,
                       bool omit_filename_prop)
    : network_(network),
      db_(db),
      logger_(logger),
      hierarchy_(hierarchy),
      omit_filename_prop_(omit_filename_prop)
{
}

void Verilog2db::makeBlock()
{
  dbChip* chip = db_->getChip();
  if (chip == nullptr) {
    chip = dbChip::create(db_);
  }
  block_ = chip->getBlock();
  if (block_) {
    // Delete existing db network objects.
    auto insts = block_->getInsts();
    for (auto iter = insts.begin(); iter != insts.end();) {
      iter = dbInst::destroy(iter);
    }
    auto nets = block_->getNets();
    for (auto iter = nets.begin(); iter != nets.end();) {
      iter = dbNet::destroy(iter);
    }
    auto bterms = block_->getBTerms();
    for (auto iter = bterms.begin(); iter != bterms.end();) {
      iter = dbBTerm::destroy(iter);
    }
    auto mod_insts = block_->getTopModule()->getChildren();
    for (auto iter = mod_insts.begin(); iter != mod_insts.end();) {
      iter = dbModInst::destroy(iter);
    }
  } else {
    const char* design
        = network_->name(network_->cell(network_->topInstance()));
    block_ = dbBlock::create(
        chip, design, db_->getTech(), network_->pathDivider());
  }
  dbTech* tech = db_->getTech();
  block_->setDefUnits(tech->getLefUnits());
  block_->setBusDelimiters('[', ']');
}

void Verilog2db::makeDbNetlist()
{
  recordBusPortsOrder();
  // As a side effect we accumulate the instance <-> modinst pairs
  InstPairs inst_pairs;
  makeDbModule(network_->topInstance(), /* parent */ nullptr, inst_pairs);
  makeDbNets(network_->topInstance());
  if (hierarchy_) {
    makeVModNets(inst_pairs);
  }
  for (auto inst : dont_touch_insts_) {
    inst->setDoNotTouch(true);
  }
}

void Verilog2db::recordBusPortsOrder()
{
  // OpenDB does not have any concept of bus ports.
  // Use a property to annotate the bus names as msb or lsb first for writing
  // verilog.
  Cell* top_cell = network_->cell(network_->topInstance());
  std::unique_ptr<CellPortIterator> bus_iter{network_->portIterator(top_cell)};
  while (bus_iter->hasNext()) {
    Port* port = bus_iter->next();
    if (network_->isBus(port)) {
      const char* port_name = network_->name(port);
      const char* cell_name = network_->name(top_cell);
      int from = network_->fromIndex(port);
      int to = network_->toIndex(port);
      std::string key
          = std::string("bus_msb_first ") + port_name + " " + cell_name;
      odb::dbBoolProperty::create(block_, key.c_str(), from > to);
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 1,
                 "Created bool prop {} {}",
                 key.c_str(),
                 from > to);
    }
  }
}

void Verilog2db::storeLineInfo(const std::string& attribute, dbInst* db_inst)
{
  if (attribute.empty()) {
    return;
  }

  std::smatch match;

  if (std::regex_match(attribute, match, kLineInfoRe)) {
    const std::string file_name = match[1];
    const auto iter = src_file_id_.find(file_name);
    int file_id;
    if (iter != src_file_id_.end()) {
      file_id = iter->second;
    } else {
      file_id = src_file_id_.size();
      src_file_id_[file_name] = file_id;
      const auto id_string = fmt::format("src_file_{}", file_id);
      odb::dbStringProperty::create(
          block_, id_string.c_str(), file_name.c_str());
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 1,
                 "Created string prop {} {}",
                 id_string.c_str(),
                 file_name.c_str());
    }
    odb::dbIntProperty::create(db_inst, "src_file_id", file_id);
    debugPrint(logger_,
               utl::ODB,
               "dbReadVerilog",
               1,
               "Created int prop src_file_id {}",
               file_id);
    odb::dbIntProperty::create(db_inst, "src_file_line", stoi(match[2]));
    debugPrint(logger_,
               utl::ODB,
               "dbReadVerilog",
               1,
               "Created int prop src_file_line {}",
               stoi(match[2]));
  }
}

// Recursively builds odb's dbModule/dbModInst hierarchy corresponding
// to the sta network rooted at inst.  parent is the dbModule to build
// the hierarchy under. If null the top module is used.

void Verilog2db::makeDbModule(
    Instance* inst,
    dbModule* parent,
    // harvest the hierarchical instances. Modnets connected to these
    InstPairs& inst_pairs)
{
  Cell* cell = network_->cell(inst);

  dbModule* module;
  if (parent == nullptr) {
    module = block_->getTopModule();
  } else {
    // This uniquifies the cell
    module = dbModule::makeUniqueDbModule(
        network_->name(cell), network_->name(inst), block_);

    std::string module_inst_name = network_->name(inst);

    dbModInst* modinst
        = dbModInst::create(parent, module, module_inst_name.c_str());

    inst_pairs.emplace_back(inst, modinst);

    // Verilog attribute is on a cell not an instance
    std::string impl_oper = network_->getAttribute(cell, "implements_operator");
    if (!impl_oper.empty()) {
      odb::dbStringProperty::create(
          modinst, "implements_operator", impl_oper.c_str());
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 1,
                 "Added implements_operator attribute to mod inst {}",
                 module_inst_name);
    }

    debugPrint(logger_,
               utl::ODB,
               "dbReadVerilog",
               1,
               "Created module instance {} in parent {} ",
               module_inst_name.c_str(),
               parent->getName());

    if (modinst == nullptr) {
      logger_->error(ORD,
                     2023,
                     "hierarchical instance creation failed for {} of {}",
                     network_->name(inst),
                     network_->name(cell));
    }
    if (hierarchy_) {
      makeModBTerms(cell, module);
      makeModITerms(inst, modinst);
    }
  }
  makeChildInsts(inst, module, inst_pairs);
}

void Verilog2db::makeModBTerms(Cell* cell, dbModule* module)
{
  dbBusPort* dbbusport = nullptr;
  // make the module ports
  std::unique_ptr<CellPortIterator> cp_iter{network_->portIterator(cell)};
  while (cp_iter->hasNext()) {
    Port* port = cp_iter->next();
    if (network_->isBus(port)) {
      // make the bus port as part of the port set for the cell.
      const char* port_name = network_->name(port);
      dbModBTerm* bmodterm = dbModBTerm::create(module, port_name);
      dbbusport = dbBusPort::create(module,
                                    bmodterm,  // the root of the bus port
                                    network_->fromIndex(port),
                                    network_->toIndex(port));
      bmodterm->setBusPort(dbbusport);
      const dbIoType io_type = staToDb(network_->direction(port));
      bmodterm->setIoType(io_type);

      //
      // Make a modbterm for each bus bit
      // Keep traversal in terms of bits
      // These modbterms are annotated as being
      // part of the port bus.
      //

      const int from_index = network_->fromIndex(port);
      const int to_index = network_->toIndex(port);
      const bool updown = (from_index <= to_index) ? true : false;
      const int size
          = updown ? to_index - from_index + 1 : from_index - to_index + 1;
      for (int i = 0; i < size; i++) {
        const int ix = updown ? from_index + i : from_index - i;
        const std::string bus_bit_port = port_name + std::string("[")
                                         + std::to_string(ix)
                                         + std::string("]");
        dbModBTerm* modbterm = dbModBTerm::create(module, bus_bit_port.c_str());
        if (i == 0) {
          dbbusport->setMembers(modbterm);
        }
        if (i == size - 1) {
          dbbusport->setLast(modbterm);
        }
        dbIoType io_type = staToDb(network_->direction(port));
        bmodterm->setIoType(io_type);
      }
    } else {
      const std::string port_name = network_->name(port);
      dbModBTerm* bmodterm = dbModBTerm::create(module, port_name.c_str());
      const dbIoType io_type = staToDb(network_->direction(port));
      bmodterm->setIoType(io_type);
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 1,
                 "Created module bterm {} ",
                 bmodterm->getName());
    }
  }
  module->getModBTerms().reverse();
}

void Verilog2db::makeModITerms(Instance* inst, dbModInst* modinst)
{
  // make the instance iterms and set up their reference
  // to the child ports (dbModBTerms).

  std::unique_ptr<InstancePinIterator> ip_iter(network_->pinIterator(inst));
  while (ip_iter->hasNext()) {
    Pin* cur_pin = ip_iter->next();
    const std::string pin_name_string = network_->portName(cur_pin);
    //
    // we do not need to store the pin names.. But they are
    // assumed to exist in the STA world.
    //

    dbModBTerm* modbterm;
    std::string port_name_str = pin_name_string;  // intentionally make copy
    const size_t last_idx = port_name_str.find_last_of('/');
    if (last_idx != std::string::npos) {
      port_name_str = port_name_str.substr(last_idx + 1);
    }
    dbModule* module = modinst->getMaster();
    modbterm = module->findModBTerm(port_name_str.c_str());
    // pass the modbterm into the moditerm creator
    // so that during journalling we keep the moditerm/modbterm correlation
    dbModITerm* moditerm
        = dbModITerm::create(modinst, pin_name_string.c_str(), modbterm);
    debugPrint(logger_,
               utl::ODB,
               "dbReadVerilog",
               1,
               "Created module iterm {} for bterm {}",
               moditerm->getName(),
               modbterm->getName());
  }
}

void Verilog2db::makeChildInsts(Instance* inst,
                                dbModule* module,
                                InstPairs& inst_pairs)
{
  std::unique_ptr<InstanceChildIterator> child_iter{
      network_->childIterator(inst)};
  while (child_iter->hasNext()) {
    Instance* child = child_iter->next();
    if (network_->isHierarchical(child)) {
      makeDbModule(child, module, inst_pairs);
    } else {
      const char* child_name = network_->pathName(child);
      Instance* parent_instance = network_->parent(child);
      dbModule* parent_module = nullptr;
      Cell* parent_cell = nullptr;
      if (parent_instance == network_->topInstance() || hierarchy_ == false) {
        parent_module = block_->getTopModule();
        parent_cell = network_->cell(parent_instance);
      } else {
        parent_cell = network_->cell(parent_instance);
        parent_module = block_->findModule(network_->name(parent_cell));
      }
      (void) parent_module;
      (void) parent_cell;
      Cell* cell = network_->cell(child);
      dbMaster* master = getMaster(cell);
      if (master == nullptr) {
        logger_->warn(ORD,
                      2013,
                      "instance {} LEF master {} not found.",
                      child_name,
                      network_->name(cell));
        continue;
      }

      auto db_inst = dbInst::create(block_, master, child_name, false, module);
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 2,
                 "Child inst {} created in makeChildInsts",
                 db_inst->getName());

      // Yosys writes a src attribute on sequential instances to give the
      // Verilog source info.
      if (!omit_filename_prop_) {
        storeLineInfo(network_->getAttribute(child, "src"), db_inst);
      }

      const auto dont_touch = network_->getAttribute(child, "dont_touch");
      if (!dont_touch.empty()) {
        if (std::stoi(dont_touch)) {
          dont_touch_insts_.push_back(db_inst);
        }
      }

      if (db_inst == nullptr) {
        logger_->error(ORD,
                       2015,
                       "Leaf instance creation failed for {} of {}",
                       network_->name(child),
                       module->getName());
      }
    }
  }

  if (module->getChildren().reversible()
      && module->getChildren().orderReversed()) {
    module->getChildren().reverse();
  }

  if (module->getInsts().reversible() && module->getInsts().orderReversed()) {
    module->getInsts().reverse();
  }
}

bool Verilog2db::staToDb(dbModule* module,
                         const Pin* pin,
                         dbBTerm*& bterm,
                         dbITerm*& iterm,
                         dbModBTerm*& mod_bterm,
                         dbModITerm*& mod_iterm)
{
  mod_bterm = nullptr;
  mod_iterm = nullptr;
  bterm = nullptr;
  iterm = nullptr;

  const char* port_name = network_->portName(pin);
  Instance* cur_inst = network_->instance(pin);
  std::string pin_name = network_->portName(pin);

  //
  // cases: All the things a pin could be:
  //
  // 1. A pin on a module instance (moditerm)
  // 2. A port on the top level (bterm)
  // 3. A pin on a dbInst (iterm)
  // 4. A port on a module (modbterm).
  //

  if (module) {
    if (cur_inst) {
      std::string instance_name = network_->pathName(cur_inst);
      size_t last_idx = instance_name.find_last_of('/');
      if (last_idx != std::string::npos) {
        instance_name = instance_name.substr(last_idx + 1);
      }
      dbModInst* mod_inst = module->findModInst(instance_name.c_str());
      if (mod_inst) {
        mod_iterm = mod_inst->findModITerm(pin_name.c_str());
      }
    }
  }

  if (!mod_iterm) {
    // a pin on the top level. Use the port name
    if (cur_inst == network_->topInstance()) {
      bterm = block_->findBTerm(port_name);
    } else {
      // a pin on an instance
      // we store just the pin name on the db inst iterm
      std::string instance_name = network_->pathName(cur_inst);
      size_t last_idx = pin_name.find_last_of('/');
      if (last_idx != std::string::npos) {
        pin_name = pin_name.substr(last_idx + 1);
      }
      // we store the full instance name for db insts
      dbInst* db_inst = module->findDbInst(instance_name.c_str());
      if (db_inst) {
        iterm = db_inst->findITerm(pin_name.c_str());
      } else {
        // a port on the module itself (a mod bterm)
        mod_bterm = module->findModBTerm(pin_name.c_str());
      }
    }
  }
  if (bterm || iterm || mod_iterm || mod_bterm) {
    return true;
  }
  return false;
}

dbIoType Verilog2db::staToDb(PortDirection* dir)
{
  if (dir == PortDirection::input()) {
    return dbIoType::INPUT;
  }
  if (dir == PortDirection::output()) {
    return dbIoType::OUTPUT;
  }
  if (dir == PortDirection::bidirect()) {
    return dbIoType::INOUT;
  }
  if (dir == PortDirection::tristate()) {
    return dbIoType::OUTPUT;
  }
  if (dir == PortDirection::unknown()) {
    return dbIoType::INPUT;
  }
  return dbIoType::INOUT;
}

void Verilog2db::makeDbNets(const Instance* inst)
{
  bool is_top = (inst == network_->topInstance());
  std::unique_ptr<NetIterator> net_iter{network_->netIterator(inst)};
  // Todo, put dbnets in the module in case of hierarchy (not block)
  while (net_iter->hasNext()) {
    Net* net = net_iter->next();

    if (!is_top && hasTerminals(net)) {
      continue;
    }

    const char* net_name = network_->pathName(net);
    dbNet* db_net = dbNet::create(block_, net_name);
    debugPrint(logger_,
               utl::ODB,
               "dbReadVerilog",
               2,
               "makeDbNets created net {}",
               db_net->getName());
    if (network_->isPower(net)) {
      db_net->setSigType(odb::dbSigType::POWER);
    }
    if (network_->isGround(net)) {
      db_net->setSigType(odb::dbSigType::GROUND);
    }

    // Sort connected pins for regression stability.
    PinSeq net_pins;
    std::unique_ptr<NetConnectedPinIterator> pin_iter{
        network_->connectedPinIterator(net)};
    while (pin_iter->hasNext()) {
      const Pin* pin = pin_iter->next();
      net_pins.push_back(pin);
    }
    sort(net_pins, PinPathNameLess(network_));

    for (const Pin* pin : net_pins) {
      if (network_->isTopLevelPort(pin)) {
        const char* port_name = network_->portName(pin);
        if (block_->findBTerm(port_name) == nullptr) {
          dbBTerm* bterm = dbBTerm::create(db_net, port_name);
          debugPrint(logger_,
                     utl::ODB,
                     "dbReadVerilog",
                     2,
                     "makeDbNets created bterm {}",
                     bterm->getName());
          dbIoType io_type = staToDb(network_->direction(pin));
          bterm->setIoType(io_type);
        }
      } else if (network_->isLeaf(pin)) {
        const char* port_name = network_->portName(pin);
        Instance* inst = network_->instance(pin);
        const char* inst_name = network_->pathName(inst);
        dbInst* db_inst = block_->findInst(inst_name);
        if (db_inst) {
          dbMaster* master = db_inst->getMaster();
          dbMTerm* mterm = master->findMTerm(block_, port_name);
          if (mterm) {
            db_inst->getITerm(mterm)->connect(db_net);
            debugPrint(logger_,
                       utl::ODB,
                       "dbReadVerilog",
                       2,
                       "makeDbNets connected mterm {} to net {}",
                       mterm->getName(),
                       db_net->getName());
          }
        }
      }
    }
  }

  std::unique_ptr<InstanceChildIterator> child_iter{
      network_->childIterator(inst)};
  while (child_iter->hasNext()) {
    const Instance* child = child_iter->next();
    makeDbNets(child);
  }
}

void Verilog2db::makeVModNets(InstPairs& inst_pairs)
{
  for (auto& [inst, modinst] : inst_pairs) {
    makeVModNets(inst, modinst);
  }
}

void Verilog2db::makeVModNets(const Instance* inst, dbModInst* mod_inst)
{
  // Given a hierarchical instance, get the pins on the outside
  // and the inside of the instance and construct the modnets

  debugPrint(logger_,
             utl::ODB,
             "dbReadVerilog",
             2,
             "makeVModNets inst: {} mod_inst: {}",
             network_->name(inst),
             mod_inst->getName());

  dbModule* parent_module = mod_inst->getParent();
  dbModule* child_module = mod_inst->getMaster();

  std::unique_ptr<InstancePinIterator> pin_iter{network_->pinIterator(inst)};
  while (pin_iter->hasNext()) {
    Pin* inst_pin = pin_iter->next();
    Net* inst_pin_net = network_->net(inst_pin);

    if (!inst_pin_net) {
      continue;
    }

    dbModNet* upper_mod_net = constructModNet(inst_pin_net, parent_module);

    dbModITerm* mod_iterm = nullptr;
    dbModBTerm* mod_bterm = nullptr;
    dbBTerm* bterm = nullptr;
    dbITerm* iterm = nullptr;
    staToDb(child_module, inst_pin, bterm, iterm, mod_bterm, mod_iterm);
    if (mod_bterm) {
      mod_iterm = mod_bterm->getParentModITerm();
      if (mod_iterm) {
        mod_iterm->connect(upper_mod_net);
        debugPrint(logger_,
                   utl::ODB,
                   "dbReadVerilog",
                   2,
                   "makeVModNets connected mod_iterm {} to upper_mod_net {}",
                   mod_iterm->getName(),
                   upper_mod_net->getName());
      }
    }

    // make sure any top level bterms are connected to this net too...
    if (parent_module == block_->getTopModule()) {
      NetConnectedPinIterator* pin_iter
          = network_->connectedPinIterator(inst_pin_net);
      while (pin_iter->hasNext()) {
        const Pin* pin = pin_iter->next();
        staToDb(parent_module, pin, bterm, iterm, mod_bterm, mod_iterm);
        if (bterm) {
          bterm->connect(upper_mod_net);
          debugPrint(logger_,
                     utl::ODB,
                     "dbReadVerilog",
                     2,
                     "makeVModNets connected bterm {} to upper_mod_net {}",
                     bterm->getName(),
                     upper_mod_net->getName());
        }
      }
    }

    // push down inside the hierarchical instance to find any
    // modnets connected on the inside of the instance
    Net* below_pin_net;
    Term* below_term = network_->term(inst_pin);
    if (below_term) {
      below_pin_net = network_->net(below_term);
      const char* below_net_name = network_->name(below_pin_net);
      if (child_module->getModNet(below_net_name)) {
        continue;
      }
      std::string pin_name = network_->name(below_term);
      size_t last_idx = pin_name.find_last_of('/');
      if (last_idx != std::string::npos) {
        pin_name = pin_name.substr(last_idx + 1);
      }
      dbModBTerm* mod_bterm = child_module->findModBTerm(pin_name.c_str());
      dbModNet* lower_mod_net = constructModNet(below_pin_net, child_module);
      mod_bterm->connect(lower_mod_net);
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 2,
                 "makeVModNets connected mod_bterm {} to lower_mod_net {}",
                 mod_bterm->getName(),
                 lower_mod_net->getName());
    }
  }
}

dbModNet* Verilog2db::constructModNet(Net* inst_pin_net, dbModule* module)
{
  dbModNet* db_mod_net = nullptr;

  std::unique_ptr<sta::NetPinIterator> npi{network_->pinIterator(inst_pin_net)};
  std::map<std::string, const sta::Pin*> net_pin_map;
  while (npi->hasNext()) {
    const sta::Pin* net_pin = npi->next();
    net_pin_map[network_->name(net_pin)] = net_pin;
  }

  const char* net_name = network_->name(inst_pin_net);
  db_mod_net = module->getModNet(net_name);
  if (!db_mod_net) {
    db_mod_net = dbModNet::create(module, net_name);
    debugPrint(logger_,
               utl::ODB,
               "dbReadVerilog",
               1,
               "created mod_net {} in module {}",
               net_name,
               module->getName());
  }
  for (auto& [name, pin] : net_pin_map) {
    dbITerm* iterm = nullptr;
    dbBTerm* bterm = nullptr;
    dbModITerm* mod_iterm = nullptr;
    dbModBTerm* mod_bterm = nullptr;
    // Make the connections to the mod net
    staToDb(module, pin, bterm, iterm, mod_bterm, mod_iterm);
    // leaf -> iterm
    // root -> bterm
    // instance -> moditerm
    // parent -> modbterm
    if (iterm) {
      iterm->connect(db_mod_net);
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 2,
                 "connected iterm {} to mod net {}",
                 iterm->getName(),
                 db_mod_net->getName());
    } else if (bterm) {
      bterm->connect(db_mod_net);
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 2,
                 "connected bterm {} to mod net {}",
                 bterm->getName(),
                 db_mod_net->getName());
    } else if (mod_bterm) {
      mod_bterm->connect(db_mod_net);
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 2,
                 "connected mod_bterm {} to mod net {}",
                 mod_bterm->getName(),
                 db_mod_net->getName());
    } else if (mod_iterm) {
      mod_iterm->connect(db_mod_net);
      debugPrint(logger_,
                 utl::ODB,
                 "dbReadVerilog",
                 2,
                 "connected mod_iterm {} to mod net {}",
                 mod_iterm->getName(),
                 db_mod_net->getName());
    }
  }
  return db_mod_net;
}

bool Verilog2db::hasTerminals(Net* net) const
{
  std::unique_ptr<NetTermIterator> term_iter{network_->termIterator(net)};
  return term_iter->hasNext();
}

dbMaster* Verilog2db::getMaster(Cell* cell)
{
  auto miter = master_map_.find(cell);
  if (miter != master_map_.end()) {
    return miter->second;
  }
  const char* cell_name = network_->name(cell);
  dbMaster* master = db_->findMaster(cell_name);
  if (master) {
    master_map_[cell] = master;
    // Check for corresponding liberty cell.
    LibertyCell* lib_cell = network_->libertyCell(cell);
    if (lib_cell == nullptr) {
      logger_->warn(ORD, 2011, "LEF master {} has no liberty cell.", cell_name);
    }
    return master;
  }
  LibertyCell* lib_cell = network_->libertyCell(cell);
  if (lib_cell) {
    logger_->warn(ORD, 2012, "Liberty cell {} has no LEF master.", cell_name);
  }
  // OpenSTA read_verilog warns about missing cells.
  master_map_[cell] = nullptr;
  return nullptr;
}

//
// Create top-level mod nets to connect boundary bterms and iterms
//
void Verilog2db::makeModNets(Instance* inst)
{
  dbModule* module = block_->getTopModule();
  std::unique_ptr<InstancePinIterator> pin_iter{network_->pinIterator(inst)};
  while (pin_iter->hasNext()) {
    Pin* inst_pin = pin_iter->next();
    debugPrint(logger_,
               utl::ODB,
               "dbReadVerilog",
               1,
               "makeModNets processing pin {}",
               network_->name(inst_pin));

    Net* below_pin_net;
    Term* below_term = network_->term(inst_pin);
    if (below_term) {
      below_pin_net = network_->net(below_term);
      if (below_pin_net) {
        const char* below_net_name = network_->name(below_pin_net);
        debugPrint(logger_,
                   utl::ODB,
                   "dbReadVerilog",
                   1,
                   "makeModNets below_net is {} for pin {}",
                   below_net_name,
                   network_->name(inst_pin));
        if (module->getModNet(below_net_name)) {
          debugPrint(logger_,
                     utl::ODB,
                     "dbReadVerilog",
                     1,
                     "makeModNets skips mod net creation for {} because it "
                     "already exists",
                     below_net_name);
          continue;
        }
        dbModBTerm* mod_bterm
            = module->findModBTerm(network_->name(below_term));
        dbModNet* lower_mod_net = constructModNet(below_pin_net, module);
        mod_bterm->connect(lower_mod_net);
        debugPrint(logger_,
                   utl::ODB,
                   "dbReadVerilog",
                   1,
                   "makeModNets connected mod_bterm {} to lower_mod_net {}",
                   mod_bterm->getName(),
                   lower_mod_net->getName());
      }
    }
  }
}

//
// Collect all unused modules such that they can be linked later
//
void Verilog2db::processUnusedCells(const char* top_cell_name,
                                    dbVerilogNetwork* verilog_network,
                                    bool link_make_black_boxes)
{
  // Collect all unused modules
  std::unique_ptr<sta::LibraryIterator> library_iterator{
      network_->libraryIterator()};
  while (library_iterator->hasNext()) {
    sta::ConcreteLibrary* lib
        = (sta::ConcreteLibrary*) (library_iterator->next());
    std::unique_ptr<sta::ConcreteLibraryCellIterator> lib_cell_iter{
        lib->cellIterator()};
    while (lib_cell_iter->hasNext()) {
      sta::ConcreteCell* curr_cell = lib_cell_iter->next();
      std::string impl_oper = curr_cell->getAttribute("implements_operator");
      if (!impl_oper.empty() && !block_->findModule(curr_cell->name())
          && !verilog_network->isBlackBox(curr_cell)) {
        unused_cells_.emplace_back(curr_cell);
        debugPrint(logger_,
                   utl::ODB,
                   "dbReadVerilog",
                   1,
                   "Found unused cell {}",
                   curr_cell->name());
      }
    }
  }

  // Link each unused module and populate content in a separate child block.
  // There will one child block for each unused module.
  for (ConcreteCell* cell : unused_cells_) {
    makeUnusedBlock(cell->name());
    debugPrint(logger_,
               utl::ODB,
               "dbReadVerilog",
               1,
               "Linking unused cell {}",
               cell->name());
    // It is important to use actual top cell name as top module name
    (void) verilog_network->linkNetwork(
        cell->name(), link_make_black_boxes, verilog_network->report());

    makeUnusedDbNetlist();
    if (logger_->debugCheck(utl::ODB, "dbReadVerilog", 1)) {
      std::string out_file_name
          = "child_block_" + std::string(cell->name()) + ".txt";
      std::ofstream out_file(out_file_name.c_str());
      block_->debugPrintContent(out_file);
    }
  }

  if (!unused_cells_.empty()) {
    restoreTopBlock(top_cell_name);
    if (logger_->debugCheck(utl::ODB, "dbReadVerilog", 1)) {
      std::ofstream out_file("top_block.txt");
      block_->debugPrintContent(out_file);
    }
  }
}

//
// makeUnusedBlock: create a separate block for each unused module
//
void Verilog2db::makeUnusedBlock(const char* name)
{
  dbChip* chip = db_->getChip();
  if (chip == nullptr) {
    chip = dbChip::create(db_);
  }
  // Create a child block
  if (top_block_ == nullptr) {
    top_block_ = chip->getBlock();
  }
  dbTech* tech = db_->getTech();
  block_ = dbBlock::create(top_block_, name, tech, network_->pathDivider());
  block_->setDefUnits(tech->getLefUnits());
  block_->setBusDelimiters('[', ']');
  debugPrint(logger_,
             utl::ODB,
             "dbReadVerilog",
             1,
             "Created child block {} under parent block {}",
             block_->getName(),
             top_block_->getName());
}

//
// makeUnusedDbNetlist: populate module content
//
void Verilog2db::makeUnusedDbNetlist()
{
  recordBusPortsOrder();
  Instance* inst = network_->topInstance();
  dbModule* module = block_->getTopModule();
  Cell* cell = network_->cell(inst);
  makeModBTerms(cell, module);
  InstPairs inst_pairs;
  makeChildInsts(inst, module, inst_pairs);
  makeDbNets(inst);
  // Create top-level mod nets
  makeModNets(inst);
  if (hierarchy_) {
    makeVModNets(inst_pairs);
  }
  for (auto inst : dont_touch_insts_) {
    inst->setDoNotTouch(true);
  }
}

//
// restoreTopBlock: restore original top cell and block
//
void Verilog2db::restoreTopBlock(const char* orig_top_cell_name)
{
  Instance* top_inst = network_->findInstance(orig_top_cell_name);
  ConcreteNetwork* cnetwork = static_cast<ConcreteNetwork*>(network_);
  cnetwork->setTopInstance(top_inst);
  block_ = top_block_;
}

}  // Namespace ord
