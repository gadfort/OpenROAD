// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <list>

#include "odb.h"

namespace odb {

class dbBlock;
class dbBox;
class dbFill;
class dbInst;
class dbMaster;
class dbNet;
class dbIoType;
class dbITerm;
class dbWire;
class dbBTerm;
class dbBPin;
class dbBlockage;
class dbPlacementStatus;
class dbObstruction;
class dbRegion;
class dbRow;
class dbSigType;
class dbSBox;
class dbSWire;
class dbMarker;
class dbMarkerCategory;

///////////////////////////////////////////////////////////////////////////////
///
/// dbBlockCallBackObj - An object comprising a list of stub routines
/// invoked by dbBlock.
/// Derived classes may implement these routines, causing external code
/// to be invoked from inside dbBlock methods.
///
///////////////////////////////////////////////////////////////////////////////

class dbBlockCallBackObj
{
 public:
  // dbInst Start
  virtual void inDbInstCreate(dbInst*) {}
  virtual void inDbInstCreate(dbInst*, dbRegion*) {}
  virtual void inDbInstDestroy(dbInst*) {}
  virtual void inDbInstPlacementStatusBefore(dbInst*, const dbPlacementStatus&)
  {
  }
  virtual void inDbInstSwapMasterBefore(dbInst*, dbMaster*) {}
  virtual void inDbInstSwapMasterAfter(dbInst*) {}
  virtual void inDbPreMoveInst(dbInst*) {}
  virtual void inDbPostMoveInst(dbInst*) {}
  // dbInst End

  // dbNet Start
  virtual void inDbNetCreate(dbNet*) {}
  virtual void inDbNetDestroy(dbNet*) {}
  virtual void inDbNetPreMerge(dbNet*, dbNet*) {}
  // dbNet End

  // dbITerm Start
  virtual void inDbITermCreate(dbITerm*) {}
  virtual void inDbITermDestroy(dbITerm*) {}  // Bugzilla #7 - payam
  virtual void inDbITermPreDisconnect(dbITerm*) {}
  virtual void inDbITermPostDisconnect(dbITerm*, dbNet*) {}
  virtual void inDbITermPreConnect(dbITerm*, dbNet*) {}
  virtual void inDbITermPostConnect(dbITerm*) {}
  // dbITerm End

  // dbBTerm Start
  virtual void inDbBTermCreate(dbBTerm*) {}
  virtual void inDbBTermDestroy(dbBTerm*) {}
  virtual void inDbBTermPreConnect(dbBTerm*, dbNet*) {}
  virtual void inDbBTermPostConnect(dbBTerm*) {}
  virtual void inDbBTermPreDisconnect(dbBTerm*) {}
  virtual void inDbBTermPostDisConnect(dbBTerm*, dbNet*) {}
  virtual void inDbBTermSetIoType(dbBTerm*, const dbIoType&) {}
  virtual void inDbBTermSetSigType(dbBTerm*, const dbSigType&) {}
  // dbBTerm End

  // dbBPin Start
  virtual void inDbBPinCreate(dbBPin*) {}
  virtual void inDbBPinDestroy(dbBPin*) {}
  // dbBPin End

  // dbBlockage Start
  virtual void inDbBlockageCreate(dbBlockage*) {}
  virtual void inDbBlockageDestroy(dbBlockage*) {}
  // dbBlockage End

  // dbObstruction Start
  virtual void inDbObstructionCreate(dbObstruction*) {}
  virtual void inDbObstructionDestroy(dbObstruction*) {}
  // dbObstruction End

  // dbRegion Start
  virtual void inDbRegionCreate(dbRegion*) {}
  virtual void inDbRegionAddBox(dbRegion*, dbBox*) {}
  virtual void inDbRegionDestroy(dbRegion*) {}
  // dbRegion End

  // dbRow Start
  virtual void inDbRowCreate(dbRow*) {}
  virtual void inDbRowDestroy(dbRow*) {}
  // dbRow End

  // dbWire Start
  virtual void inDbWireCreate(dbWire*) {}
  virtual void inDbWireDestroy(dbWire*) {}
  virtual void inDbWirePostModify(dbWire*) {}
  virtual void inDbWirePreAttach(dbWire*, dbNet*) {}
  virtual void inDbWirePostAttach(dbWire*) {}
  virtual void inDbWirePreDetach(dbWire*) {}
  virtual void inDbWirePostDetach(dbWire*, dbNet*) {}
  virtual void inDbWirePreAppend(dbWire*, dbWire*) {
  }  // first is src, second is dst
  virtual void inDbWirePostAppend(dbWire*, dbWire*) {
  }  // first is src, second is dst
  virtual void inDbWirePreCopy(dbWire*, dbWire*) {
  }  // first is src, second is dst
  virtual void inDbWirePostCopy(dbWire*, dbWire*) {
  }  // first is src, second is dst
  // dbWire End

  // dbSWire Start
  virtual void inDbSWireCreate(dbSWire*) {}
  virtual void inDbSWireDestroy(dbSWire*) {}
  virtual void inDbSWireAddSBox(dbSBox*) {}
  virtual void inDbSWireRemoveSBox(dbSBox*) {}
  virtual void inDbSWirePreDestroySBoxes(dbSWire*) {}
  virtual void inDbSWirePostDestroySBoxes(dbSWire*) {}
  // dbSWire End

  // dbFill Start
  virtual void inDbFillCreate(dbFill*) {}
  // dbFill End

  // dbMarkerCategory Start
  virtual void inDbMarkerCategoryCreate(dbMarkerCategory*) {}
  virtual void inDbMarkerCategoryDestroy(dbMarkerCategory*) {}
  // dbMarkerCategory End

  // dbMarker Start
  virtual void inDbMarkerCreate(dbMarker*) {}
  virtual void inDbMarkerDestroy(dbMarker*) {}
  // dbMarker End

  virtual void inDbBlockStreamOutBefore(dbBlock*) {}
  virtual void inDbBlockStreamOutAfter(dbBlock*) {}
  virtual void inDbBlockReadNetsBefore(dbBlock*) {}
  virtual void inDbBlockSetDieArea(dbBlock*) {}

  // allow ECO client initialization - payam
  virtual dbBlockCallBackObj& operator()() { return *this; }

  // Manipulate _callback list of owner -- in journal.cpp
  void addOwner(dbBlock* new_owner);
  bool hasOwner() const { return (_owner != nullptr); }
  void removeOwner();

  dbBlockCallBackObj() { _owner = nullptr; }
  virtual ~dbBlockCallBackObj() { removeOwner(); }

 private:
  dbBlock* _owner;
};

}  // namespace odb
