// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// SdcHandler is declared in request_handler.h (consistent with other
// handlers). This header hosts the JSON-emission helpers shared by
// sdc_handler.cpp and cdc_handler.cpp — both produce the same wire
// shapes for ODB-pin references and rise/fall × min/max delay tuples,
// so the helpers live in one place. The STA / db_sta includes stay
// scoped to this header (rather than leaking into request_handler.h)
// so the non-SDC handlers don't pay compile-time cost for unused STA
// types.
//
// All helpers are `inline` so multiple TUs can include this header
// without ODR conflicts.

#pragma once

#include <string>

#include <boost/json.hpp>

#include "db_sta/dbNetwork.hh"
#include "sta/MinMax.hh"
#include "sta/NetworkClass.hh"
#include "sta/RiseFallMinMax.hh"

namespace web {

// Resolve a sta::Pin to its ODB handle and write {type, id} into the two
// out-params. Returns false if either input is null or the pin is unresolved.
inline bool resolvePinOdb(const sta::Pin* pin,
                          sta::dbNetwork* db_network,
                          const char*& out_type,
                          int& out_id)
{
  if (!pin || !db_network) {
    return false;
  }
  odb::dbITerm* iterm = nullptr;
  odb::dbBTerm* bterm = nullptr;
  odb::dbModITerm* moditerm = nullptr;
  db_network->staToDb(pin, iterm, bterm, moditerm);
  if (iterm) {
    out_type = "iterm";
    out_id = static_cast<int>(iterm->getId());
    return true;
  }
  if (bterm) {
    out_type = "bterm";
    out_id = static_cast<int>(bterm->getId());
    return true;
  }
  if (moditerm) {
    // Hierarchical (module-boundary) pin — also inspectable via kInspect
    // with odb_type=moditerm.
    out_type = "moditerm";
    out_id = static_cast<int>(moditerm->getId());
    return true;
  }
  return false;
}

// Emit "<prefix>_odb_type" / "<prefix>_odb_id" so the frontend can dispatch
// an inspect-by-ODB without a name-resolution round-trip. Emits nothing for
// unresolved pins; _linkifyPin on the frontend no-ops in that case.
inline void emitPinOdbRef(boost::json::object& obj,
                          const std::string& prefix,
                          const sta::Pin* pin,
                          sta::dbNetwork* db_network)
{
  const char* type = nullptr;
  int id = 0;
  if (resolvePinOdb(pin, db_network, type, id)) {
    obj[prefix + "_odb_type"] = std::string(type);
    obj[prefix + "_odb_id"] = id;
  }
}

// Bare-field variant of emitPinOdbRef for arrays of {name, odb_type, odb_id}.
inline void emitPinOdbBare(boost::json::object& obj,
                           const sta::Pin* pin,
                           sta::dbNetwork* db_network)
{
  const char* type = nullptr;
  int id = 0;
  if (resolvePinOdb(pin, db_network, type, id)) {
    obj["odb_type"] = std::string(type);
    obj["odb_id"] = id;
  }
}

// Emit bare "odb_type" / "odb_id" for an Instance: dbInst → "inst",
// dbModInst → "modinst".
inline void emitInstanceOdbBare(boost::json::object& obj,
                                const sta::Instance* inst,
                                sta::dbNetwork* db_network)
{
  if (!inst || !db_network) {
    return;
  }
  odb::dbInst* di = nullptr;
  odb::dbModInst* dmi = nullptr;
  db_network->staToDb(inst, di, dmi);
  if (di) {
    obj["odb_type"] = std::string("inst");
    obj["odb_id"] = static_cast<int>(di->getId());
  } else if (dmi) {
    obj["odb_type"] = std::string("modinst");
    obj["odb_id"] = static_cast<int>(dmi->getId());
  }
}

// Emit a RiseFallMinMax as four nullable JSON fields (rise_max, rise_min,
// fall_max, fall_min). When `rfmm` is null, every slot is emitted as null.
// `scale` divides the raw STA values to convert to display units.
inline void emitRiseFallMinMax(boost::json::object& obj,
                               const sta::RiseFallMinMax* rfmm,
                               float scale)
{
  const struct
  {
    const char* key;
    const sta::RiseFall* rf;
    const sta::MinMax* mm;
  } slots[] = {
      {"rise_max", sta::RiseFall::rise(), sta::MinMax::max()},
      {"rise_min", sta::RiseFall::rise(), sta::MinMax::min()},
      {"fall_max", sta::RiseFall::fall(), sta::MinMax::max()},
      {"fall_min", sta::RiseFall::fall(), sta::MinMax::min()},
  };
  for (const auto& s : slots) {
    float val;
    bool exists;
    if (rfmm) {
      rfmm->value(s.rf, s.mm, val, exists);
    } else {
      exists = false;
    }
    if (exists) {
      obj[s.key] = static_cast<double>(val / scale);
    } else {
      obj[s.key] = nullptr;
    }
  }
}

}  // namespace web
