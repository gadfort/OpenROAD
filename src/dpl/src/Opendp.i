/////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019, The Regents of the University of California
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

%{

#include "ord/OpenRoad.hh"
#include "Graphics.h"
#include "DplObserver.h"
#include "dpl/Opendp.h"
#include "utl/Logger.h"



namespace dpl {

using std::vector;

// Swig vector type in does not seem to work at all.
// (see odb/src/swig/common/polgon.i)
// Copied from opensta/tcl/StaTcl.i
template <class TYPE>
vector<TYPE> *
tclListSeq(Tcl_Obj *const source,
	   swig_type_info *swig_type,
	   Tcl_Interp *interp)
{
  int argc;
  Tcl_Obj **argv;

  if (Tcl_ListObjGetElements(interp, source, &argc, &argv) == TCL_OK
      && argc > 0) {
    vector<TYPE> *seq = new vector<TYPE>;
    for (int i = 0; i < argc; i++) {
      void *obj;
      // Ignore returned TCL_ERROR because can't get swig_type_info.
      SWIG_ConvertPtr(argv[i], &obj, swig_type, false);
      seq->push_back(reinterpret_cast<TYPE>(obj));
    }
    return seq;
  }
  else
    return nullptr;
}
  
dpl::dbMasterSeq *
tclListSeqdbMaster(Tcl_Obj *const source,
                   Tcl_Interp *interp)
{
  return tclListSeq<odb::dbMaster*>(source, SWIGTYPE_p_odb__dbMaster, interp);
}

}

%}

%include "../../Exception.i"

%typemap(in) dpl::dbMasterSeq * {
  $1 = dpl::tclListSeqdbMaster($input, interp);
}

%inline %{

namespace dpl {

void
detailed_placement_cmd(int max_displacment_x,
                       int max_displacment_y,
                       const char* report_file_name){
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->detailedPlacement(max_displacment_x, max_displacment_y, std::string(report_file_name));
}

void
report_legalization_stats()
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->reportLegalizationStats();
}

void
check_placement_cmd(bool verbose, const char* report_file_name)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->checkPlacement(verbose, std::string(report_file_name));
}


void
set_padding_global(int left,
                   int right)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setPaddingGlobal(left, right);
}

void
set_padding_master(odb::dbMaster *master,
                   int left,
                   int right)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setPadding(master, left, right);
}

void
set_padding_inst(odb::dbInst *inst,
                 int left,
                 int right)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setPadding(inst, left, right);
}

void
filler_placement_cmd(dpl::dbMasterSeq *filler_masters,
                     const char* prefix,
                     bool verbose)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->fillerPlacement(filler_masters, prefix, verbose);
}

void
remove_fillers_cmd()
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->removeFillers();
}

void
optimize_mirroring_cmd()
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->optimizeMirroring();
}

void
set_debug_cmd(float min_displacement,
              const odb::dbInst* debug_instance)
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  if (dpl::Graphics::guiActive()) {
      std::unique_ptr<DplObserver> graphics = std::make_unique<dpl::Graphics>(
          opendp, min_displacement, debug_instance);
      opendp->setDebug(graphics);
  }
}

} // namespace

%} // inline
