// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include <functional>
#include <string>
#include <vector>

#include "boostParser.h"
#include "lefLayerPropParser.h"
#include "odb/db.h"
#include "odb/lefin.h"
#include "parserUtils.h"

namespace odb {

lefTechLayerEolExtensionRuleParser::lefTechLayerEolExtensionRuleParser(
    lefinReader* l)
{
  lefin_ = l;
}

void lefTechLayerEolExtensionRuleParser::parse(const std::string& s,
                                               odb::dbTechLayer* layer)
{
  processRules(s, [this, layer](const std::string& rule) {
    if (!parseSubRule(rule, layer)) {
      lefin_->warning(260,
                      "parse mismatch in layer property "
                      "LEF58_EOLEXTENSIONSPACING for layer {} :\"{}\"",
                      layer->getName(),
                      rule);
    }
  });
}

void lefTechLayerEolExtensionRuleParser::setInt(
    double val,
    odb::dbTechLayerEolExtensionRule* rule,
    void (odb::dbTechLayerEolExtensionRule::*func)(int))
{
  (rule->*func)(lefin_->dbdist(val));
}
void lefTechLayerEolExtensionRuleParser::addEntry(
    boost::fusion::vector<double, double>& params,
    odb::dbTechLayerEolExtensionRule* rule)
{
  double eol = at_c<0>(params);
  double ext = at_c<1>(params);
  rule->addEntry(lefin_->dbdist(eol), lefin_->dbdist(ext));
}
bool lefTechLayerEolExtensionRuleParser::parseSubRule(const std::string& s,
                                                      odb::dbTechLayer* layer)
{
  odb::dbTechLayerEolExtensionRule* rule
      = odb::dbTechLayerEolExtensionRule::create(layer);

  qi::rule<std::string::const_iterator, space_type> EXTENSION_ENTRY
      = (lit("ENDOFLINE") >> double_ >> lit("EXTENSION")
         >> double_)[boost::bind(
          &lefTechLayerEolExtensionRuleParser::addEntry, this, _1, rule)];

  qi::rule<std::string::const_iterator, space_type> EOLEXTENSIONSPACING
      = (lit("EOLEXTENSIONSPACING")
         >> double_[boost::bind(&lefTechLayerEolExtensionRuleParser::setInt,
                                this,
                                _1,
                                rule,
                                &odb::dbTechLayerEolExtensionRule::setSpacing)]
         >> -lit("PARALLELONLY")[boost::bind(
             &odb::dbTechLayerEolExtensionRule::setParallelOnly, rule, true)]
         >> +EXTENSION_ENTRY >> lit(";"));
  auto first = s.begin();
  auto last = s.end();
  bool valid = qi::phrase_parse(first, last, EOLEXTENSIONSPACING, space)
               && first == last;
  if (!valid) {
    odb::dbTechLayerEolExtensionRule::destroy(rule);
  }
  return valid;
}

}  // namespace odb
