// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#include "FrCatenaryForce_ee444.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  FrCatenaryForce_ee444::FrCatenaryForce_ee444(const std::string &name, FrBody *body, FrCatenaryLine_ee444 *line,
                                   FrCatenaryLine_ee444::LINE_SIDE side) :
      FrForce(name, TypeToString(this), body),
      m_line(line),
      m_line_side(side) {}

  bool FrCatenaryForce_ee444::IncludedInStaticAnalysis() const { return true; }

  void FrCatenaryForce_ee444::Compute(double time) {

    Position relpos;
    Force ForceInWorld;

    // Get the line tension from the corresponding node
    switch (m_line_side) {
      case FrCatenaryLine_ee444::LINE_START:
        ForceInWorld = m_line->GetStartingNodeTension(NWU);
        relpos = m_line->GetStartingNode()->GetNodePositionInBody(NWU);
        break;

      case FrCatenaryLine_ee444::LINE_END:
        ForceInWorld = m_line->GetEndingNodeTension(NWU);
        relpos = m_line->GetEndingNode()->GetNodePositionInBody(NWU);
        break;
    }

    // Set the tension in the world reference frame and NWU frame convention
    SetForceTorqueInWorldAtPointInBody(ForceInWorld, Torque(), relpos, NWU);

  }


}  // end namespace frydom
