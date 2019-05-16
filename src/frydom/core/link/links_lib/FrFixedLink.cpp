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


#include "FrFixedLink.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"


namespace frydom {


    FrFixedLink::FrFixedLink(const std::shared_ptr <FrNode>& node1, const std::shared_ptr <FrNode>& node2, FrOffshoreSystem *system)
            : FrLink(node1, node2, system) {
        m_chronoLink->SetLinkType(FIXED_LINK);
    }

    std::shared_ptr<FrFixedLink> make_fixed_link(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem* system) {

        auto link = std::make_shared<FrFixedLink>(node1, node2, system);

        system->AddLink(link);

        return link;

    }
}  // end namespace frydom
