//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRDYNAMICCABLE_H
#define FRYDOM_FRDYNAMICCABLE_H

#include <memory>
#include <frydom/core/FrNode.h>
#include <chrono_fea/ChVisualizationFEAmesh.h>
#include <frydom/cable/FrCatenaryLine.h>

#include "chrono_fea/ChBeamSection.h"
#include "chrono_fea/ChElementCableANCF.h"
#include "FrCable.h"


// TODO: harmoniser entre le cable catenaire et le cable dynamique !!!
// TODO: faire une classe abstraite de base pour les cables afin d'harmoniser les methodes

// TODO: mettre en place un (de)raffinement automatique...

using namespace chrono::fea;

namespace frydom {

    class FrDynamicCable : public ChMesh, public FrCable {

    private:

        std::shared_ptr<ChBeamSectionCable> m_section;

        double m_rayleighDamping;   ///< Rayleigh damping
        unsigned int m_nbElements;  ///< Number of elements in the finite element cable model

    public:
        FrDynamicCable() = default;

        // TODO: faire constructeur par copie

        void SetRayleighDamping(const double damping) { m_rayleighDamping = damping; }

        double GetRayleighDamping() const { return m_rayleighDamping; }

        void SetNumberOfElements(const unsigned int nbElements) { m_nbElements = nbElements; }

        unsigned int GetNumberOfElements() const {}

        void SetTargetElementLength() {
            // TODO: on donne une longueur cible d'element fini et ca calcule une discretisation spatiale basee sur la
            // longueur du cable ainsi qu'un nombre subsequent d'elements
        }

        std::shared_ptr<FrForce> GetStartingForce() const {}

        std::shared_ptr<FrForce> GetEndingForce() const {}

        chrono::ChVector<double> GetTension(const double s) const {}

        chrono::ChVector<double> GetAbsPosition(const double s) const {}

        chrono::ChVector<double> GetStartingNodeTension() const {}

        chrono::ChVector<double> GetEndingNodeTension() const {}


        /// Initialize the cable with given data
        void Initialize() {
//            m_mesh = std::make_shared<ChMesh>();

            m_section = std::make_shared<ChBeamSectionCable>();
            m_section->SetArea(m_sectionArea);
            m_section->SetBeamRaleyghDamping(m_rayleighDamping);
            m_section->SetDensity(GetDensity());
            m_section->SetYoungModulus(m_youngModulus);
            m_section->SetI(1.); // TODO: ajouter une inertie !!!

        /// Initialize the cable with given data
        void Initialize() {
            // First, creating a catenary line to initialize finite element mesh node positions
            // TODO: comment on definit q ???
            double q = 600;
            auto catenary_line = FrCatenaryLine(m_starting_node,
                                                m_ending_node,
                                                true,
                                                m_youngModulus * m_sectionArea,
                                                m_cableLength,
                                                q,
                                                chrono::ChVector<double>(0, 0, -1));

            // Now, creating the nodes
            double s = 0.;
            double ds = m_cableLength / m_nbElements;

            auto direction = catenary_line.GetTension(s);
            direction.Normalize();
            auto position = m_starting_node->GetAbsPos();
            auto nodeA = std::make_shared<ChNodeFEAxyzD>(position, direction);
            AddNode(nodeA);

            // Creating the specified number of ANCF Cable elements
            for (uint i = 1; i<= m_nbElements; ++i) {
                s += ds;

                direction = catenary_line.GetTension(s);
                direction.Normalize();
                position = catenary_line.GetAbsPosition(s);

                auto nodeB = std::make_shared<ChNodeFEAxyzD>(position, direction);
                AddNode(nodeB);

                auto element = std::make_shared<ChElementCableANCF>();
                element->SetNodes(nodeA, nodeB);
                element->SetSection(m_section);
                AddElement(element);

                nodeA = nodeB;

        }

            // Removing forces from catenary line that have been automatically created at instanciation
            m_starting_node->GetBody()->RemoveForce(catenary_line.GetStartingForce());
            m_ending_node->GetBody()->RemoveForce(catenary_line.GetEndingForce());

        }



//        void UpdateTime(double time) {
//            m_time = time;
//        }
//
//        void UpdateState() {
//
//        }

    };

}

#endif //FRYDOM_FRDYNAMICCABLE_H
