//
// Created by frongere on 05/10/18.
//

#include <frydom/core/FrException.h>
#include "FrCatway.h"
#include "catenary/Catenary.h"

#include "frydom/core/FrNode.h"
#include "frydom/core/FrForce.h"
#include "frydom/core/FrBody.h"


namespace frydom {


    namespace internal {

        _CatenaryBase::_CatenaryBase(FrCatway* frydomCatLine,
                                     std::shared_ptr<catenary::CatenaryProperties> properties,
                                     std::shared_ptr<catenary::CatenaryNode> startNode,
                                     std::shared_ptr<catenary::CatenaryNode> endNode,
                                     const double unstretchedLength)
                                         : chrono::ChPhysicsItem(),
                                           catenary::CatenaryLine(properties, startNode, endNode, unstretchedLength),
                                           m_frydomCatLine(frydomCatLine)
                                           {}

        void _CatenaryBase::Update(double time, bool update_assets) {
            m_frydomCatLine->Update();
            chrono::ChPhysicsItem::Update(time, update_assets);
        }

        void _CatenaryBase::SetupInitial() {
            m_frydomCatLine->Initialize();
        }

        double _CatenaryBase::GetBreakingTension() {
            return m_breakingTension;
        }


        _CatenaryNode::_CatenaryNode(frydom::FrCatForce *force) : catenary::CatenaryNode(), m_force(force) {}

        void _CatenaryNode::Update(bool reverse) {

            // Updating forces
            auto tension = (Force)GetForceBalance();  // Returns the total force on the node as applied by the different connected catenary elements

//            if (reverse) tension = -tension;

            m_force->SetAbsTension(tension);




        }


    }  // end namespace internal




    FrCatway::FrCatway(double youngModulus, double diameter, double linearDensity, double length,
                       unsigned int nbElt, std::shared_ptr<FrNode_> startNode, std::shared_ptr<FrNode_> endNode) :
                       FrCable_(startNode, endNode, length, youngModulus, MU_PI * pow(0.5*diameter, 2), linearDensity) {

        // Creating the line properties
        auto props = catenary::make_properties(m_youngModulus, m_sectionArea, m_linearDensity, true);

        // Creating forces on attached bodies
        m_startForce = std::make_shared<FrCatForce>(m_startNode);
        m_endForce   = std::make_shared<FrCatForce>(m_endNode);

        // Adding forces to body
        startNode->GetBody()->AddExternalForce(m_startForce);
        endNode->GetBody()->AddExternalForce(m_endForce);

        // Creating the line
        m_startCatNode = std::make_shared<internal::_CatenaryNode>(m_startForce.get());
        m_startCatNode->SetFixed(true);

        m_endCatNode   = std::make_shared<internal::_CatenaryNode>(m_endForce.get());
        m_endCatNode->SetFixed(true);

        m_catLine      = std::make_shared<internal::_CatenaryBase>(this, props, m_startCatNode, m_endCatNode, m_cableLength);

    }


    FrCatway::~FrCatway() {}

    void FrCatway::Update() {

        // Updating catway's node position from frydom nodes
        m_startCatNode->SetPosition(m_startNode->GetAbsPosition());
        m_endCatNode->SetPosition(m_endNode->GetAbsPosition());

        // Solving catenary line
        m_catLine->Solve();

        // Updating catenaryNodes
        m_startCatNode->Update(false);
        m_endCatNode->Update(true);

        // Asset
        m_asset->Update();
    }

    void FrCatway::Initialize() {

        // Discretizing the line  // FIXME : permettre de discretiser apres coup !
//        m_catLine->Discretize(nbElt);

        // Initializing catenary nodes positions
        m_startCatNode->SetPosition(m_startNode->GetAbsPosition());

        m_endCatNode->SetPosition(m_endNode->GetAbsPosition());

        m_catLine->Initialize();

        // TODO : voir s'il faut faire un update des forces sur les corps (CatForce)


        // Initialize asset
        m_asset = std::make_shared<CatenaryCableAsset>(m_catLine.get());
        m_asset->Initialize();


    }

    void FrCatway::StepFinalize() {
        // TODO
    }

    double FrCatway::GetBreakingTension() {
        return m_catLine->GetBreakingTension();
    }

    Force FrCatway::GetTension(const double s) const {
        return (Force)m_catLine->GetTension(s);
    }

    Force FrCatway::GetStartNodeTension() const {
        return (Force)m_catLine->GetStartNodeTension();
    }

    Force FrCatway::GetEndNodeTension() const {
        return (Force)m_catLine->GetEndNodeTension();
    }

    Position FrCatway::GetAbsPosition(const double s) const {
        return (Position)m_catLine->GetAbsPosition(s);
    }

    std::shared_ptr<CatenaryCableAsset> FrCatway::GetAsset() {
        return m_asset;
    }

    std::shared_ptr<chrono::ChPhysicsItem> FrCatway::GetChronoPhysicsItem() {
        return m_catLine;
    }






    FrCatForce::FrCatForce(std::shared_ptr<FrNode_> node) : FrForce_(node) {}

    void FrCatForce::Update(double time) {
        // Update of FrCatForce is not done by the force itself but by the catenary line that is updated
        // before the bodies external forces.
        // Nothing to do here.


        // FIXME : une force de cable catenaire doit etre rattache a un noeud fixe.

//        // TODO : retirer l'enum SIDE, pas besoin !!
//        // Forces are updated directly by catenary lines
//
//        // Getting tensions
//        Force tension;
//
//        switch (m_side) {
//            case START:
//                tension = m_catenaryLine->GetStartNodeTension();
//                break;
//            case END:
//                tension = m_catenaryLine->GetEndNodeTension();
//                break;
//            default:
//                throw FrException("Unknown cable SIDE");
//        }
//
//        // On update les 2 forces ...
//        // TODO : terminer






    }

    void FrCatForce::Initialize() {
        // TODO  : rien a faire ?
    }

    void FrCatForce::StepFinalize() {
        // TODO : Logging..
    }

    void FrCatForce::SetAbsTension(const Force& tension) {
        SetAbsForce(tension);

        // TODO : transporter le moment...

    }


//    namespace internal {
//
//        CatLineGeom::CatLineGeom(frydom::FrCatway *catLine) : chrono::geometry::ChLine(), m_catLine(catLine) {}
//
//        void CatLineGeom::Evaluate(chrono::ChVector<double> &pos, const double u) const {
//            assert(0. <= u <= 1.);
//            pos = internal::Vector3dToChVector(m_catLine->GetAbsPosition(u * m_catLine->GetCableLength()));
//        }
//
//
//    }








    CatenaryCableAsset::CatenaryCableAsset(internal::_CatenaryBase *cable) : m_cable(cable) {}

    void CatenaryCableAsset::SetNbElements(unsigned int n) {
        m_nbDrawnElements = n;
    }

    void CatenaryCableAsset::Initialize() { // TODO : il semble que ChLine soit capable de rendre des lignes courbes

        // Generating line segments
        double ds = m_cable->GetStretchedLength() / m_nbDrawnElements;

        chrono::ChVector<double> p0, p1;
        chrono::ChColor color;
        p0 = internal::Vector3dToChVector(m_cable->GetStartNode()->GetPosition());
        double s0 = 0.;
        double s1 = ds;
//        for (int i = 1; i < m_nbDrawnElements; i++) {

        while (s1 <= m_cable->GetStretchedLength()) {

            p1 = internal::Vector3dToChVector(m_cable->GetAbsPosition(s1));
            auto newElement = std::make_shared<chrono::ChLineShape>();
            color = chrono::ChColor::ComputeFalseColor(m_cable->GetTension(s0).norm(), 0, m_cable->GetBreakingTension(), true);

            newElement->SetColor(color);
            newElement->SetLineGeometry(std::make_shared<chrono::geometry::ChLineSegment>(p0, p1));

            m_elements.push_back(make_triplet(s0, s1, newElement));
            m_cable->AddAsset(newElement);

            p0 = p1;
            s0 = s1;
            s1 += ds;
        }

    }

    void CatenaryCableAsset::Update() {

        std::shared_ptr<chrono::geometry::ChLineSegment> lineGeom;
        chrono::ChVector<double> p0, p1;
        double s0, s1;

        std::shared_ptr<chrono::ChLineShape> lineShape;
        std::shared_ptr<chrono::geometry::ChLineSegment> lineSegment;

        p0 = internal::Vector3dToChVector(m_cable->GetStartNode()->GetPosition());

        for (auto& element : m_elements) {

            s0 = std::get<0>(element);
            s1 = std::get<1>(element);
            lineShape = std::get<2>(element);
            lineSegment = std::dynamic_pointer_cast<chrono::geometry::ChLineSegment>(lineShape->GetLineGeometry());

            p1 = internal::Vector3dToChVector(m_cable->GetAbsPosition(s1));

            lineSegment->pA = p0;
            lineSegment->pB = p1;

            lineShape->SetColor(
                    chrono::ChColor::ComputeFalseColor(m_cable->GetTension(s0).norm(), 0, m_cable->GetBreakingTension(), false));

            p0 = p1;

        }
    }




}  // end namespace frydom