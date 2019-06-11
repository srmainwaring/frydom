//
// Created by lletourn on 06/06/19.
//

#include "frydom/frydom.h"

using namespace frydom;



/// Attach two bodies, with a fixed link
/// \param Pos1 position in body1 reference frame, of the fixed link marker
/// \param Pos2 position in body2 reference frame, of the fixed link marker
/// \param fc frame convention (NED/NWU)
void AttachBodies(const std::shared_ptr<FrBody>& body1, const std::shared_ptr<FrBody>& body2,
                  const Position& Pos1, const Position& Pos2, FRAME_CONVENTION fc) {

    auto thisNode = body1->NewNode();
    thisNode->SetPositionInBody(Pos1, fc);

    auto newNode = body2->NewNode();
    newNode->SetPositionInBody(Pos2, fc);

    auto fixedLink = make_fixed_link(thisNode, newNode, body1->GetSystem());

}

/// Attach two bodies, with a fixed link
/// \param frame1 frame in body1 reference frame, of the fixed link marker
/// \param frame2 frame in body2 reference frame, of the fixed link marker
/// \return new body created
void AttachBodies(const std::shared_ptr<FrBody>& body1, const std::shared_ptr<FrBody>& body2,
                  const FrFrame& frame1, const FrFrame& frame2) {

    auto thisNode = body1->NewNode();
    thisNode->SetFrameInBody(frame1);

    auto newNode = body2->NewNode();
    newNode->SetFrameInBody(frame2);

    auto fixedLink = make_fixed_link(thisNode, newNode, body1->GetSystem());

}

int main() {

    FRAME_CONVENTION fc = NWU;

    FrOffshoreSystem system;

    //-------------------------------------
    // Barge
    //-------------------------------------

    auto CB28 = system.NewBody();
    CB28->SetName("CB28");
    CB28->AddMeshAsset("CB28_Full.obj");
    CB28->SetColor(Yellow);
    CB28->AllowCollision(false);

    // Inertia
    double mass = 1E6;
    Position COG = {12.,6.,13.44};
    FrFrame COGFrame(COG, FrRotation(), fc);

    double Ixx = 7.9E7, Iyy = 7.7E8, Izz = 8.5E8;

    CB28->SetInertiaTensor(FrInertiaTensor(mass,Ixx,Iyy,Izz,0.,0.,0.,COG,fc));

    auto eqFrame = std::make_shared<FrEquilibriumFrame>(CB28.get());
    system.AddPhysicsItem(eqFrame);

    // hydrostatic
    double K33 = 1.8E7, K44 = 9.2E8, K55 = 9E9;
    FrLinearHydrostaticStiffnessMatrix hydrostaticStiffness;
    hydrostaticStiffness.SetDiagonal(K33, K44, K55);

    auto hydrostaticForce = make_linear_hydrostatic_force(eqFrame, CB28);
    hydrostaticForce->SetStiffnessMatrix(hydrostaticStiffness);


    //-------------------------------------
    // Pile
    //-------------------------------------

    auto pile = system.NewBody();
    pile->SetName("Pile");
    pile->SetColor(DarkRed);
    makeItCylinder(pile, 3, 16, 80E3);
    pile->AllowCollision(false);

    FrFrame pileFrame(Position(0.,-8.,0.), FrRotation(), fc);
    pileFrame.RotX_DEGREES(-90,fc,true);

    FrFrame CB28Frame(Position(7.6, -4.064, 4.04), FrRotation(), fc);

    AttachBodies(CB28, pile, CB28Frame, pileFrame);

//    auto pile = CB28->NewBody(CB28Frame, pileFrame);

    //-------------------------------------
    // Manifold
    //-------------------------------------

    auto manifold = system.NewBody(); // CB28->NewBody(Position(-8, -5.6, 4.04), Position(0.,0.,-3), fc);
    manifold->SetName("Manifold");
    manifold->SetColor(DarkGreen);
    makeItBox(manifold, 16, 9.5, 6, 140E3);
    manifold->AllowCollision(false);

    AttachBodies(CB28, manifold, Position(-8, -5.6, 4.04), Position(0.,0.,-3), fc);

//    system.Initialize();
//    system.DoAssembly();

    system.SetTimeStep(0.01);

//    system.SolveStaticWithRelaxation();

    system.RunInViewer(0., 50, false);
//    system.Visualize(50, false);
}
