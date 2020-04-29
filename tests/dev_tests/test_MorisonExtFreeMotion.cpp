//
// Created by camille on 14/04/2020.
//

#include  "frydom/frydom.h"

using namespace frydom;

// -----------------------------------------------
// Linear spring
// -----------------------------------------------

class LinearSpring : public FrForce {

 public:

  LinearSpring(const std::string& name,
               const std::shared_ptr<FrNode>& node1,
               const std::shared_ptr<FrNode>& node2,
               Vector3d<double> stiffness) :
               FrForce(name, "LinearSpring", node1->GetBody()),
               m_stiffness(stiffness) {
    m_node1 = node1;
    m_node2 = node2;
  }

 protected:

  void Compute(double time) override {

    Position worldPos1 = m_node1->GetPositionInWorld(NWU);
    Position worldPos2 = m_node2->GetPositionInWorld(NWU);
    Direction vect = worldPos1 - worldPos2;

    Force forceInWorld = -m_stiffness.cwiseProduct(vect);

    SetForceTorqueInWorldAtPointInWorld(forceInWorld, Torque(), worldPos1, NWU);
  }

 private:

  std::shared_ptr<FrNode> m_node1;
  std::shared_ptr<FrNode> m_node2;
  Vector3d<double> m_stiffness;

};

std::shared_ptr<LinearSpring> make_linear_spring(const std::string& name,
                                                 const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2,
                                                 Vector3d<double> stiffness) {
  auto force = std::make_shared<LinearSpring>(name, node1, node2, stiffness);
  auto body = node1->GetBody();
  body->AddExternalForce(force);
}

// ------------------------------------------------
// Main
// ------------------------------------------------

int main(int argc, char* argv[]) {

  // System

  FrOffshoreSystem system("test_MorisonExtFreeMotion");

  // Body

  auto body = system.NewBody("cylinder");
  body->SetPosition({0., 0., 0.}, NWU);
  //body->GetDOFMask()->SetLock_X(true);
  //body->GetDOFMask()->SetLock_Y(true);
  //body->GetDOFMask()->SetLock_Z(true);
  //body->GetDOFMask()->SetLock_Rx(true);
  //body->GetDOFMask()->SetLock_Ry(false);
  //body->GetDOFMask()->SetLock_Rz(true);

  double radius = 1.;
  double length = 20.;
  double zcog = -8.;

  double mass = 45100.;
  double Ixx = mass / 12. * (3.*std::pow(radius, 2)+ std::pow(length, 2));
  double Iyy = Ixx;
  double Izz = 0.5 * mass * std::pow(radius, 2);
  FrInertiaTensor inertia(mass, Ixx, Iyy, Izz, 0., 0., 0., {0., 0., zcog}, NWU);

  body->SetInertiaTensor(inertia);

  body->SetPosition({2., 0., 0.}, NWU);
  body->SetRotation(FrRotation({0., 1., 0.}, 0.5* M_PI_4, NWU));

  // Mesh asset

  auto cylinderMesh = FrFileSystem::join({system.config_file().GetDataFolder(),
                                          "ce/Cylinder/cylinder_r1_l20.obj"});
  body->AddMeshAsset(cylinderMesh);

  // Hydrostatic

  FrFrame meshOffset;
  auto hydroMesh = make_hydro_mesh("HydroMesh", body, cylinderMesh, meshOffset,
      FrHydroMesh::ClippingSupport::PLANESURFACE);

  auto forceHst = make_nonlinear_hydrostatic_force("nonlinear_hydrostatic", body, hydroMesh);

  // Morison

  auto morisonModel = make_morison_model("morison", body, true);
  morisonModel->AddElement({0., 0., -14}, {0., 0., 6.}, 2., 0.5, 0.6, 0, 20);
  auto morisonForce = make_morison_force("morison", body, morisonModel);

  // Additional spring force

  //auto body_node = body->NewNode("body_node");
  //body_node->SetPositionInBody({0., 0., -7}, NWU);
  //auto world_node = system.GetWorldBody()->NewNode("world_node");
  //world_node->SetPositionInWorld({0., 0., -7.}, NWU);

  //Vector3d<double> stiffness(17804., 17804., 10000.);

  //auto spring = make_linear_spring("LinearSpring", body_node, world_node, stiffness);

  // Simulation

  double dt = 0.01;
  double t_end = 50.;
  double time = 0.;

  system.SetTimeStep(dt);
  system.Initialize();

  bool is_irrlicht = true;

  if (is_irrlicht) {
    system.RunInViewer(t_end, 100.);

  } else {
    while (time < t_end) {
      system.AdvanceTo(time);
      std::cout << "time : " << time << " s" << std::endl;
      time += dt;
    }
  }

}