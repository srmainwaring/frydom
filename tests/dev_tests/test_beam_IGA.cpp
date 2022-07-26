//
// Created by frongere on 22/04/2020.
//



#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverPMINRES.h"
//#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/fea/ChBuilderBeam.h"
//#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/physics/ChLoaderU.h"
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoadContainer.h"


#include "frydom/frydom.h"


using namespace chrono;
//using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;


//    ChElementBeamIGA
//    ChNodeFEAxyzrot
//    ChMesh
//    ChBeamSectionCosserat
//    ChElasticityCosseratSimple

using namespace frydom;




class MyElementBeamIGA : public chrono::fea::ChElementBeamIGA {
 public:

  // TODO: a retirer, pas besoin
  void SetSystem(chrono::ChSystem* system) {
    m_system = system;
  }

  // TODO: ajouter une methode permettant de ne pas

  /// Gets the absolute xyz velocity of a point on the beam line, at abscissa 'eta'.
  /// Note, eta=-1 at node1, eta=+1 at node2.
  virtual void EvaluateSectionSpeed(const double eta,
                                    chrono::ChVector<> &point_speed) {

    // compute parameter in knot space from eta-1..+1
    double u1 = knots(order); // extreme of span
    double u2 = knots(knots.rows() - order - 1);
    double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
    int nspan = order;

    chrono::ChVectorDynamic<> N((int) nodes.size());

    chrono::geometry::ChBasisToolsBspline::BasisEvaluate(
        this->order,
        nspan,
        u,
        knots,
        N);           ///< here return  in N

    point_speed = chrono::VNULL;
    for (int i = 0; i < nodes.size(); ++i) {
      point_speed += N(i) * nodes[i]->coord_dt.pos;
    }
  }

  /// Gets the absolute xyz position of a point on the beam line, at abscissa 'eta'.
  /// Note, eta=-1 at node1, eta=+1 at node2.
  virtual void EvaluateSectionAcceleration(const double eta,
                                           chrono::ChVector<> &point_acceleration) {

    // compute parameter in knot space from eta-1..+1
    double u1 = knots(order); // extreme of span
    double u2 = knots(knots.rows() - order - 1);
    double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
    int nspan = order;

    chrono::ChVectorDynamic<> N((int) nodes.size());

    chrono::geometry::ChBasisToolsBspline::BasisEvaluate(
        this->order,
        nspan,
        u,
        knots,
        N);           ///< here return  in N

    point_acceleration = chrono::VNULL;
    for (int i = 0; i < nodes.size(); ++i) {
      point_acceleration += N(i) * nodes[i]->coord_dtdt.pos;
    }
  }



  chrono::ChSystem * m_system;  // TODO: a retirer


};

class MyLoader : public chrono::ChLoaderUdistributed {
 public:

  MyLoader(std::shared_ptr <chrono::ChLoadableU> loadable) : chrono::ChLoaderUdistributed(loadable) {}

  int GetIntegrationPointsU() override {
    return 4;
  }

  void ComputeF(const double U,
                chrono::ChVectorDynamic<> &F,
                chrono::ChVectorDynamic<> *state_x,
                chrono::ChVectorDynamic<> *state_w) override {

    auto element = std::dynamic_pointer_cast<MyElementBeamIGA>(loadable);

    Force unit_force;


    // Get the point position
    chrono::ChVector<double> position_;
    chrono::ChQuaternion<double> quaternion_;

    element->EvaluateSectionFrame(U, position_, quaternion_);

    auto position = internal::ChVectorToVector3d<Position>(position_);

    bool m_include_waves = true;

    // Buoyancy

//    double gravity = m_system->GetGravityAcceleration();
//    auto fluid_type = m_system->GetEnvironment()->GetFluidTypeAtPointInWorld(position, NWU, m_include_waves);
//    auto fluid_density = m_system->GetEnvironment()->GetFluidDensity(fluid_type);
//
//    double section = m_cable->GetProperties()->GetSectionArea(); // TODO: voir si on ne prend pas un diameter hydro plutot...

    // Temporaire
    double gravity = 9.81;
    double fluid_density = 1023.;
    double section = std::dynamic_pointer_cast<fea::ChInertiaCosseratSimple>(element->GetSection()->GetInertia())->GetArea();


    Force hydrostatic_force = {0., 0., fluid_density * section * gravity};


    // Morison

    // FIXME:

    // Evaluate velocity at U

//    auto vel_a = internal::ChVectorToVector3d<Velocity>(element->GetNodeA()->GetPos_dt());
//    auto vel_b = internal::ChVectorToVector3d<Velocity>(element->GetNodeB()->GetPos_dt());
//
//    // We take the mean velocity value, having nothing to interpolate best...
//    auto cable_velocity = 0.5 * (vel_a + vel_b);
    chrono::ChVector<double> vel;
    element->EvaluateSectionSpeed(U, vel);
    auto cable_velocity = internal::ChVectorToVector3d<Velocity>(vel);

//    auto acc_a = internal::ChVectorToVector3d<Acceleration>(element->GetNodeA()->GetPos_dtdt());
//    auto acc_b = internal::ChVectorToVector3d<Acceleration>(element->GetNodeB()->GetPos_dtdt());
//
//    // We take the mean acceleration value, having nothing to interpolate best...
//    auto cable_acceleration = 0.5 * (acc_a + acc_b);
    chrono::ChVector<double> acc;
    element->EvaluateSectionAcceleration(U, acc);
    auto cable_acceleration = internal::ChVectorToVector3d<Acceleration>(acc);

    // Fluid velocity and acceleration at point U
    Velocity fluid_relative_velocity;
    Acceleration fluid_relative_acceleration;
//    if (fluid_type == WATER) {
//      auto ocean = m_system->GetEnvironment()->GetOcean();
//
//      // Current
//      fluid_relative_velocity += ocean->GetCurrent()->GetFluxVelocityInWorld(position, NWU);
//
//      // Wave orbital velocities
//      if (m_include_waves) {
//        auto wave_field = ocean->GetFreeSurface()->GetWaveField();
//        fluid_relative_velocity += wave_field->GetVelocity(position, NWU);
//        fluid_relative_acceleration += wave_field->GetAcceleration(position, NWU);
//      }
//
//    } else {  // AIR
//      fluid_relative_velocity +=
//          m_system->GetEnvironment()->GetAtmosphere()->GetWind()->GetFluxVelocityInWorld(position, NWU);
//    }

//    fluid_relative_velocity.x() = 10.;

    // Taking the cable motion into account into the fluid motion
    fluid_relative_velocity -= cable_velocity;
    fluid_relative_acceleration -= cable_acceleration;

    // Getting the tangent direction of cable
    Direction tangent_direction = internal::ChVectorToVector3d<Direction>(quaternion_.GetXaxis());

    // Getting tangential and normal relative fluid velocities
    Velocity tangent_fluid_velocity = fluid_relative_velocity.dot(tangent_direction) * tangent_direction;
    Velocity normal_fluid_velocity = fluid_relative_velocity - tangent_fluid_velocity;

    // Computing morison load
//    auto cable_properties = m_cable->GetProperties();
//    double d = cable_properties->GetHydrodynamicDiameter();
//    double d = section
//    double A = 0.25 * MU_PI * d * d;
    double d = std::sqrt(4. * section / M_PI);

    double Cfn = 1.2;
    double Cft = 0.;

    double Cmn = 2.;
    double Cmt = 0.;

    Force morison_force_drag;
    // Normal morison drag
    morison_force_drag += 0.5 * fluid_density * Cfn * d *
                     normal_fluid_velocity.norm() * normal_fluid_velocity;

    // Tangent morison drag
    morison_force_drag += 0.5 * fluid_density * Cft * d *
                     tangent_fluid_velocity.norm() * tangent_fluid_velocity;

    Acceleration tangent_fluid_acceleration = fluid_relative_acceleration.dot(tangent_direction) * tangent_direction;
    Acceleration normal_fluid_acceleration = fluid_relative_acceleration - tangent_fluid_acceleration;


    // Added mass effects
    Force morison_force_added_mass;

    // Normal morison
//    morison_force_added_mass += fluid_density * Cmn * section *
//        normal_fluid_acceleration.norm() * normal_fluid_acceleration;
//
//    morison_force_added_mass += fluid_density * Cmt * section *
//        tangent_fluid_acceleration.norm() * tangent_fluid_acceleration;

//    unit_force += morison_force;

    // FIXME: les effets de masse ajoutee vont etre pris en compte dans la masse du corps !
    // Par contre du coup, c'est à nous de gerer la force de gravite !

    // C'est lors du setupInitial de ChElementBeamIGA qu'on calcule les matrices masse. Du coup, il faudrait incorporer
    // la masse ajoutee dedans. Par contre, l'effet de la gravite sera calcule dans le present chargement.
    // Verifier qu'on obtient bien le bon comportement suivant qu'on laisse la gravite en auto ou qu'on la gere nous-meme...







    unit_force = hydrostatic_force + morison_force_drag + morison_force_added_mass;

    auto time = element->m_system->GetChTime();


    // FIXME: a retirer
    if (element->m_system->GetChTime() > 5.) {
      unit_force.setZero();
    }



    // Pasting the results

    F.segment(0, 3) = unit_force; // load, force part
    F.segment(3, 0) = chrono::VNULL.eigen(); // No torque

  }

};

class MyLoad : public chrono::ChLoad<MyLoader> {
 public:

  MyLoad(std::shared_ptr <chrono::ChLoadableU> loadable) : chrono::ChLoad<MyLoader>(loadable) {}

};


/// Class for an helper object that provides easy functions to create
/// complex beams of ChElementBeamIGA class, for example subdivides a segment
/// in multiple finite elements.
class MyFEACableBuilder {
 protected:
  std::vector <std::shared_ptr<MyElementBeamIGA>> beam_elems;
  std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> beam_nodes;

 public:
  /// Helper function.
  /// Adds beam FEM elements to the mesh to create a segment beam
  /// from point starting_node to point ending_node, using ChElementBeamIGA type elements.
  /// Before running, each time resets lists of beam_elems and beam_nodes.
  void BuildBeam(std::shared_ptr <chrono::fea::ChMesh> mesh,              ///< mesh to store the resulting elements
                 std::shared_ptr <chrono::fea::ChBeamSectionCosserat> sect,    ///< section property for beam elements
                 const int N,                               ///< number of elements in the segment
                 const FrNode starting_node,                        ///< starting point
                 const FrNode ending_node,                        ///< ending point
//                 const ChVector<> Ydir,                     ///< the 'up' Y direction of the beam
                 const int order = 3                        ///< the order of spline (default=3,cubic)
  ) {
    beam_elems.clear();
    beam_nodes.clear();

//    // rotation of all nodes
//    ChMatrix33<> mrot;
//    mrot.Set_A_Xdir(ending_node - starting_node, Ydir);

    Position p0 = starting_node.GetPositionInWorld(NWU);
    Position p1 = ending_node.GetPositionInWorld(NWU);

    FrRotation rot;
    rot.SetXaxis(p1 - p0, {0., 0., 1.}, NWU);


    int p = order;

    // Create the 'complete' knot vector, with multiple at the ends
    ChVectorDynamic<> myknots(N + p + p + 1);
    geometry::ChBasisToolsBspline::ComputeKnotUniformMultipleEnds(myknots, p, 0.0, 1.0);

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < N + p; ++i_node) {
      double abscyssa = ((double) i_node / (double) (N + p - 1));

      // position of node
//      ChVector<> pos = starting_node + (ending_node - starting_node) * abscyssa;

      Position pos = p0 + (p1 - p0) * abscyssa;

      FrFrame node_frame(pos, rot, NWU);
//      node_frame.Set


      auto hnode_i = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(internal::FrFrame2ChFrame(node_frame));
      mesh->AddNode(hnode_i);
      mynodes.push_back(hnode_i);
      this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
      std::vector<double> local_knots;
      for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
        local_knots.push_back(myknots(i_el + i_el_knot));
      }

      std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> element_nodes;
      for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
        element_nodes.push_back(mynodes[i_el + i_el_node]);
      }

      auto belement_i = std::make_shared<MyElementBeamIGA>();
      belement_i->SetNodesGenericOrder(element_nodes, local_knots, p);
      belement_i->SetSection(sect);
      mesh->AddElement(belement_i);
      this->beam_elems.push_back(belement_i);
    }
  }

  /// Helper function.
  /// Adds beam FEM elements to the mesh to create a spline beam
  /// using ChElementBeamIGA type elements, given a B-spline line in 3D space.
  /// Before running, each time resets lists of beam_elems and beam_nodes.
  void BuildBeam(std::shared_ptr <chrono::fea::ChMesh> mesh,                ///< mesh to store the resulting elements
                 std::shared_ptr <chrono::fea::ChBeamSectionCosserat> sect, ///< section material for beam elements
                 geometry::ChLineBspline &spline,             ///< the B-spline to be used as the centerline
                 const ChVector<> Ydir                        ///< the 'up' Y direction of the beam
  ) {
    beam_elems.clear();
    beam_nodes.clear();

    int p = spline.GetOrder();

    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = spline.Knots().rows() - p - p - 1;  // = n+p+1 -p-p-1 = n-p

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < spline.Points().size(); ++i_node) {
      double abscyssa = ((double) i_node / (double) (spline.Points().size() - 1));

      // position of node
      ChVector<> pos = spline.Points()[i_node];

      // rotation of node, x aligned to tangent at input spline
      ChMatrix33<> mrot;
      ChVector<> tangent;
      spline.Derive(tangent, abscyssa);
      mrot.Set_A_Xdir(tangent, Ydir);

      auto hnode_i = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
      mesh->AddNode(hnode_i);
      mynodes.push_back(hnode_i);
      this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
      std::vector<double> my_el_knots;
      for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
        my_el_knots.push_back(spline.Knots()(i_el + i_el_knot));
      }

      std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> my_el_nodes;
      for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
        my_el_nodes.push_back(mynodes[i_el + i_el_node]);
      }

      auto belement_i = std::make_shared<MyElementBeamIGA>();
      belement_i->SetSystem(mesh->GetSystem()); // TODO: a retirer test
      belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
      belement_i->SetSection(sect);
      mesh->AddElement(belement_i);
      this->beam_elems.push_back(belement_i);
    }
  }

  /// Access the list of elements used by the last built beam.
  /// It can be useful for changing properties afterwards.
  /// This list is reset all times a 'Build...' function is called.
  std::vector <std::shared_ptr<MyElementBeamIGA>> &GetLastBeamElements() { return beam_elems; }

  /// Access the list of nodes used by the last built beam.
  /// It can be useful for adding constraints or changing properties afterwards.
  /// This list is reset all times a 'Build...' function is called.
  std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> &GetLastBeamNodes() { return beam_nodes; }
};


int main() {


// Create a Chrono::Engine physical system
  ChSystemNSC my_system;


  // Creating a load container to load the cable
  auto load_container = std::make_shared<chrono::ChLoadContainer>();
  my_system.Add(load_container);



  // Create a mesh, that is a container for groups
  // of elements and their referenced nodes.
  // Remember to add it to the system.
  auto my_mesh = std::make_shared<chrono::fea::ChMesh>();
//  my_mesh->SetAutomaticGravity(false);
  my_system.Add(my_mesh);

  // Create a section, i.e. thickness and material properties
  // for beams. This will be shared among some beams.

  // Real cable
  double diam_ = 0.168; // m
  double EA_ = 602.68e6; // N
  double lambda_ = 141; // kg/m
  double cable_length = 500; // m

  double A_ = M_PI * diam_ * diam_ / 4.; // m**2
  double E = EA_ / A_; // Pa
  double density = lambda_ / A_; // kg/m**3

  // Reduced cable
  double scale = 1. / 1.; // The scale at which we are testing the model
  double diameter = diam_ * scale;


  // Constitutive model
  auto melasticity = std::make_shared<chrono::fea::ChElasticityCosseratSimple>();
  melasticity->SetYoungModulus(E);
  melasticity->SetGshearModulus(E * 0.3); // Jouer avec...
  //melasticity->SetBeamRaleyghDamping(1e3);

  // Damping
  auto mdamping = std::make_shared<chrono::fea::ChDampingCosseratRayleigh>(melasticity, 1e3);

  // Inertia
  auto minertia = std::make_shared<chrono::fea::ChInertiaCosseratSimple>();
  minertia->SetAsCircularSection(diameter, density);

  // Section definition
  auto msection = std::make_shared<chrono::fea::ChBeamSectionCosserat>(minertia, melasticity);
  msection->SetDamping(mdamping);


  // Building the BSpline to initialize
  std::vector <ChVector<>> my_points;

  int nb_ctrl_points = 200;
  double dz = cable_length * scale / (double) (nb_ctrl_points - 1);
  double z = 0.;
  for (int i = 0; i < nb_ctrl_points; i++) {
    my_points.emplace_back(ChVector<>(0., 0., z));
    z += dz;
  }

  geometry::ChLineBspline my_spline(2,          // order (3 = cubic, etc)
                                    my_points); // control points, will become the IGA nodes


  MyFEACableBuilder builderR;
  builderR.BuildBeam(my_mesh,            // the mesh to put the elements in
                     msection,           // section of the beam
                     my_spline,          // Bspline to match (also order will be matched)
                     VECT_Y);            // suggested Y direction of section

  builderR.GetLastBeamNodes().front()->SetFixed(true);
//  builderR.GetLastBeamNodes().back()->SetFixed(true);



  // Adding loads on each element newly created by the builder
  for (auto &element : builderR.GetLastBeamElements()) {
    auto load = std::make_shared<MyLoad>(element);
    load_container->Add(load);
  }

  // Viz...
  auto mvisualizebeamA = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(my_mesh.get()));
  mvisualizebeamA->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_SURFACE);
  mvisualizebeamA->SetSmoothFaces(true);
  my_mesh->AddAsset(mvisualizebeamA);

  auto mvisualizebeamC = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(my_mesh.get()));
  mvisualizebeamC->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
  mvisualizebeamC->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
  mvisualizebeamC->SetSymbolsThickness(0.006);
  mvisualizebeamC->SetSymbolsScale(0.01);
  mvisualizebeamC->SetZbufferHide(false);
  my_mesh->AddAsset(mvisualizebeamC);



  ///////// SECOND CABLE



//  // Building the BSpline to initialize a second cable
//  auto my_mesh_2 = std::make_shared<chrono::fea::ChMesh>();
////  my_mesh->SetAutomaticGravity(false);
//  my_system.Add(my_mesh_2);
//
//
//  std::vector <ChVector<>> my_points_2;
//
////  int nb_ctrl_points = 200;
////  double dz = cable_length * scale / (double) (nb_ctrl_points - 1);
//  z = 0.;
//  for (int i = 0; i < nb_ctrl_points; i++) {
//    my_points_2.emplace_back(ChVector<>(3., 0., z));
//    z += dz;
//  }
//
//  geometry::ChLineBspline my_spline_2(2,          // order (3 = cubic, etc)
//                                    my_points_2); // control points, will become the IGA nodes
//
//
//  MyFEACableBuilder builderR_2;
//  builderR_2.BuildBeam(my_mesh_2,            // the mesh to put the elements in
//                     msection,           // section of the beam
//                     my_spline_2,          // Bspline to match (also order will be matched)
//                     VECT_Y);            // suggested Y direction of section
//
//  builderR_2.GetLastBeamNodes().front()->SetFixed(true);
////  builderR.GetLastBeamNodes().back()->SetFixed(true);
//
//  auto mvisualizebeamA_2 = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(my_mesh_2.get()));
//  mvisualizebeamA_2->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_SURFACE);
//  mvisualizebeamA_2->SetSmoothFaces(true);
//  my_mesh_2->AddAsset(mvisualizebeamA_2);
//
//  auto mvisualizebeamC_2 = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(my_mesh_2.get()));
//  mvisualizebeamC_2->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
//  mvisualizebeamC_2->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
//  mvisualizebeamC_2->SetSymbolsThickness(0.006);
//  mvisualizebeamC_2->SetSymbolsScale(0.01);
//  mvisualizebeamC_2->SetZbufferHide(false);
//  my_mesh_2->AddAsset(mvisualizebeamC_2);




//  auto mbodywing = std::make_shared<ChBodyEasyBox>(0.01, 0.2, 0.05, 2000);

//  mbodywing->SetCoord(builderR.GetLastBeamNodes().back()->GetCoord());
//  application.GetSystem()->Add(mbodywing);


// Rigidly attach body to cable
//  auto myjoint = std::make_shared<ChLinkMateFix>();
//  myjoint->Initialize(builderR.GetLastBeamNodes().back(), mbodywing);
//  application.GetSystem()->Add(myjoint);


  // Attach a visualization of the FEM mesh.








  // System setting and irrlicht settings



  // Create the Irrlicht visualization (open the Irrlicht device,
  // bind a simple user interface, etc. etc.)
  ChIrrApp application(&my_system, L"IGA beams DEMO (SPACE for dynamics, F10 / F11 statics)",
                       core::dimension2d<u32>(800, 600),
                       false, true);

  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
  application.AddTypicalLogo();
  application.AddTypicalSky(std::string(RESOURCES_VIZU_PATH) + "skybox/");
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

  application.SetVideoframeSave(true);
  application.SetVideoframeSaveInterval(2);

  // Solver default settings for all the sub demos:
  my_system.SetSolverType(ChSolver::Type::MINRES);
  my_system.SetSolverMaxIterations(500);
  my_system.SetSolverTolerance(1e-14);


  auto msolver = std::static_pointer_cast<ChSolverPMINRES>(my_system.GetSolver());
  msolver->SetVerbose(false);
  //msolver->SetDiagonalPreconditioning(true);

  application.SetTimestep(0.01);


  // Clear previous demo, if any:
//  application.GetSystem()->Clear();
  application.GetSystem()->SetChTime(0);




  // This is needed if you want to see things in Irrlicht 3D view.
  application.AssetBindAll();
  application.AssetUpdateAll();

  // Mark completion of system construction
  application.GetSystem()->Setup();

  while (application.GetDevice()->run()) {
    application.BeginScene();
    application.DrawAll();
    application.DoStep();
    application.EndScene();
  }


  return 0.;
}
