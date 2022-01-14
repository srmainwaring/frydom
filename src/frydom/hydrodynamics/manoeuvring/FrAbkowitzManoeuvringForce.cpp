//
// Created by frongere on 12/10/2021.
//

#include "FrAbkowitzManoeuvringForce.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/io/JSONNode.h"

namespace frydom {

  FrAbkowitzManoeuvringForce::FrAbkowitzManoeuvringForce(const std::string &name,
                                                         FrBody *body,
                                                         const std::string &file,
                                                         const Position& mid_ship) :
      FrForce(name, "FrManoeuvringForce", body),
      c_filepath(file),
      m_Xvv(0.), m_Xvvvv(0.), m_Xrr(0.), m_Xvr(0.),
      m_Yv(0.), m_Yvvv(0.), m_Yvrr(0.), m_Yr(0.), m_Yrrr(0.), m_Yvvr(0.),
      m_Nv(0.), m_Nvvv(0.), m_Nvrr(0.), m_Nr(0.), m_Nrrr(0.), m_Nvvr(0.),
      m_mid_ship(mid_ship) {}

  void FrAbkowitzManoeuvringForce::Initialize() {
    FrForce::Initialize();
    LoadManoeuvringData();
  }

  void FrAbkowitzManoeuvringForce::Compute(double time) {

    auto body = GetBody();
    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

    // Get the velocity at mid-ship
    Velocity vessel_vel_world = body->GetVelocityInWorldAtPointInBody(m_mid_ship, NWU);
    auto vessel_angular_vel_world = body->GetAngularVelocityInWorld(NWU);

    // Projection to the planar frame
    Direction vessel_x_axis_world = body->GetFrame().GetXAxisInParent(NWU);
    vessel_x_axis_world[2] = 0;
    vessel_x_axis_world.Normalize();

    Direction vessel_y_axis_world = body->GetFrame().GetYAxisInParent(NWU);
    vessel_y_axis_world[2] = 0;
    vessel_y_axis_world.Normalize();

    double u = vessel_vel_world.dot(vessel_x_axis_world);
    double v = vessel_vel_world.dot(vessel_y_axis_world);
    double r = vessel_angular_vel_world.GetWz();

    // Adim
    double vnorm2 = u*u + v*v;
    double vnorm = std::sqrt(vnorm2);
    auto q = 0.5 *rho * m_Lpp * m_draft * vnorm2;

    // Compute hydrodynamic derivatives
    double c_Xvv = m_Xvv * v*v;
    double c_Xvr = m_Xvr * v*r;
    double c_Xrr = m_Xrr * r*r;
    double c_Xvvvv = m_Xvvvv * pow(v, 4);

    double c_Yv = m_Yv * v;
    double c_Yr = m_Yr * r;
    double c_Yvvv = m_Yvvv * pow(v, 3);
    double c_Yvvr = m_Yvvr * v*v*r;
    double c_Yvrr = m_Yvrr * v*r*r;
    double c_Yrrr = m_Yrrr * r*r*r;

    double c_Nv = m_Nv * v;
    double c_Nr = m_Nr * r;
    double c_Nvvv = m_Nvvv * pow(v, 3);
    double c_Nvvr = m_Nvvr * v*v*r;
    double c_Nvrr = m_Nvrr * v*r*r;
    double c_Nrrr = m_Nrrr * pow(r, 3);

    // Compute force and torque
    auto X = q * (c_Xvv + c_Xvr + c_Xrr + c_Xvvvv);
    auto Y = q * (c_Yv + c_Yr + c_Yvvv + c_Yvvr + c_Yvrr + c_Yrrr);
    auto N = q * m_Lpp * (c_Nv + c_Nr + c_Nvvv + c_Nvvr + c_Nvrr + c_Nrrr);

    Force forceInWorld = X * vessel_x_axis_world + Y * vessel_y_axis_world;
    Torque torqueInWorld = {0., 0., N};

    SetForceTorqueInWorldAtCOG(forceInWorld, torqueInWorld, NWU);

  }

  void FrAbkowitzManoeuvringForce::DefineLogMessages() {
    FrForce::DefineLogMessages();
  }

  void FrAbkowitzManoeuvringForce::LoadManoeuvringData() {
    JSONNode node(c_filepath);

    JSONNode man_node = node({"manoeuvring_model"});

    if (man_node.get_val<std::string>("type") != "abkowitz") {
      std::cerr << "Manoeuvring model enclosed in json file "
                << c_filepath
                << " was intended to be of type abkowitz but type was "
                << man_node.get_val<std::string>("type")
                << std::endl
                << "Aborting..."
                << std::endl;
      exit(EXIT_FAILURE);
    }

    auto file_format_version = man_node.get_val<std::string>("file_format_version");

    m_Lpp = man_node({"hull_characteristics"}).get_val<double>("length_m");
    m_draft = man_node({"hull_characteristics"}).get_val<double>("draft_m");

    auto wave_resistance_file = man_node.get_filepath("wave_resistance_file");

    JSONNode coeffs_node = man_node({"coefficients"});

    m_Xvv = coeffs_node.get_val<double>("Xvv", 0.0);
    m_Xvvvv = coeffs_node.get_val<double>("Xvvvv", 0.0);
    m_Xrr = coeffs_node.get_val<double>("Xrr", 0.0);
    m_Xvr = coeffs_node.get_val<double>("Xvr", 0.0);
    m_Yv = coeffs_node.get_val<double>("Yv", 0.0);
    m_Yvvv = coeffs_node.get_val<double>("Yvvv", 0.0);
    m_Yvrr = coeffs_node.get_val<double>("Yvrr", 0.0);
    m_Yr = coeffs_node.get_val<double>("Yr", 0.0);
    m_Yrrr = coeffs_node.get_val<double>("Yrrr", 0.0);
    m_Yvvr = coeffs_node.get_val<double>("Yvvr", 0.0);
    m_Nv = coeffs_node.get_val<double>("Nv", 0.0);
    m_Nvvv = coeffs_node.get_val<double>("Nvvv", 0.0);
    m_Nvrr = coeffs_node.get_val<double>("Nvrr", 0.0);
    m_Nr = coeffs_node.get_val<double>("Nr", 0.0);
    m_Nrrr = coeffs_node.get_val<double>("Nrrr", 0.0);
    m_Nvvr = coeffs_node.get_val<double>("Nvvr", 0.0);
  }

  std::shared_ptr<FrAbkowitzManoeuvringForce>
      make_abkowitz_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body,
                                      const std::string& file, const Position& mid_ship) {
    auto force = std::make_shared<FrAbkowitzManoeuvringForce>(name, body.get(), file, mid_ship);
    body->AddExternalForce(force);
    return force;
  }

}  // end namespace frydom
