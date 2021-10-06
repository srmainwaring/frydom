//
// Created by lletourn on 25/05/2021.
//

#include "FrRudderForce.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {

  FrRudderForce::FrRudderForce(const std::string &name, FrBody *body, const std::shared_ptr<FrNode> &node,
                               const std::string &fileCoefficients)
      : FrActuatorForceBase(name, "FrRudderForce", body),
        m_rudderNode(node),
        m_projectedLateralArea(1),
        m_wakeFraction0(0.4),
        m_K1(4),
        has_hullRudderInteraction(false),
        m_rootChord(0.),
        m_height(0.),
        m_ramp_slope(1. * DEG2RAD),
        m_k(2.),
        m_beta1(1.3),
        m_beta2(MU_PI_2),
        m_K2(0.5),
        m_K3(0.45),
        c_fileCoefficients(fileCoefficients) {

    m_rudderAngle = new FrConstantFunction(0.);

  }

  void FrRudderForce::Initialize() {
    FrForce::Initialize();
    ReadCoefficientsFile();
  }

  void FrRudderForce::SetProjectedLateralArea(double area) {
    m_projectedLateralArea = area;
  }

  double FrRudderForce::GetProjectedLateralArea() const {
    return m_projectedLateralArea;
  }

  void FrRudderForce::SetHeight(double h) {
    m_height = h;
  }

  double FrRudderForce::GetHeight() const {
    return m_height;
  }

  void FrRudderForce::SetRootChord(double br) {
    m_rootChord = br;
  }

  double FrRudderForce::GetRootChord() const {
    return m_rootChord;
  }

  void FrRudderForce::SetRampSlope(double value, FREQUENCY_UNIT unit) {
    m_ramp_slope = convert_frequency(value, unit, RADS);
  }

  double FrRudderForce::GetRampSlope(FREQUENCY_UNIT unit) const {
    return convert_frequency(m_ramp_slope, RADS, unit);
  }

  Velocity FrRudderForce::GetRudderRelativeVelocityInWorld() const {

    auto body = GetBody();
    auto environment = body->GetSystem()->GetEnvironment();
    auto rudderVelocityInWorld = body->GetVelocityInWorldAtPointInBody(GetPositionInBody(), NWU);
    Velocity rudderRelativeVelocity = -environment->GetRelativeVelocityInFrame(FrFrame(), rudderVelocityInWorld, WATER,
                                                                               NWU);

    if (has_hullRudderInteraction) {
      auto rudderRelativeVelocityInBody = body->ProjectVectorInBody(rudderRelativeVelocity, NWU);
      auto sidewashAngle = rudderRelativeVelocityInBody.GetProjectedAngleAroundZ(RAD);
      auto uRA = rudderRelativeVelocityInBody.GetVx() * (1. - GetWakeFraction(sidewashAngle));
      auto vRA = rudderRelativeVelocityInBody.GetVy();
      if (has_hull_influence_transverse_velocity)
        vRA *= Kappa(ComputeSpecialSidewashAngle());
      rudderRelativeVelocity = body->ProjectVectorInWorld(Velocity(uRA, vRA, 0.0), NWU);
    }

    return rudderRelativeVelocity;
  }

  double FrRudderForce::GetWakeFraction(double sidewashAngle) const {
    return m_wakeFraction0 * exp(-m_K1 * sidewashAngle * sidewashAngle);
  }

  void FrRudderForce::SetRudderAngle(double angle, ANGLE_UNIT unit) {
    if (unit == DEG) angle *= DEG2RAD;

    double actualRudderAngle = GetRudderAngle(RAD);

    if (std::abs(angle - GetRudderAngle(RAD)) <= 0)
      m_rudderAngle = new FrConstantFunction(angle);
    else {
      auto t_ramp = std::abs(angle - actualRudderAngle) / m_ramp_slope;
      m_rudderAngle = new FrLinearRampFunction(GetSystem()->GetTime(), actualRudderAngle,
                                               GetSystem()->GetTime() + t_ramp, angle);
    }
  }

  double FrRudderForce::GetRudderAngle(ANGLE_UNIT unit) const {
    auto angle = m_rudderAngle->Get_y(GetSystem()->GetTime());
    if (unit == DEG) angle *= RAD2DEG;
    return angle;
  }

  void FrRudderForce::SetStraightRunWakeFraction(double w0) {
    m_wakeFraction0 = w0;
  }

  double FrRudderForce::GetStraightRunWakeFraction() const {
    return m_wakeFraction0;
  }

  void FrRudderForce::SetTransverseVelocityCorrection(double k) {
    m_k = k;
  }

  Position FrRudderForce::GetPositionInBody() const {
    return m_rudderNode->GetNodePositionInBody(NWU);
  }

  FrFrame FrRudderForce::GetRudderFlowFrame(const Velocity &rudderFlowVelocity) const {

    auto x = rudderFlowVelocity;
    x.normalize();
    auto z = m_rudderNode->GetFrameInWorld().GetZAxisInParent(NWU);
    z.normalize();
    auto y = z.cross(x);
    if (y.isZero(1E-3)) {
      event_logger::error("FrRudderForce", GetName(), "rudderFlowVelocity is along Rudder axis");
      return FrFrame();
    }
    y.normalize();
    x = y.cross(z);
    x.normalize();

    FrFrame rudderFlowFrame;
    rudderFlowFrame.Set(Position(), x, y, z, NWU);

    return rudderFlowFrame;
  }

  double FrRudderForce::GetAttackAngle(const Velocity &rudderVelocity) const {
    return GetRudderAngle(RAD) - GetDriftAngle(rudderVelocity);
  }

  double FrRudderForce::GetDriftAngle(const Velocity &rudderVelocity) const {
    return GetBody()->ProjectVectorInBody(rudderVelocity, NWU).GetProjectedAngleAroundZ(RAD);
  }

  void FrRudderForce::Compute(double time) {

    auto rudderRelativeVelocityInWorld = GetRudderRelativeVelocityInWorld();

    auto rudderGeneralizedForceInWorld = ComputeGeneralizedForceInWorld(rudderRelativeVelocityInWorld);

    SetForceTorqueInWorldAtPointInBody(rudderGeneralizedForceInWorld.GetForce(),
                                       rudderGeneralizedForceInWorld.GetTorque(),
                                       GetPositionInBody(), NWU);

  }

  GeneralizedForce FrRudderForce::ComputeGeneralizedForceInWorld(const Velocity &rudderRelativeVelocity, double kd) const {
    GeneralizedForce rudderForce;
    rudderForce.SetNull();

    if (not rudderRelativeVelocity.isZero(1E-3)) {

      auto attackAngle = GetAttackAngle(rudderRelativeVelocity);
      attackAngle = mathutils::Normalize__PI_PI(attackAngle);

      // projection of the rudder velocity in the body COG reference frame
      auto rudderVelocityInBody = GetBody()->ProjectVectorInBody(rudderRelativeVelocity, NWU);
      // vertical component in the body reference frame should not be considered
      rudderVelocityInBody.GetVz() = 0.;
      auto squaredNormVelocity = rudderVelocityInBody.norm();
      squaredNormVelocity *= squaredNormVelocity;

      auto density = GetBody()->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

      c_drag = 0.5 * density * GetDragCoefficient(attackAngle) * m_projectedLateralArea * squaredNormVelocity;
      c_lift = 0.5 * density * GetLiftCoefficient(attackAngle) * m_projectedLateralArea * squaredNormVelocity * kd;
      c_torque = 0.5 * density * GetTorqueCoefficient(attackAngle) * m_projectedLateralArea * m_rootChord * squaredNormVelocity;

      auto rudderFlowFrame = GetRudderFlowFrame(rudderRelativeVelocity);

      rudderForce.SetForce(rudderFlowFrame.ProjectVectorFrameInParent(Force(c_drag, c_lift, 0.), NWU));
      rudderForce.SetTorque(rudderFlowFrame.ProjectVectorFrameInParent(Torque(0., 0., c_torque), NWU));

    }
    return rudderForce;
  }

  double FrRudderForce::ComputeSpecialSidewashAngle() const {
    auto vesselVelocity = GetBody()->GetVelocityInHeadingFrame(NWU);

    auto r = GetBody()->GetAngularVelocityInWorld(NWU).GetWz();
    auto x_r = GetPositionInBody().GetX() + GetBody()->GetCOG(NWU).GetX();

    vesselVelocity.GetVy() += +m_k * r * x_r;

    return vesselVelocity.GetProjectedAngleAroundZ(RAD);
  }

  double FrRudderForce::Kappa(double specialSidewashAngle) const {
    auto beta = std::abs(specialSidewashAngle);
    double kappa;
    if (beta < m_beta1) {
      kappa = std::min(m_K2, m_K3 * beta);
    } else if (beta <= m_beta2 and beta >= m_beta1) {
      auto bv = (1.-m_K2) / (m_beta2 - m_beta1);
      auto av = m_K2 - bv * m_beta1;
      kappa = av + bv * beta;
    } else {
      kappa = 1.0;
    }
    return kappa;
  }

  double FrRudderForce::GetLiftCoefficient(double attackAngle) const {
    return m_coefficients.Eval("lift", attackAngle);
  }

  double FrRudderForce::GetDragCoefficient(double attackAngle) const {
    return m_coefficients.Eval("drag", attackAngle);
  }

  double FrRudderForce::GetTorqueCoefficient(double attackAngle) const {
    return m_coefficients.Eval("torque", attackAngle);
  }

  double FrRudderForce::GetDrag() const {
    return c_drag;
  }

  double FrRudderForce::GetLift() const {
    return c_lift;
  }

  double FrRudderForce::GetTorque() const {
    return c_torque;
  }

  void FrRudderForce::ReadCoefficientsFile() {
    // This function reads the rudder rudderCoeff coefficients from a Json input file.

    std::vector<double> attack_angle, Cd, Cl, Cn;
    FRAME_CONVENTION fc;
    DIRECTION_CONVENTION dc;

    // Loader.
    std::ifstream ifs(c_fileCoefficients);
    json j = json::parse(ifs);

    auto node = j["rudder"];

    try {
      m_reference = node["reference"].get<json::string_t>();
    } catch (json::parse_error &err) {
      event_logger::error("FrRudderForce", GetName(), "no reference in json file");
      exit(EXIT_FAILURE);
    }
//
//    try {
//      m_rootChord = node["data"]["chord_length_m"].get<double>();
//    } catch (json::parse_error &err) {
//      event_logger::error("FrRudderForce", GetName(), "no chord_length_m in json file");
//      exit(EXIT_FAILURE);
//    }

    try {
      fc = STRING2FRAME(node["load_coefficients"]["frame_convention"].get<json::string_t>());
    } catch (json::parse_error &err) {
      event_logger::error("FrRudderForce", "", "no frame_convention in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      dc = STRING2DIRECTION(node["load_coefficients"]["direction_convention"].get<json::string_t>());
    } catch (json::parse_error &err) {
      event_logger::error("FrRudderForce", "", "no direction_convention in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      attack_angle = node["load_coefficients"]["flow_incidence_on_main_rudder_deg"].get<std::vector<double>>();
      for (auto &angle:attack_angle) {
        angle *= DEG2RAD;
      }
    } catch (json::parse_error &err) {
      event_logger::error("FrRudderForce", "", "no flow_incidence_on_main_rudder_deg in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      Cd = node["load_coefficients"]["Cd"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrRudderForce", "", "no Cd in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      Cl = node["load_coefficients"]["Cl"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrRudderForce", "", "no Cl in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      Cn = node["load_coefficients"]["Cn"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrRudderForce", "", "no Cn in load_coefficients");
      exit(EXIT_FAILURE);
    }

//    assert(attack_angle.size() == Cd.size());
//    assert(attack_angle.size() == Cl.size());
//    assert(attack_angle.size() == Cn.size());
//    std::vector<std::pair<double, mathutils::Vector3d<double>>> rudderCoeff;
//
//    for (int i = 0; i < attack_angle.size(); i++) {
//      rudderCoeff.emplace_back(attack_angle[i], mathutils::Vector3d<double>(Cd[i], Cl[i], Cn[i]));
//    }
//
//    std::pair<double, mathutils::Vector3d<double>> new_element;
//    // Complete if symmetry
//    auto max_angle = rudderCoeff.back().first;
//    auto min_angle = rudderCoeff[0].first;
//
//    if (std::abs(min_angle) < 10e-2 and std::abs(max_angle - MU_PI) < 10e-2) {
//      for (unsigned int i = rudderCoeff.size() - 2; i >= 1; i--) {
//        new_element.first = 2. * MU_PI - rudderCoeff[i].first;
//        new_element.second = {rudderCoeff[i].second[0], -rudderCoeff[i].second[1], -rudderCoeff[i].second[2]};
//        rudderCoeff.push_back(new_element);
//      }
//    } else if (std::abs(min_angle + MU_PI) < 10e-2 and std::abs(max_angle) < 10e-2) {
//      for (unsigned int i = rudderCoeff.size() - 2; i >= 1; i--) {
//        new_element.first = -rudderCoeff[i].first;
//        new_element.second = {rudderCoeff[i].second[0], -rudderCoeff[i].second[1], -rudderCoeff[i].second[2]};
//        rudderCoeff.push_back(new_element);
//      }
//    }
//
//    // Delete double term
//    if (std::abs(rudderCoeff[0].first) < 10e-2 and std::abs(rudderCoeff.back().first - 2. * MU_PI) < 10e-2
//                                                   or std::abs(rudderCoeff[0].first + MU_PI) < 10e-2 and
//        std::abs(rudderCoeff.back().first - MU_PI) < 10e-2) {
//      rudderCoeff.pop_back();
//    }
//
//    // Conversion to NWU if NED convention is used
//    if (fc == NED) {
//      for (auto &it : rudderCoeff) {
//        it.first = -it.first;
//        it.second = {it.second[0], -it.second[1], -it.second[2]};
//      }
//    }
//
//    // Conversion to COMEFROM if GOTO convention is used
//    if (dc == GOTO) {
//      for (auto &it : rudderCoeff) { it.first += MU_PI; }
//    }
//
//    // Normalized angle in [0, 2pi]
//    for (auto &it : rudderCoeff) { it.first = mathutils::Normalize_0_2PI(it.first); }
//
//    // Sort element according to increasing angles
//    std::sort(rudderCoeff.begin(), rudderCoeff.end(), [](auto const &a, auto const &b) {
//      return a.first < b.first;
//    });
//
//    // Adding last term for angle equal to 2pi
//    new_element.first = 2. * MU_PI;
//    new_element.second = rudderCoeff.begin()->second;
//    rudderCoeff.push_back(new_element);
//
//    // Complete lookup table
//    attack_angle.clear();
//    Cl.clear();
//    Cd.clear();
//    Cn.clear();
//
//    for (auto &it : rudderCoeff) {
//      attack_angle.push_back(it.first);
//      Cd.push_back(it.second[0]);
//      Cl.push_back(it.second[1]);
//      Cn.push_back(it.second[2]);
//    }

    m_coefficients.SetX(attack_angle);
    m_coefficients.AddY("drag", Cd);
    m_coefficients.AddY("lift", Cl);
    m_coefficients.AddY("torque", Cn);


  }

  void FrRudderForce::DefineLogMessages() {

    auto msg = NewMessage("FrForce", "Rudder force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    msg->AddField<double>("RudderAngle", "rad", "Rudder angle",
                          [this]() { return this->GetRudderAngle(RAD); });

    msg->AddField<double>("DriftAngle", "rad", "Drift angle",
                          [this]() { return this->GetDriftAngle(GetRudderRelativeVelocityInWorld()); });

    msg->AddField<double>("AttackAngle", "rad", "Attack angle",
                          [this]() { return this->GetAttackAngle(GetRudderRelativeVelocityInWorld()); });

    msg->AddField<double>("Drag", "N", "Drag delivered by the rudder",
                          [this]() { return this->GetDrag(); });

    msg->AddField<double>("Lift", "N", "Lift delivered by the rudder",
                          [this]() { return this->GetLift(); });

    msg->AddField<double>("Torque", "Nm", "Torque delivered by the rudder",
                          [this]() { return this->GetTorque(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("RudderRelativeVelocityInWorld", "m/s", fmt::format("rudder velocity relative to the surrounding flow, in the World reference frame in {}", GetLogFC()),
         [this]() { return GetRudderRelativeVelocityInWorld(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInBody", "N", fmt::format("force in body reference frame in {}", GetLogFC()),
         [this]() { return GetForceInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtCOG", "Nm", fmt::format("torque at COG in body reference frame in {}", GetLogFC()),
         [this]() { return GetTorqueInBodyAtCOG(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInWorld", "N", fmt::format("force in world reference frame in {}", GetLogFC()),
         [this]() { return GetForceInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtCOG", "Nm", fmt::format("torque at COG in world reference frame in {}", GetLogFC()),
         [this]() { return GetTorqueInWorldAtCOG(GetLogFC()); });

  }

  std::shared_ptr<FrRudderForce>
  make_rudder_force(const std::string &name, const std::shared_ptr<FrBody> &body, const std::shared_ptr<FrNode> &node,
                    const std::string &fileCoefficients) {
    auto force = std::make_shared<FrRudderForce>(name, body.get(), node, fileCoefficients);
    body->AddExternalForce(force);
    return force;
  }

} // end namespace frydom