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


#include "FrFlowForce.h"

#include "frydom/core/common/FrUnits.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/io/FrLoader.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/logging/FrTypeNames.h"


namespace frydom {

  FrFlowForce::FrFlowForce(const std::string& name,
                           const std::string& type_name,
                           FrBody* body, double frontal_area_m, double lateral_area_m, double length_m,
                           const std::vector<double>& angles, const std::vector<double>& cx,
                           const std::vector<double>& cy, const std::vector<double>& cn,
                           ANGLE_UNIT unit_angle, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) :
      FrForce(name, type_name, body) ,
      m_table(mathutils::LINEAR),
      m_frontal_area(frontal_area_m),
      m_lateral_area(lateral_area_m),
      m_length(length_m) {

    std::vector<std::pair<double, mathutils::Vector3d<double>>> polar;

    auto n = angles.size();
    assert(cx.size() == n);
    assert(cy.size() == n);
    assert(cn.size() == n);

    std::pair<double, mathutils::Vector3d<double>> new_element;
    for (int i = 0; i < angles.size(); i++) {
      polar.push_back(
          std::pair<double, mathutils::Vector3d<double>>(angles[i], mathutils::Vector3d<double>(cx[i], cy[i], cn[i])));
    }

    AdaptPolar(polar, unit_angle, fc, dc);

    // Complete lookup table
    std::vector<double> anglesL;
    std::vector<mathutils::Vector3d<double>> vectL;

    for (auto it = polar.begin(); it != polar.end(); ++it) {
      anglesL.push_back(it->first);
      vectL.push_back(it->second);
    }

    m_table.SetX(anglesL);
    m_table.AddY("coeff", vectL);

  }

  FrFlowForce::FrFlowForce(const std::string &name,
                           const std::string &type_name,
                           FrBody *body,
                           const std::string &jsonFile) :
      FrForce(name, type_name, body),
      m_table(mathutils::LINEAR),
      m_frontal_area(1.),
      m_lateral_area(1.),
      m_length(1.){

    this->ReadTable(jsonFile);
  }

  void FrFlowForce::ReadTable(const std::string &jsonFile) {

    std::vector<std::pair<double, mathutils::Vector3d<double>>> polar;
    ANGLE_UNIT angle_unit;
    FRAME_CONVENTION fc;
    DIRECTION_CONVENTION dc;

    LoadFlowPolarCoeffFromJson(jsonFile,
                               polar,
                               m_frontal_area, m_lateral_area, m_length,
                               angle_unit, fc, dc);

    AdaptPolar(polar, angle_unit, fc, dc);

    // Complete lookup table
    std::vector<double> anglesL;
    std::vector<mathutils::Vector3d<double>> vectL;

    for (auto it = polar.begin(); it != polar.end(); ++it) {
      anglesL.push_back(it->first);
      vectL.push_back(it->second);
    }

    m_table.SetX(anglesL);
    m_table.AddY("coeff", vectL);
  }

  void FrFlowForce::AdaptPolar(std::vector<std::pair<double, mathutils::Vector3d<double>>>& polar,
                               ANGLE_UNIT& angle_unit, FRAME_CONVENTION& fc, DIRECTION_CONVENTION& dc) {

    std::pair<double, mathutils::Vector3d<double>> new_element;

    if (angle_unit == mathutils::DEG) {
      for (auto it = polar.begin(); it != polar.end(); ++it) { it->first *= DEG2RAD; }
    }

    // Complete if symmetry
    auto max_angle = polar.back().first;
    auto min_angle = polar[0].first;

    if (std::abs(min_angle) < 10e-2 and std::abs(max_angle - MU_PI) < 10e-2) {
      for (unsigned int i = polar.size() - 2; i >= 1; i--) {
        new_element.first = 2. * MU_PI - polar[i].first;
        new_element.second = {polar[i].second[0], -polar[i].second[1], -polar[i].second[2]};
        polar.push_back(new_element);
      }
    } else if (std::abs(min_angle + MU_PI) < 10e-2 and std::abs(max_angle) < 10e-2) {
      for (unsigned int i = polar.size() - 2; i >= 1; i--) {
        new_element.first = -polar[i].first;
        new_element.second = {polar[i].second[0], -polar[i].second[1], -polar[i].second[2]};
        polar.push_back(new_element);
      }
    }

    // Delete double term
    if (std::abs(polar[0].first) < 10e-2 and std::abs(polar.back().first - 2. * MU_PI) < 10e-2
                                             or std::abs(polar[0].first + MU_PI) < 10e-2 and
        std::abs(polar.back().first - MU_PI) < 10e-2) {
      polar.pop_back();
    }

    // Conversion to NWU if NED convention is used
    if (fc == NED) {
      for (auto it = polar.begin(); it != polar.end(); ++it) {
        it->first = -it->first;
        it->second = {it->second[0], -it->second[1], -it->second[2]};
      }
    }

    // Conversion to GOTO if COMEFROM convention is used
    if (dc == COMEFROM) {
      for (auto it = polar.begin(); it != polar.end(); ++it) { it->first += MU_PI; }
    }

    // Normalized angle in [0, 2pi]
    for (auto it = polar.begin(); it != polar.end(); ++it) { it->first = mathutils::Normalize_0_2PI(it->first); }

    // Sort element according to increasing angles
    std::sort(polar.begin(), polar.end(), [](auto const &a, auto const &b) {
      return a.first < b.first;
    });

    // Ensuring first term is 0
    assert(polar[0].first <= 1E-6);
    polar[0].first = 0;

    // Adding last term for angle equal to 2pi
    new_element.first = 2. * MU_PI;
    new_element.second = polar.begin()->second;
    polar.push_back(new_element);
  }

  void FrFlowForce::Compute(double time) {

    auto body = GetBody();

    double alpha = GetProjectedAngleAroundZ(m_fluxVelocityInBody, mathutils::RAD);
    alpha = mathutils::Normalize_0_2PI(alpha);

    auto coeff = m_table.Eval("coeff", alpha);
    double cx = coeff[0];
    double cy = coeff[1];
    double cn = coeff[2];

    double SquaredVelocity = m_fluxVelocityInBody.squaredNorm();
    double rho = GetFluidDensity();

    double fx = 0.5 * rho * cx * m_frontal_area * SquaredVelocity;
    double fy = 0.5 * rho * cy * m_lateral_area * SquaredVelocity;
    double mz = 0.5 * rho * cn * m_lateral_area * m_length * SquaredVelocity;

    // Build the projected rotation in the XoY plane.
    double phi, theta, psi;
    body->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
    auto bodyRotation = FrRotation(Direction(0., 0., 1.), psi, NWU);
    auto frame = FrFrame(body->GetCOGPositionInWorld(NWU), bodyRotation, NWU);

//        auto frame = m_body->GetFrameAtCOG(NWU).ProjectToXYPlane(NWU);
    auto worldForce = frame.ProjectVectorFrameInParent(Force(fx, fy, 0), NWU);
    auto worldTorque = frame.ProjectVectorFrameInParent(Torque(0., 0., mz), NWU);

    SetForceTorqueInWorldAtCOG(worldForce, worldTorque, NWU);
  }

  void FrCurrentForce::Compute(double time) {

    auto body = GetBody();

    FrFrame FrameAtCOG = body->GetFrameAtCOG();
    Velocity VelocityInWorldAtCOG = body->GetCOGLinearVelocityInWorld(NWU);

    m_fluxVelocityInBody =
        body->GetSystem()->GetEnvironment()->GetOcean()->GetCurrent()->GetFluxRelativeVelocityInFrame(FrameAtCOG,
                                                                                                      VelocityInWorldAtCOG,
                                                                                                      NWU);
    FrFlowForce::Compute(time);
  }

  FrCurrentForce::FrCurrentForce(const std::string &name,
                                 FrBody *body,
                                 const std::string &jsonFile) :
      FrFlowForce(name, TypeToString(this), body, jsonFile) {}

  FrCurrentForce::FrCurrentForce(const std::string& name, FrBody* body,
                                 double frontal_area_m, double lateral_area_m, double length_m,
                                 const std::vector<double>& angles, const std::vector<double>& cx,
                                 const std::vector<double>& cy, const std::vector<double>& cn,
                                 ANGLE_UNIT unit_angle, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc)
     : FrFlowForce(name, TypeToString(this), body, frontal_area_m, lateral_area_m, length_m, angles,
                   cx, cy, cn, unit_angle, fc, dc) {}

  double FrCurrentForce::GetFluidDensity() const {
    return GetBody()->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();
  }

  void FrWindForce::Compute(double time) {

    auto body = GetBody();

    FrFrame FrameAtCOG = body->GetFrameAtCOG();
    Velocity VelocityInWorldAtCOG = body->GetCOGLinearVelocityInWorld(NWU);

    m_fluxVelocityInBody =
        body->GetSystem()->GetEnvironment()->GetAtmosphere()->GetWind()->GetFluxRelativeVelocityInFrame(FrameAtCOG,
                                                                                                        VelocityInWorldAtCOG,
                                                                                                        NWU);

    FrFlowForce::Compute(time);
  }

  FrWindForce::FrWindForce(const std::string &name,
                           FrBody *body,
                           const std::string &jsonFile) :
      FrFlowForce(name, TypeToString(this), body, jsonFile) {}

  FrWindForce::FrWindForce(const std::string& name, FrBody* body,
                           double frontal_area_m, double lateral_area_m, double length_m,
                           const std::vector<double>& angles, const std::vector<double>& cx,
                           const std::vector<double>& cy, const std::vector<double>& cn,
                           ANGLE_UNIT unit_angle, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) :
      FrFlowForce(name, TypeToString(this), body, frontal_area_m, lateral_area_m, length_m, angles,
                  cx, cy, cn, unit_angle, fc, dc) {}

  double FrWindForce::GetFluidDensity() const {
    return GetBody()->GetSystem()->GetEnvironment()->GetAtmosphere()->GetDensity();
  }

  std::shared_ptr<FrCurrentForce> make_current_force(const std::string &name,
                                                     std::shared_ptr<FrBody> body,
                                                     const std::string &jsonFile) {
    auto currentForce = std::make_shared<FrCurrentForce>(name, body.get(), jsonFile);
    body->AddExternalForce(currentForce);
    return currentForce;
  }

  std::shared_ptr<FrCurrentForce> make_current_force(const std::string& name,
                                                     std::shared_ptr<FrBody> body,
                                                     double frontal_area_m, double lateral_area_m, double length_m,
                                                     const std::vector<double>& angles,
                                                     const std::vector<double>& cx, const std::vector<double>& cy,
                                                     const std::vector<double>& cn, ANGLE_UNIT unit_angle,
                                                     FRAME_CONVENTION fc, DIRECTION_CONVENTION dc)
  {
    auto currentForce = std::make_shared<FrCurrentForce>(name, body.get(), frontal_area_m, lateral_area_m, length_m,
                                                         angles, cx, cy, cn, unit_angle, fc, dc);
    body->AddExternalForce(currentForce);
    return currentForce;
  }

  std::shared_ptr<FrWindForce> make_wind_force(const std::string &name,
                                               std::shared_ptr<FrBody> body,
                                               const std::string &jsonFile) {
    auto windForce = std::make_shared<FrWindForce>(name, body.get(), jsonFile);
    body->AddExternalForce(windForce);
    return windForce;
  }

  std::shared_ptr<FrWindForce> make_wind_force(const std::string& name, std::shared_ptr<FrBody> body,
                                               double frontal_area_m, double lateral_area_m, double length_m,
                                               const std::vector<double>& angles,
                                               const std::vector<double>& cx, const std::vector<double>& cy,
                                               const std::vector<double>& cn, ANGLE_UNIT unit_angle,
                                               FRAME_CONVENTION fc, DIRECTION_CONVENTION dc)
  {
    auto windForce = std::make_shared<FrWindForce>(name, body.get(), frontal_area_m, lateral_area_m, length_m,
                                                   angles, cx, cy, cn, unit_angle, fc, dc);
    body->AddExternalForce(windForce);
    return windForce;
  }


} // end of namespace frydom
