//
// Created by frongere on 15/04/2020.
//

#include "FrFEACableLoads.h"

#include "FrCableProperties.h"


namespace frydom {


  FrFEACableLoader::FrFEACableLoader(std::shared_ptr<chrono::ChLoadableU> mloadable) :
      m_cable(nullptr),
      m_nb_integration_points(1),
      chrono::ChLoaderUdistributed(mloadable) {}

  void FrFEACableLoader::SetCable(FrFEACable *cable) {
    m_cable = cable;
    m_system = cable->GetSystem();
  }

  void FrFEACableLoader::SetNbIntegrationPoints(const unsigned int n) {
    m_nb_integration_points = int(n);
  }

  int FrFEACableLoader::GetIntegrationPointsU() { return m_nb_integration_points; }


  /*
   * Buoyancy loader
   */

  FrBuoyancyLoader::FrBuoyancyLoader(std::shared_ptr<chrono::ChLoadableU> mloadable) :
      FrFEACableLoader(mloadable) {}

  void FrBuoyancyLoader::ComputeF(const double U, chrono::ChVectorDynamic<> &F,
                                  chrono::ChVectorDynamic<> *state_x,
                                  chrono::ChVectorDynamic<> *state_w) {  ///< if != 0, update state (speed part) to this, then evaluate F)

    // FIXME: il faudra templater l'element
    auto element = std::dynamic_pointer_cast<internal::FrElementBeamIGA>(loadable);

    // Get the point position
    chrono::ChVector<double> position_;
    chrono::ChQuaternion<double> quaternion_;

    element->EvaluateSectionFrame(U, position_, quaternion_);

    auto position = internal::ChVectorToVector3d<Position>(position_);


    bool m_include_waves = true;

    // Buoyancy

    double gravity = m_system->GetGravityAcceleration();
    auto fluid_type = m_system->GetEnvironment()->GetFluidTypeAtPointInWorld(position, NWU, m_include_waves);
    auto fluid_density = m_system->GetEnvironment()->GetFluidDensity(fluid_type);

    double section = m_cable->GetProperties()->GetSectionArea(); // TODO: voir si on ne prend pas un diameter hydro plutot...
    Force unit_force = {0., 0., fluid_density * section * gravity};


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
    if (fluid_type == WATER) {
      auto ocean = m_system->GetEnvironment()->GetOcean();

      // Current
      fluid_relative_velocity += ocean->GetCurrent()->GetFluxVelocityInWorld(position, NWU);

      // Wave orbital velocities
      if (m_include_waves) {
        auto wave_field = ocean->GetFreeSurface()->GetWaveField();
        fluid_relative_velocity += wave_field->GetVelocity(position, NWU);
        fluid_relative_acceleration += wave_field->GetAcceleration(position, NWU);
      }

    } else {  // AIR
      fluid_relative_velocity +=
          m_system->GetEnvironment()->GetAtmosphere()->GetWind()->GetFluxVelocityInWorld(position, NWU);
    }

    // Taking the cable motion into account into the fluid motion
    fluid_relative_velocity -= cable_velocity;
    fluid_relative_acceleration -= cable_acceleration;

    // Getting the tangent direction of cable
    Direction tangent_direction = internal::ChVectorToVector3d<Direction>(quaternion_.GetXaxis());

    // Getting tangential and normal relative fluid velocities
    Velocity tangent_fluid_velocity = fluid_relative_velocity.dot(tangent_direction) * tangent_direction;
    Velocity normal_fluid_velocity = fluid_relative_velocity - tangent_fluid_velocity;

    // Computing morison load
    auto cable_properties = m_cable->GetProperties();
    double d = cable_properties->GetHydrodynamicDiameter();
    double A = 0.25 * MU_PI * d * d;

    Force morison_force;
    // Normal morison drag
    morison_force += 0.5 * fluid_density * cable_properties->GetTransverseDragCoefficient() * d *
                     normal_fluid_velocity.norm() * normal_fluid_velocity;

    // Tangent morison drag
    morison_force += 0.5 * fluid_density * cable_properties->GetTangentialDragCoefficient() * d *
                     tangent_fluid_velocity.norm() * tangent_fluid_velocity;

    Acceleration tangent_fluid_acceleration = fluid_relative_acceleration.dot(tangent_direction) * tangent_direction;
    Acceleration normal_fluid_acceleration = fluid_relative_acceleration - tangent_fluid_acceleration;

    // Normal morison
//    morison_force += fluid_density * cable_properties->GetTransverseAddedMassCoefficient() * A *
//        normal_fluid_acceleration.norm() * normal_fluid_acceleration;
//
//    morison_force += fluid_density * cable_properties->GetTangentialAddedMassCoefficient() * A *
//        tangent_fluid_acceleration.norm() * tangent_fluid_acceleration;


    unit_force += morison_force;



    // Pasting the results

    F.PasteVector(internal::Vector3dToChVector(unit_force), 0, 0);   // load, force part
    F.PasteVector(chrono::VNULL, 3, 0);  // No torque

  }

  FrBuoyancyLoad::FrBuoyancyLoad(FrFEACable *cable, std::shared_ptr<chrono::ChLoadableU> mloadable) :
      chrono::ChLoad<FrBuoyancyLoader>(mloadable) {
    loader.SetCable(cable);
  }

}  // end namespace frydom
