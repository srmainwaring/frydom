//
// Created by frongere on 15/04/2020.
//

#include "FrFEACableLoads.h"

#include "frydom/core/math/FrVector.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironmentInc.h"

#include "frydom/cable/common/FrCableProperties.h"
#include "FrFEACableElement.h"
#include "FrFEACable.h"


namespace frydom {

  namespace internal {

    FrFEAloaderBase::FrFEAloaderBase(std::shared_ptr<chrono::ChLoadableU> loadable) :
        chrono::ChLoaderUdistributed(loadable),
        m_nb_integration_points(1) {

    }// TODO: parametrer le nb de points

    int FrFEAloaderBase::GetIntegrationPointsU() {
      return m_nb_integration_points;
    }

    void FrFEAloaderBase::SetNbIntegrationPoints(int n) {
      m_nb_integration_points = n;
    }

    void FrFEAloaderBase::SetSystem(FrOffshoreSystem *system) {
      m_system = system;
    }

    FrFEACableHydroLoader::FrFEACableHydroLoader(std::shared_ptr<chrono::ChLoadableU> loadable) :
        FrFEAloaderBase(loadable) {

    }

    void FrFEACableHydroLoader::ComputeF(const double U,
                                         chrono::ChVectorDynamic<> &F,
                                         chrono::ChVectorDynamic<> *state_x,
                                         chrono::ChVectorDynamic<> *state_w) {

      // FIXME: il faudra templater l'element
      auto element = std::dynamic_pointer_cast<internal::FrFEACableElementBase>(loadable);

      // Get the point position
      chrono::ChVector<double> position_;
      chrono::ChQuaternion<double> quaternion_;

      element->EvaluateSectionFrame(U, position_, quaternion_);

      auto position = internal::ChVectorToVector3d<Position>(position_);


      bool m_include_waves = true; // TODO: faire en sorte que le calcul des orbitales soit fait

      // Buoyancy

      double gravity = m_system->GetGravityAcceleration();
      auto fluid_type = m_system->GetEnvironment()->GetFluidTypeAtPointInWorld(position, NWU, m_include_waves);
      auto fluid_density = m_system->GetEnvironment()->GetFluidDensity(fluid_type);

//      element->GetSection()->Set

      double section_area = m_cable->GetProperties()->GetSectionArea(); // TODO: voir si on ne prend pas un diameter hydro plutot...
      Force unit_force = {0., 0., fluid_density * section_area * gravity};


      // Morison

      // FIXME:

      // Relative fluid velocity induced by cable motion at U
      chrono::ChVector<double> vel;
      element->EvaluateSectionSpeed(U, vel);
      Velocity fluid_relative_velocity = -internal::ChVectorToVector3d<Velocity>(vel);

      // Relative fluid acceleration induces by cable motion at U
      chrono::ChVector<double> acc;
      element->EvaluateSectionAcceleration(U, acc);
      Acceleration fluid_relative_acceleration = -internal::ChVectorToVector3d<Acceleration>(acc);


      // Environment fluid velocity at U
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


//      fluid_relative_velocity -= cable_velocity;
//      fluid_relative_acceleration -= cable_acceleration;

      // Getting the tangent direction of cable
      Direction tangent_direction = internal::ChVectorToVector3d<Direction>(quaternion_.GetXaxis());

      // Getting tangential and normal relative fluid velocities
      Velocity tangent_fluid_velocity = fluid_relative_velocity.dot(tangent_direction) * tangent_direction;
      Velocity normal_fluid_velocity = fluid_relative_velocity - tangent_fluid_velocity;

      // Computing morison load
      auto cable_properties = m_cable->GetProperties();
      double d = cable_properties->GetHydrodynamicDiameter(); // FIXME: permettre d'avoir un diametre materiel et un diametre hydro !!!
//      double A = 0.25 * MU_PI * d * d;

      /*
       * Morison drag
       */

      Force morison_force;

      // Normal morison drag
      morison_force += 0.5 * fluid_density * cable_properties->GetTransverseDragCoefficient() * d *
                       normal_fluid_velocity.norm() * normal_fluid_velocity;

      // Tangent morison drag
      morison_force += 0.5 * fluid_density * cable_properties->GetTangentialDragCoefficient() * d *
                       tangent_fluid_velocity.norm() * tangent_fluid_velocity;


//      /*
//       * Morison added mass effects
//       */
//      Acceleration tangent_fluid_acceleration = fluid_relative_acceleration.dot(tangent_direction) * tangent_direction;
//      Acceleration normal_fluid_acceleration = fluid_relative_acceleration - tangent_fluid_acceleration;
//
//      // Normal morison added mass
//      morison_force += fluid_density * cable_properties->GetTransverseAddedMassCoefficient() * A *
//                       normal_fluid_acceleration.norm() * normal_fluid_acceleration;
//
//      // Tangent Morison added mass
//      morison_force += fluid_density * cable_properties->GetTangentialAddedMassCoefficient() * A *
//                       tangent_fluid_acceleration.norm() * tangent_fluid_acceleration;


      unit_force += morison_force;

//      std::cout << unit_force << std::endl;

      // Pasting the results

      F.PasteVector(internal::Vector3dToChVector(unit_force), 0, 0);   // load, force part
      F.PasteVector(chrono::VNULL, 3, 0);  // No torque

    }

    void FrFEACableHydroLoader::SetCable(FrFEACable *cable) {
      m_cable = cable;
      SetSystem(cable->GetSystem());
    }

  }  // end namespace frydom::internal

}  // end namespace frydom
