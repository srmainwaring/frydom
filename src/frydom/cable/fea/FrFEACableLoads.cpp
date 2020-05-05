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

//    void FrFEAloaderBase::SetSystem(FrOffshoreSystem *system) {
//      m_system = system;
//    }

    FrFEACableHydroLoader::FrFEACableHydroLoader(std::shared_ptr<chrono::ChLoadableU> loadable) :
        FrFEAloaderBase(loadable) {

    }

    void FrFEACableHydroLoader::ComputeF(const double U,
                                         chrono::ChVectorDynamic<> &F,
                                         chrono::ChVectorDynamic<> *state_x,
                                         chrono::ChVectorDynamic<> *state_w) {

      auto element = dynamic_cast<internal::FrFEACableElementBase*>(loadable.get());


      // Get the point position
      chrono::ChVector<double> position_;
      chrono::ChQuaternion<double> quaternion_;

      element->EvaluateSectionFrame(U, position_, quaternion_);

      auto position = internal::ChVectorToVector3d<Position>(position_);

      // Getting the tangent direction of cable at U
      chrono::ChVector<double> ut = quaternion_.GetXaxis();




      bool m_include_waves = true; // TODO: faire en sorte que le calcul des orbitales soit fait


      double gravity = element->m_environment->GetGravityAcceleration();
      auto fluid_type = element->m_environment->GetFluidTypeAtPointInWorld(position, NWU, m_include_waves);
      auto fluid_density = element->m_environment->GetFluidDensity(fluid_type);
//      auto cable_properties = m_cable->GetProperties();
//      double d = cable_properties->GetHydrodynamicDiameter(); // FIXME: permettre d'avoir un diametre materiel et un diametre hydro !!!

      double d = element->GetSection()->Get


      /*
       * Computing fluid velocities
       */

      chrono::ChVector<double> vf;
      chrono::ChVector<double> vf_dt;

      vf.SetNull();
      vf_dt.SetNull();

      if (fluid_type == WATER) {
        auto ocean = element->m_environment->GetOcean();

        // Current
        auto current = ocean->GetCurrent();
        vf += internal::Vector3dToChVector(current->GetFluxVelocityInWorld(position, NWU));

        // Wave orbital velocities
        if (m_include_waves) {
          auto wave_field = ocean->GetFreeSurface()->GetWaveField();
          vf_dt += internal::Vector3dToChVector(wave_field->GetVelocity(position, NWU));
          vf_dt += internal::Vector3dToChVector(wave_field->GetAcceleration(position, NWU));
        }

      } else {  // AIR
        auto wind = element->m_environment->GetAtmosphere()->GetWind();
        vf += internal::Vector3dToChVector(wind->GetFluxVelocityInWorld(position, NWU));
      }

      /*
       * Cable velocity
       */
      chrono::ChVector<double> cable_velocity;
      element->EvaluateSectionSpeed(U, cable_velocity);


      // Getting tangential and normal relative fluid velocities
      chrono::ChVector<double> vf_t = vf.Dot(ut) * ut;
      chrono::ChVector<double> vf_n = vf - vf_t;

      chrono::ChVector<double> vf_t_dt = vf_dt.Dot(ut) * ut;
      chrono::ChVector<double> vf_n_dt = vf_dt - vf_t;

      chrono::ChVector<double> v = vf - cable_velocity;
      chrono::ChVector<double> v_t = v.Dot(ut) * ut;
      chrono::ChVector<double> v_n = v - v_t;


      /*
       * Buoyancy
       */

      double section_area = m_cable->GetProperties()->GetSectionArea(); // TODO: voir si on ne prend pas un diameter hydro plutot...
      chrono::ChVector<double> buoyancy_unit_force = {0., 0., fluid_density * section_area * gravity};

      /*
       * Morison drag
       */

      chrono::ChVector<double> morison_drag;
      morison_drag.SetNull();

      double v2 = v.Length2(); // TODO: verifier
      morison_drag += -0.5 * fluid_density * d * Cd * v2 * v_n / v_n.GetNormalized() * fn;
      morison_drag += -0.5 * fluid_density * d * Cd * v2 * v_t / v_t.GetNormalized() * ft * sign(v_t.Dot(ut));

      /*
       * VIV drag amplification
       */
      chrono::ChVector<double> drag_viv;
      drag_viv.SetNull();

      double vf2 = vf.Length2(); // TODO: verifier
      morison_drag += -0.5 * fluid_density * d * Cd * v2 * vf_n / vf_n.GetNormalized() * fn;
      morison_drag += -0.5 * fluid_density * d * Cd * v2 * vf_t / vf_t.GetNormalized() * ft * sign(vf_t.Dot(ut));

      /*
       * Morison added mass effects
       */

      chrono::ChVector<double> morison_inertia;
      morison_inertia = fluid_density * section_area * (Cm + 1) * vf_n_dt;


      /*
       * Summing up contributions
       */

      chrono::ChVector<double> unit_force = buoyancy_unit_force + morison_drag + morison_inertia + drag_viv;


      // Pasting the results
      F.PasteVector(unit_force, 0, 0);   // load, force part
      F.PasteVector(chrono::VNULL, 3, 0);  // No torque

    }

    void FrFEACableHydroLoader::SetCable(FrFEACable *cable) {
      m_cable = cable;
    }

  }  // end namespace frydom::internal

}  // end namespace frydom
