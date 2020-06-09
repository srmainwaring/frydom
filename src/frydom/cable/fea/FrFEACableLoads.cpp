//
// Created by frongere on 15/04/2020.
//

#include "FrFEACableLoads.h"

#include "frydom/core/math/FrVector.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironmentInc.h"

#include "frydom/cable/common/FrCableProperties.h"
#include "FrFEACableElement.h"
#include "FrFEACableSection.h"
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

    FrFEACableHydroLoader::FrFEACableHydroLoader(std::shared_ptr<chrono::ChLoadableU> loadable) :
        FrFEAloaderBase(loadable) {}

    void FrFEACableHydroLoader::ComputeF_(const double U,
                                         chrono::ChVectorDynamic<> &F,
                                         chrono::ChVectorDynamic<> *state_x,
                                         chrono::ChVectorDynamic<> *state_w) {

      // This implementation of cable hydrodynamics comes from:
      // Westin C., Modelling and Simulation of Marine Cables with Dynamic Winch and Sheave Contact, 2018,
      // Master thesis, Carleton University, Ottawa, Ontario.

      auto element = dynamic_cast<internal::FrFEACableElementBase *>(loadable.get());
      auto el_section = dynamic_cast<internal::FrFEACableSection *>(element->GetSection().get());

      // Get the point position
      chrono::ChVector<double> position_;
      chrono::ChQuaternion<double> quaternion_;

      element->EvaluateSectionFrame(U, position_, quaternion_);

      auto position = internal::ChVectorToVector3d<Position>(position_);

      // Getting the tangent direction of cable at U
      chrono::ChVector<double> tangent = quaternion_.GetXaxis();

      bool m_include_waves = true; // TODO: faire en sorte que le calcul des orbitales soit fait









      /// TEST POUR RECUPERER les fonctions de base

      auto knots = element->GetKnotSequence();

      // compute parameter in knot space from eta-1..+1
      double u1 = element->GetU1(); // extreme of span
      double u2 = element->GetU2();
      double u = u1 + ((U + 1) / 2.0) * (u2 - u1);
      int order = element->GetOrder();

      chrono::ChVectorDynamic<> N((int) element->GetNnodes());

      chrono::geometry::ChBasisToolsBspline::BasisEvaluate(
          order,
          order,
          u,
          knots,
          N);           ///< here return  in N

      // Les splines de base sont les N(i)




      /// TEST POUR RECUPERER les fonctions de base









      /*
       * Extracting parameters
       */

      // From environment
      auto environment = element->m_environment;

      double gravity = environment->GetGravityAcceleration();
      auto fluid_type = environment->GetFluidTypeAtPointInWorld(position, NWU, m_include_waves);
      auto rho_f = environment->GetFluidDensity(fluid_type);

      // From element
      double section_area = el_section->GetArea(); // TODO: voir si on ne prend pas un diameter hydro plutot...
      double diameter = std::sqrt(4. * section_area / MU_PI);


      /*
       * Computing fluid velocities
       */

      chrono::ChVector<double> vf;
      chrono::ChVector<double> vf_dt;

      vf.SetNull();
      vf_dt.SetNull();

      if (fluid_type == WATER) {
        auto ocean = environment->GetOcean();

        // Current
        auto current = ocean->GetCurrent();
        vf += internal::Vector3dToChVector(current->GetFluxVelocityInWorld(position, NWU));

        // Wave orbital velocities
        if (m_include_waves) {
          auto wave_field = ocean->GetFreeSurface()->GetWaveField();
          vf += internal::Vector3dToChVector(wave_field->GetVelocity(position, NWU));
          vf_dt += internal::Vector3dToChVector(wave_field->GetAcceleration(position, NWU));
        }

      } else {  // AIR
        auto wind = environment->GetAtmosphere()->GetWind();
        vf += internal::Vector3dToChVector(wind->GetFluxVelocityInWorld(position, NWU));
      }

      /*
       * Cable velocity
       */
      chrono::ChVector<double> vc;
      element->EvaluateSectionSpeed(U, vc);

      /*
       * Velocities and accelerations decompositions
       */

      // Getting tangential and normal fluid velocities
      chrono::ChVector<double> vf_t = vf.Dot(tangent) * tangent;
      chrono::ChVector<double> vf_n = vf - vf_t;

      // Getting tangential and normal fluid accelerations
      chrono::ChVector<double> vf_t_dt = vf_dt.Dot(tangent) * tangent;
      chrono::ChVector<double> vf_n_dt = vf_dt - vf_t;

      // Getting tangential and normal relative fluid velocities
      chrono::ChVector<double> vr = vf - vc;
      chrono::ChVector<double> vr_t = vr.Dot(tangent) * tangent;
      chrono::ChVector<double> vr_n = vr - vr_t;


      /*
       * Buoyancy
       */

      chrono::ChVector<double> buoyancy_unit_force = {0., 0., rho_f * section_area * gravity};

      /*
       * Morison drag
       */

//      double alpha = std::acos(ut.Dot(v.GetNormalized())); // TODO: verifier
//      assert(0. <= alpha && alpha <= MU_PI_2);
//      double alpha_deg = alpha * MU_180_PI;
//
      double Cd = el_section->GetCd();

      chrono::ChVector<double> morison_drag;
      morison_drag.SetNull();
//
//
//      double v2 = v.Length2(); // TODO: verifier
//
//      double v_n_length = v_n.Length();
//      if (v_n_length != 0.) {
////        morison_drag += 0.5 * rho_f * d * Cd * v2 * v_n / v_n_length * fn(alpha);
//        morison_drag += 0.5 * rho_f * d * Cd * v2 * v_n / v_n_length;
//      }
//
//      double v_t_length = v_t.Length();
//      if (v_t_length != 0.) {
////        morison_drag += 0.5 * rho_f * d * Cd * v2 * v_t / v_t_length * ft(alpha) * mathutils::sgn(v_t.Dot(ut));
//        morison_drag += 0.5 * rho_f * d * Cd * v2 * v_t / v_t_length * mathutils::sgn(v_t.Dot(ut));
//      }


      // New implementation
      morison_drag = 0.5 * rho_f * diameter * Cd * vr_n.Length() * vr_n;
      morison_drag += 0.5 * rho_f * diameter * Cd * vr_t.Length() * vr_t;


//      std::cout << morison_drag.x() << "\t" << morison_drag.z() << std::endl;


      /*
       * Morison added mass effects
       */

      double Cm = el_section->GetCm();

      chrono::ChVector<double> morison_inertia;
      morison_inertia.SetNull();
      morison_inertia = rho_f * section_area * (1. + Cm) * vf_n_dt;

      /*
       * VIV drag amplification
       */

//      alpha = std::acos(ut.Dot(vf.GetNormalized())); // TODO: verifier
//
//      double G = el_section->GetVIVAmpFactor();
//
//      chrono::ChVector<double> drag_viv;
//      drag_viv.SetNull();
//
//      double vf2 = vf.Length2(); // TODO: verifier
//
//      double vf_n_length = vf_n.Length();
//      if (vf_n_length != 0.) {
//        drag_viv += 0.5 * rho_f * d * Cd * vf2 * vf_n / vf_n.Length() * fn(alpha);
//      }
//
//      double vf_t_length = vf_t.Length();
//      if (vf_t_length != 0.) {
//        drag_viv += 0.5 * rho_f * d * Cd * vf2 * vf_t / vf_t.Length() * ft(alpha) * mathutils::sgn(vf_t.Dot(ut));
//      }
//
//      drag_viv *= G;

      /*
       * Summing up contributions
       */

      chrono::ChVector<double> unit_force;
      unit_force.SetNull();
      unit_force += buoyancy_unit_force;
      unit_force += morison_drag;
      unit_force += morison_inertia;
//      unit_force += drag_viv;


      // Pasting the results
      F.PasteVector(unit_force, 0, 0);   // load, force part
      F.PasteVector(chrono::VNULL, 3, 0);  // No torque

    }


    void FrFEACableHydroLoader::ComputeF(const double U,
                                         chrono::ChVectorDynamic<> &F,
                                         chrono::ChVectorDynamic<> *state_x,
                                         chrono::ChVectorDynamic<> *state_w) {

      // This implementation of cable hydrodynamics comes from:
      // Westin C., Modelling and Simulation of Marine Cables with Dynamic Winch and Sheave Contact, 2018,
      // Master thesis, Carleton University, Ottawa, Ontario.

      auto element = dynamic_cast<internal::FrFEACableElementBase *>(loadable.get());
      auto el_section = dynamic_cast<internal::FrFEACableSection *>(element->GetSection().get());

      // Get the point position
      chrono::ChVector<double> position_;
      chrono::ChQuaternion<double> quaternion_;

      element->EvaluateSectionFrame(U, position_, quaternion_);

      auto position = internal::ChVectorToVector3d<Position>(position_);

      // Getting the tangent direction of cable at U
      chrono::ChVector<double> tangent = quaternion_.GetXaxis();

      bool m_include_waves = true; // TODO: faire en sorte que le calcul des orbitales soit fait


      /*
       * Extracting parameters
       */

      // From environment
      auto environment = element->m_environment;

      double gravity = environment->GetGravityAcceleration();
      auto fluid_type = environment->GetFluidTypeAtPointInWorld(position, NWU, m_include_waves);
      auto rho_f = environment->GetFluidDensity(fluid_type);

      // From element
      double section_area = el_section->GetArea(); // TODO: voir si on ne prend pas un diameter hydro plutot...
      double diameter = std::sqrt(4. * section_area / MU_PI);


      /*
       * Computing fluid velocities
       */

      chrono::ChVector<double> vf;
      chrono::ChVector<double> vf_dt;

      vf.SetNull();
      vf_dt.SetNull();

      if (fluid_type == WATER) {
        auto ocean = environment->GetOcean();

        // Current
        auto current = ocean->GetCurrent();
        vf += internal::Vector3dToChVector(current->GetFluxVelocityInWorld(position, NWU));

        // Wave orbital velocities
        if (m_include_waves) {
          auto wave_field = ocean->GetFreeSurface()->GetWaveField();
          vf += internal::Vector3dToChVector(wave_field->GetVelocity(position, NWU));
          vf_dt += internal::Vector3dToChVector(wave_field->GetAcceleration(position, NWU));
        }

      } else {  // AIR
        auto wind = environment->GetAtmosphere()->GetWind();
        vf += internal::Vector3dToChVector(wind->GetFluxVelocityInWorld(position, NWU));
      }

      /*
       * Cable velocity at U
       */
      chrono::ChVector<double> vc;
      element->EvaluateSectionSpeed(U, vc);

      /*
       * Velocities and accelerations decompositions
       */

      // Getting tangential and normal fluid velocities
      chrono::ChVector<double> vf_t = vf.Dot(tangent) * tangent;
      chrono::ChVector<double> vf_n = vf - vf_t;

      // Getting tangential and normal fluid accelerations
      chrono::ChVector<double> vf_dt_t = vf_dt.Dot(tangent) * tangent;
      chrono::ChVector<double> vf_dt_n = vf_dt - vf_dt_t;

      // Getting tangential and normal relative fluid velocities
      chrono::ChVector<double> vr = vf - vc;
      chrono::ChVector<double> vr_t = vr.Dot(tangent) * tangent;
      chrono::ChVector<double> vr_n = vr - vr_t;


      /*
       * Buoyancy effects
       */

      chrono::ChVector<double> buoyancy_unit_force = {0., 0., rho_f * section_area * gravity};


      /*
       * Morison drag effects
       */
      double Cd_n = el_section->GetCd();

      chrono::ChVector<double> morison_drag;
      morison_drag = 0.5 * rho_f * diameter * Cd_n * vr_n.Length() * vr_n;

//      morison_drag = 0.5 * rho_f * diameter * Cd_n * vr_n;
//      morison_drag += 0.5 * rho_f * diameter * Cd * vr_t.Length() * vr_t; // TODO: reactiver avec un coeff tangentiel

      /*
       * Morison added mass effects
       */

      double Ca_n = el_section->GetCm();

      chrono::ChVector<double> morison_inertia;
      morison_inertia.SetNull();
      morison_inertia = rho_f * section_area * ((1. + Ca_n) * vf_dt - Ca_n * vf_dt_t); // TODO: ajouter les effet tangentiels...

      /*
       * Summing up contributions
       */

      chrono::ChVector<double> unit_force;
      unit_force.SetNull();
      unit_force += buoyancy_unit_force;
      unit_force += morison_drag;
      unit_force += morison_inertia;


      // Pasting the results
      F.PasteVector(unit_force, 0, 0);   // load, force part
      F.PasteVector(chrono::VNULL, 3, 0);  // No torque

    }

    void FrFEACableHydroLoader::SetCable(FrFEACable *cable) {
      m_cable = cable;
    }

  }  // end namespace frydom::internal

}  // end namespace frydom
