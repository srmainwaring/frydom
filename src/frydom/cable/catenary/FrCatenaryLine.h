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


#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include "FrCatenaryLineBase.h"
#include "FrCatenaryLineSeabed.h"

#include "frydom/environment/FrFluidType.h"
#include "frydom/cable/mooring_components/FrClumpWeight.h"


namespace frydom {

  class FrCatenaryLine;

  namespace internal {

    class PointForce {
     public:
      PointForce(FrCatenaryLine *line, const double &s, const Force &force);

      const double &s() const;

      Force force() const;

     private:
      FrCatenaryLine *m_line;
      double m_s;
      Force m_force;
    };

  }  // end namespace frydom::internal


  class FrCatenaryLine : public FrCatenaryLineBase { // TODO: retirer l'option elastic qui sera toujours a true !!!

   public:

    FrCatenaryLine(const std::string &name,
                   const std::shared_ptr<FrNode> &startingNode,
                   const std::shared_ptr<FrNode> &endingNode,
                   const std::shared_ptr<FrCableProperties> &properties,
                   bool elastic,
                   double unstretchedLength,
                   FLUID_TYPE fluid_type);

    FrCatenaryLine(const std::string &name,
                   FrCableBase *cable,
                   bool elastic,
                   FLUID_TYPE fluid_type);

    void AddClumpWeight(double s, const double &mass, bool reversed = false);

    void AddBuoy(double s, const double &mass, bool reversed = false);

    bool IsSingular() const;

    void Initialize() override;

    void StepFinalize() override;

    Force GetTension(const double &s, FRAME_CONVENTION fc) const override;

    Direction GetTangent(const double s, FRAME_CONVENTION fc) const override;

    Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const override;

    double GetUnstretchedLength() const override;

    void solve() override;

    bool HasSeabedInteraction() const override;

    void GetLowestPoint(Position &position,
                        double &s,
                        FRAME_CONVENTION fc,
                        const double tol,
                        const unsigned int maxIter) const override;

    using Residue3 = mathutils::Vector3d<double>;
    using Jacobian33 = mathutils::Matrix33<double>;

   private:
    // WARNING: every method into the private scope are working with dimensionless variables and in NWU
    // The public part of the catenary line API works with dimensional variables.
    // Forces are divided by Lq and lengths by L

    Jacobian33 GetJacobian() const;

    Residue3 GetResidue() const;

    void AddPointMass(const double &s, const Force &force);

    double rho(const unsigned int &i, const double &s) const; // inline

    Force Fi(const unsigned int &i) const; // inline

    Force fi(const unsigned int &i) const; // inline

    double si(const unsigned int &i) const; // inline

    Direction Lambda_tau(const unsigned int &i, const double &s) const; // inline

    unsigned int N() const; // inline en priorite !!

    Force ti(const unsigned int &i, const double &s) const; // inline

    Tension t(const double &s) const; // inline

    Tension tL() const; // inline

    unsigned int SToI(const double &s) const;

    void p_pi(Position &position, const unsigned int &i, const double &s) const;

    void p_perp(Position &position, const unsigned int &i, const double &s) const;

    void pc(Position &position, const unsigned int &i, const double &s) const;

    Force sum_fs(const unsigned int &i) const; // inline

    void pe(Position &position, const unsigned int &i, const double &s) const;

    void pi(Position &position, const unsigned int &i, const double &s) const; // inline

    void p(const double &s, Position &position) const; // inline

    Position p(const double &s) const;

    Position pL() const; // inline

    void FirstGuess() override;

    void dpc_dt(Jacobian33 &jacobian) const; // inline

    void dp_pi_dt(Jacobian33 &jacobian) const; // inline

    void dp_perp_dt(Jacobian33 &jacobian) const; // inline

    void dpe_dt(Jacobian33 &jacobian) const; // inline

    void Compute(double time) override;

    void DefineLogMessages() override;

    unsigned int GetIter() const { return c_iter;}
    double GetCriterion() const { return c_criterion;}
    double GetErr() const { return c_pos_error;}

    void BuildCache() override;


    // Friends declarations

    friend Force internal::PointForce::force() const;

    friend class internal::JacobianBuilder;

    friend class FrCatenaryLineSeabed;

   private:
    Tension m_t0;
    std::vector<internal::PointForce> m_point_forces;

    // Cache
    FLUID_TYPE c_fluid;                 ///< cached value of the fluid type in which the catenary line is mostly in.
    std::vector<Force> c_Fi;
    std::vector<Force> c_sum_fs;
    mathutils::Matrix33<double> c_U; // This is (I - pi.pi^t)
    double c_qL;  // Used for adimensionalization of forces and tensions

    unsigned int c_iter;
    double c_pos_error;
    double c_criterion;

  };


  std::shared_ptr<FrCatenaryLine>
  make_catenary_line(const std::string &name,
                     const std::shared_ptr<FrNode> &startingNode,
                     const std::shared_ptr<FrNode> &endingNode,
                     const std::shared_ptr<FrCableProperties> &properties,
                     bool elastic,
                     double unstretchedLength,
                     FLUID_TYPE fluid_type);

}  // end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
