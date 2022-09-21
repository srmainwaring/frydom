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

#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include "hdb5_io/hdb5_io.h"
#include "FrHydroMapper.h"

namespace frydom {

  using FrWaveDriftPolarData = hdb5_io::WaveDrift;

  class FrHydroDB {

   public:

    /// Constructor of the hydrodynamic database with specified HDF5 filename
    explicit FrHydroDB(std::string h5file);

    explicit FrHydroDB(const std::shared_ptr<hdb5_io::HydrodynamicDataBase>& hdb);

    /// Return the BEM body database
    /// \param ibody Index of the BEM body
    /// \return BEM Body database
    FrBEMBody *GetBody(int ibody);

//    /// Return the BEM body database
//    /// \param body frydom body object
//    /// \return BEM body database
//    FrBEMBody *GetBody(std::shared_ptr<FrBody> body);

    /// Return the BEM body database
    /// \param body frydom body object
    /// \return BEM body database
    FrBEMBody *GetBody(FrBody *body);

    /// Return the frydom body object
    /// \param body BEM body database
    /// \return frydom body
    FrBody *GetBody(FrBEMBody *body);

    /// This method returns the number of BEM bodies.
    int GetBEMBodyNumber();

    /// Return the mapper between frydom body and BEM body database
    /// \return Mapper
    FrHydroMapper *GetMapper();

    /// Define a map between a BEM body database and a body
    /// \param BEMBody BEM body database
    /// \param body body (frydom object)
    /// \param eqFrame Equilibrium frame of the corresponding body
    void Map(FrBEMBody *BEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

    /// Define a map between a BEM body database and a body
    /// \param iBEMBody Index of the BEM body database in the HDB
    /// \param body body (frydom object)
    /// \param eqFrame Equilibrium frame of the corresponding body
    void Map(int iBEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

    std::unordered_map<FrBEMBody *, FrBody *>::iterator begin();

    std::unordered_map<FrBEMBody *, FrBody *>::iterator end();


    mathutils::VectorN<double> GetFrequencyDiscretization() const {return m_HDB->GetFrequencyDiscretization();};

    //TODO : add frame and direction conventions + angle unit
    mathutils::VectorN<double> GetWaveDirectionDiscretization() const {return m_HDB->GetWaveDirectionDiscretization();};

    mathutils::VectorN<double> GetTimeDiscretization() const {return m_HDB->GetTimeDiscretization();};

    FrWaveDriftPolarData * GetWaveDrift() const;

    double GetMinFrequency() const;

    double GetMaxFrequency() const;

    FrMask GetBodyDOFMask(FrBEMBody* BEMBody) const;

    mathutils::Matrix66<bool> GetBodyRadiationMask(FrBEMBody* BEMBody, FrBEMBody* BEMBodyMotion);

    /// This method gives the boolean to known if x-derivatives of the added mass and damping coefficients are present.
    bool GetIsXDerivative() const;

    void ActivateDOFMask(bool dofMaskApplied) {
      m_dofMaskApplied = dofMaskApplied;
    }

   private:

    std::shared_ptr<hdb5_io::HydrodynamicDataBase> m_HDB;
    std::unique_ptr<FrHydroMapper> m_mapper;            ///< Mapper between bodies and hdb body database
    bool m_dofMaskApplied = true;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrHydroDB> make_hydrodynamic_database(std::string h5file);

  std::shared_ptr<FrHydroDB> make_hydrodynamic_database(const std::shared_ptr<hdb5_io::HydrodynamicDataBase>& hdb);

} //end namespace frydom

#endif //FRYDOM_FRHYDRODB_H
