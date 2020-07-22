//
// Created by lletourn on 27/05/20.
//

#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include "hdb5_io/HDB5_io.h"
#include "FrHydroMapper.h"

namespace frydom {

  using FrWaveDriftPolarData = HDB5_io::WaveDrift;

  class FrHydroDB {

   public:

    /// Constructor of the hydrodynamic database with specified HDF5 filename
    explicit FrHydroDB(std::string h5file);

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


    FrMask GetBodyDOFMask(FrBEMBody* BEMBody) const;

    void GetImpulseResponseSize(double timeStep, double &Te, double &dt) const;

   private:

    std::shared_ptr<HDB5_io::HydrodynamicDataBase> m_HDB;
    std::unique_ptr<FrHydroMapper> m_mapper;            ///< Mapper between bodies and hdb body database

  };

  std::shared_ptr<FrHydroDB> make_hydrodynamic_database(std::string h5file);

} //end namespace frydom

#endif //FRYDOM_FRHYDRODB_H
