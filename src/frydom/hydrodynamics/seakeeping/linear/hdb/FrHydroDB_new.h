//
// Created by lletourn on 27/05/20.
//

#ifndef FRYDOM_FRHYDRODB_NEW_H
#define FRYDOM_FRHYDRODB_NEW_H

#include "hdb5_io/HDB5_io.h"
#include "FrHydroMapper_new.h"

namespace frydom {

  class FrHydroDB_new {

   public:

    /// Constructor of the hydrodynamic database with specified HDF5 filename
    explicit FrHydroDB_new(std::string h5file);

    /// Return the mapper between frydom body and BEM body database
    /// \return Mapper
    FrHydroMapper_new *GetMapper();

    /// Define a map between a BEM body database and a body
    /// \param BEMBody BEM body database
    /// \param body body (frydom object)
    /// \param eqFrame Equilibrium frame of the corresponding body
    void Map(FrBEMBody_new *BEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

    /// Define a map between a BEM body database and a body
    /// \param iBEMBody Index of the BEM body database in the HDB
    /// \param body body (frydom object)
    /// \param eqFrame Equilibrium frame of the corresponding body
    void Map(int iBEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

    std::unordered_map<FrBEMBody_new *, FrBody *>::iterator begin();

    std::unordered_map<FrBEMBody_new *, FrBody *>::iterator end();


   private:

    std::shared_ptr<HDB5_io::HydrodynamicDataBase> m_HDB;
    std::unique_ptr<FrHydroMapper_new> m_mapper;            ///< Mapper between bodies and hdb body database

  };

  std::shared_ptr<FrHydroDB_new> make_new_hydrodynamic_database(std::string h5file);

} //end namespace frydom

#endif //FRYDOM_FRHYDRODB_NEW_H
