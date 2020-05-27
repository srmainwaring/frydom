//
// Created by lletourn on 27/05/20.
//

#ifndef FRYDOM_FRHYDROMAPPER_NEW_H
#define FRYDOM_FRHYDROMAPPER_NEW_H

#include "hdb5_io/HDB5_io.h"

namespace frydom {

  using FrBEMBody_new = HDB5_io::Body;

  // Forward declaration
  class FrEquilibriumFrame;

  class FrBody;

  class FrEquilibriumFrame;

  class FrHydroMapper_new {

   private:
    std::unordered_map<FrBEMBody_new *, FrBody *> m_mapBEMToBody; // Mapping of FrBEMBodies with FrBodies.
    std::unordered_map<FrBody *, FrBEMBody_new *> m_mapBodyToBEM; // Mapping of FrBodies with FrBEMBodies.
    std::unordered_map<FrBEMBody_new *, std::shared_ptr<FrEquilibriumFrame>> m_mapEqFrame; // Mapping of BEMBodies with FrEquilibriumFrame.

   public:

    /// Mapping a BEM body database with a body
    /// \param BEMBody BEM Body database
    /// \param body body (frydom object)
    /// \param eqFrame Equilibrium frame corresponding to the body
    void Map(FrBEMBody_new *BEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

    /// Get the number of bodies map
    /// \return Number of bodies map
    unsigned long GetNbMappings() const;

    /// Return the body corresponding to a given BEM body database
    /// \param BEMBody BEM body database
    /// \return body(frydom object)
    FrBody *GetBody(FrBEMBody_new *BEMBody) const;

    /// Return the BEM body database corresponding to the given body
    /// \param body body (frydom object)
    /// \return BEM body database
    FrBEMBody_new *GetBEMBody(FrBody *body) const;

    /// Return the corresponding index in the map of a given body
    /// \param body body (frydom object)
    /// \return Index of the body in the map
    unsigned int GetBEMBodyIndex(FrBody *body) const;

    /// Return the equilibrium frame
    /// \param BEMBody BEM body database
    /// \return Equilibrium frame
    FrEquilibriumFrame *GetEquilibriumFrame(FrBEMBody_new *BEMBody) const;

    /// Return the equilibrium frame
    /// \param body body (frydom object)
    /// \return Equilibrium frame
    FrEquilibriumFrame *GetEquilibriumFrame(FrBody *body) const;

    /// Return the equilibrium frame
    /// \param body body (frydom object)
    /// \return Equilibrium frame
    std::shared_ptr<FrEquilibriumFrame> GetSharedEquilibriumFrame(FrBody *body) const;

    std::unordered_map<FrBEMBody_new *, FrBody *>::iterator begin();

    std::unordered_map<FrBEMBody_new *, FrBody *>::iterator end();;

  };

} //end namespace frydom

#endif //FRYDOM_FRHYDROMAPPER_NEW_H
