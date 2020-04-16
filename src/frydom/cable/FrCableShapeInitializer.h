//
// Created by frongere on 15/02/2020.
//

#ifndef FRYDOM_FRCABLESHAPEINITIALIZER_H
#define FRYDOM_FRCABLESHAPEINITIALIZER_H

#include "frydom/core/math/FrVector.h"


namespace frydom {

  // Forward declarations
  class FrEnvironment;

  class FrCableBase;

  class FrCableShapeInitializer {

   public:
    // Factory static method to get the correct Shape initializer for the given cable
    static std::unique_ptr<FrCableShapeInitializer> Create(FrCableBase *cable, FrEnvironment *environment);

    virtual Position GetPosition(const double &s, FRAME_CONVENTION fc) const = 0;

    virtual Direction GetTangent(const double &s, FRAME_CONVENTION fc) const = 0;

   protected:
    // info: constructor is protected as we must use the static Create method which is a factory method
    // and choose the rigth concrete class to implement
    explicit FrCableShapeInitializer(FrCableBase *cable);

    FrCableBase *m_cable;
  };

  class FrCatenaryLine;

  class FrCatenaryLineSeabed;

  namespace internal {

    class FrCableShapeInitializerTaut : public FrCableShapeInitializer {
     public:
      explicit FrCableShapeInitializerTaut(FrCableBase *cable);

      Position GetPosition(const double &s, FRAME_CONVENTION fc) const override;

      Direction GetTangent(const double &s, FRAME_CONVENTION fc) const override;

     private:
      Direction m_unit_vector;

    };

    class FrCableShapeInitializerSlack : public FrCableShapeInitializer {
     public:
      explicit FrCableShapeInitializerSlack(FrCableBase *cable, std::unique_ptr<FrCatenaryLine> catenary_cable);

      Position GetPosition(const double &s, FRAME_CONVENTION fc) const override;

      Direction GetTangent(const double &s, FRAME_CONVENTION fc) const override;

     private:
      std::unique_ptr<FrCatenaryLine> m_catenary_line;

    };

    class FrCableShapeInitializerSlackSeabed : public FrCableShapeInitializer {
     public:
      FrCableShapeInitializerSlackSeabed(FrCableBase *cable,
                                         FrEnvironment *environment);

      Position GetPosition(const double &s, FRAME_CONVENTION fc) const override;

      Direction GetTangent(const double &s, FRAME_CONVENTION fc) const override;

     private:
      FrEnvironment *m_environment;
      std::unique_ptr<FrCatenaryLineSeabed> m_catenary_line_seabed;

      bool m_reversed;

    };

  }  // end namespace frydom::internal
} // end namespace frydom

#endif //FRYDOM_FRCABLESHAPEINITIALIZER_H
