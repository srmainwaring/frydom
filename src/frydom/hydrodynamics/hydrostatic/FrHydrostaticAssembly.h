//
// Created by lletourn on 05/02/20.
//

#ifndef FRYDOM_FRHYDROSTATICASSEMBLY_H
#define FRYDOM_FRHYDROSTATICASSEMBLY_H

#include "frydom/core/body/FrBody.h"
#include "frydom/core/body/FrInertiaTensor.h"
#include "frydom/core/body/FrAssembly.h"

namespace frydom {

  class FrHydrostaticBodyBase {

   public:
    virtual ~FrHydrostaticBodyBase() {}

    virtual FrInertiaTensor GetInertiaTensor() const = 0;

    virtual std::shared_ptr<FrBody> GetBody() const = 0;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  class FrHydrostaticBody : public FrHydrostaticBodyBase {

   public:
    virtual ~FrHydrostaticBody() {}

    explicit FrHydrostaticBody(const std::shared_ptr<FrBody> &body) : m_body(body) {}

    FrInertiaTensor GetInertiaTensor() const override {return m_body->GetInertiaTensor();};

    std::shared_ptr<FrBody> GetBody() const override { return m_body;}

   private:
    std::shared_ptr<FrBody> m_body;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  class FrHydrostaticAssembly : public FrHydrostaticBodyBase {

   public:
    virtual ~FrHydrostaticAssembly() {}

    explicit FrHydrostaticAssembly(const std::shared_ptr<FrAssembly> &assembly) : m_assembly(assembly) {}

    FrInertiaTensor GetInertiaTensor() const override {return m_assembly->GetInertiaTensor();};

    std::shared_ptr<FrBody> GetBody() const override { return m_assembly->GetMasterBody();}

   private:
    std::shared_ptr<FrAssembly> m_assembly;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };


} // end namespace frydom

#endif //FRYDOM_FRHYDROSTATICASSEMBLY_H
