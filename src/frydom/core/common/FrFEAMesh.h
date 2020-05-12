//
// Created by lletourn on 06/03/19.
//

#ifndef FRYDOM_FRFEAMESH_H
#define FRYDOM_FRFEAMESH_H


//#include "frydom/core/common/FrPhysicsItem.h"

#include "frydom/asset/FrAssetOwner.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrTreeNode.h"


namespace frydom {

  class FrFEAMesh;

  namespace internal {

    class FrFEAMeshBase : public chrono::fea::ChMesh { // This is a chrono::ChPhysicsItem from ChMesh !

     public:
      explicit FrFEAMeshBase(FrFEAMesh *frydom_mesh);

      virtual void Initialize() = 0;

     protected:
      FrFEAMesh *m_frydom_mesh;

    };

    std::shared_ptr<FrFEAMeshBase> GetChronoFEAMesh(std::shared_ptr<FrFEAMesh> mesh);

  }  // end namespace frydom::internal


  class FrFEAMesh : public FrObject, public FrLoggable<FrOffshoreSystem> {

   public:

    FrFEAMesh(const std::string &name,
              const std::string &type_name,
              FrOffshoreSystem *system,
              std::shared_ptr<internal::FrFEAMeshBase> chrono_mesh);


    virtual double GetStaticResidual() = 0;

    virtual void Relax() = 0;


   protected:
    std::shared_ptr<internal::FrFEAMeshBase> GetFEAMeshBase();

    std::shared_ptr<internal::FrFEAMeshBase> m_chrono_mesh;

    friend std::shared_ptr<internal::FrFEAMeshBase> internal::GetChronoFEAMesh(std::shared_ptr<FrFEAMesh> mesh);

  };

} //end namespace frydom


#endif //FRYDOM_FRFEAMESH_H
