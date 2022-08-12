//
// Created by frongere on 07/05/2020.
//

#ifndef FRYDOM_FRCONSTANTFORCE_H
#define FRYDOM_FRCONSTANTFORCE_H

#include "FrForce.h"


namespace frydom {

  // Forward declaration
  class FrNode;

  class FrConstantForce : public FrForce {

   public:
    enum MODE {
      ABSOLUTE,
      FOLLOWING
    };

   public:
    FrConstantForce(const std::string &name,
                    std::shared_ptr<FrNode> node,
                    MODE mode,
                    const Force &force,
                    FRAME_CONVENTION fc);

    void SetForce(const Force &force, FRAME_CONVENTION fc);

    Force GetForce() const;

    MODE GetMode() const;

    void SetMode(MODE mode);

    void SwitchMode();

    void Compute(double time) override;

   private:
    std::shared_ptr<FrNode> m_node;
    MODE m_mode;
    Force m_force;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrConstantForce> make_constant_force(const std::string &name,
                                                       std::shared_ptr<FrNode> node,
                                                       FrConstantForce::MODE mode,
                                                       const Force &force,
                                                       FRAME_CONVENTION fc);

}  // end namespace frydom



#endif //FRYDOM_FRCONSTANTFORCE_H
