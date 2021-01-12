//
// Created by camille on 27/03/2020.
//

#ifndef FRYDOM_BARELEMENT_H
#define FRYDOM_BARELEMENT_H

#include "frydom/frydom.h"

namespace frydom {

  // ----------------------------------------------------------
  // Bar element
  // ----------------------------------------------------------

  class FrBarElementBase {

   public:

    FrBarElementBase();

    virtual void Update(double time) = 0;

    virtual void Initialize() = 0;

    Force GetForce() const;

    Torque GetTorque() const;

    bool IsImmerged() const;

    std::shared_ptr<FrNode>& GetNode();

   protected:

    Force m_force;                      //< Force of elements in world reference frame
    Torque m_torque;                    //< Torque of elements in body reference frame at COG
    bool m_is_immerged;

    std::shared_ptr<FrNode> m_node;

  };

  // -------------------------------------------------------
  // Single Bar element
  // -------------------------------------------------------

  class FrSingleBarElement : public FrBarElementBase {

   public:

    FrSingleBarElement(FrBody* body, Position posA, Position posB, double radius);

    void Update(double time) override;

    void Initialize() override;

   protected:

    void SetFrame(FrBody* body, Position& posA, Position& posB);

    void NullifyForceTorque();

    void SetForceTorque(const Position& posA, const double wA, const Position& posB, const double wB);

    void SetForceTorque();

   protected:

    std::shared_ptr<FrNode> m_node_start;
    std::shared_ptr<FrNode> m_node_end;

    double m_length;
    double m_radius;
    double m_volume;
    double m_buoyancy;

  };

  // -----------------------------------------------------
  // Composite bar element
  // -----------------------------------------------------

  class FrCompositeBarElement : public FrBarElementBase {

   public:

    FrCompositeBarElement(FrBody* body);

    void AddElement(FrBarElementBase* element);

    void AddElement(Position posA, Position posB, double radius, unsigned int n=1);

    void Update(double time) override;

    void Initialize() override;

    void SetCorrectionFactor(double cfactor);

   protected:

    std::vector<std::unique_ptr<FrBarElementBase>> m_elements;

    double m_cfactor;

  };

  // ----------------------------------------------------
  // Makers
  // ----------------------------------------------------

  std::shared_ptr<FrCompositeBarElement> make_bar_element(const std::shared_ptr<FrBody> &body);

  std::shared_ptr<FrCompositeBarElement> make_bar_element(const std::shared_ptr<FrBody> &body,
                                                          const std::string& filename);


} // end namespace frydom

#endif //FRYDOM_BARELEMENT_H
