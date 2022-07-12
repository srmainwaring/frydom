

#include <chrono/assets/ChCylinderShape.h>

#include "FrCylinderShape.h"

namespace frydom {
  FrCylinderShape::FrCylinderShape(double radius, double height,
      const Position& relative_position, FRAME_CONVENTION fc) :
      m_cylinder(std::make_shared<chrono::ChCylinderShape>()) {
    m_cylinder->GetCylinderGeometry().rad = radius;

    Position pos = relative_position;
    if (IsNED(fc)) {
      internal::SwapFrameConvention<Position>(pos);
    }
    m_cylinder->GetCylinderGeometry().p1 = pos + Position(0., -height*0.5, 0.);
    m_cylinder->GetCylinderGeometry().p2 = pos + Position(0., height*0.5, 0.);
  }

  double FrCylinderShape::radius() const {
    return m_cylinder->GetCylinderGeometry().rad;
  }

  double FrCylinderShape::height() const {
    return fabs(m_cylinder->GetCylinderGeometry().p2.y() - m_cylinder->GetCylinderGeometry().p1.y());
  }


  namespace internal {
    std::shared_ptr<chrono::ChAsset> GetChronoAsset(std::shared_ptr<FrCylinderShape> cylinder) {
      return cylinder->m_cylinder;
    }

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(FrCylinderShape *cylinder) {
      return cylinder->m_cylinder;
    }

  }  // end namespace frydom::internal

}  // end namespace frydom
