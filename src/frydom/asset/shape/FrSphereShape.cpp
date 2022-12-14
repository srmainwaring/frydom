
#include <chrono/assets/ChSphereShape.h>

#include "FrSphereShape.h"


namespace frydom {

  FrSphereShape::FrSphereShape(double radius, const Position& relative_position, FRAME_CONVENTION fc) :
  m_sphere(std::make_shared<chrono::ChSphereShape>()) {
    m_sphere->GetSphereGeometry().rad = radius;

    Position pos = relative_position;
    if (IsNED(fc)) {
      pos = internal::SwapFrameConvention<Position>(pos);
    }
    m_sphere->GetSphereGeometry().center = internal::Vector3dToChVector(pos);
  }

  double FrSphereShape::radius() const {
    return m_sphere->GetSphereGeometry().rad;
  }


  namespace internal {

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(std::shared_ptr<FrSphereShape> sphere) {
      return sphere->m_sphere;
    }

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(FrSphereShape *sphere) {
      return sphere->m_sphere;
    }

  } // end namespace frydom::internal


}  // end namespace frydom
