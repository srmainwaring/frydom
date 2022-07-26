
#include <chrono/assets/ChBoxShape.h>

#include "frydom/core/math/FrVector.h"
#include "FrBoxShape.h"


namespace frydom {

  FrBoxShape::FrBoxShape(double xSize, double ySize, double zSize,
                         const Position &relative_position, FRAME_CONVENTION fc) :
      m_box(std::make_shared<chrono::ChBoxShape>()) {
    m_box->GetBoxGeometry().SetLengths(chrono::ChVector<double>(xSize, ySize, zSize));

    Position pos = relative_position;
    if (IsNED(fc)) {
      pos = internal::SwapFrameConvention<Position>(pos);
    }
    m_box->GetBoxGeometry().Pos = internal::Vector3dToChVector(pos);

  }

  double FrBoxShape::xSize() const {
    return m_box->GetBoxGeometry().GetLengths().x();
  }

  double FrBoxShape::ySize() const {
    return m_box->GetBoxGeometry().GetLengths().y();
  }

  double FrBoxShape::zSize() const {
    return m_box->GetBoxGeometry().GetLengths().z();
  }

  void FrBoxShape::Translate(const Direction &direction) const {
    m_box->Pos = internal::Vector3dToChVector(direction);
  }

  namespace internal {

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(std::shared_ptr<FrBoxShape> box) {
      return box->m_box;
    }

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(FrBoxShape *box) {
      return box->m_box;
    }

  }  // end namespace frydom::internal

}  // end namespace frydom
