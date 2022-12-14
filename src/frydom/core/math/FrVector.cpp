// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrVector.h"
#include "MathUtils/Constants.h"


namespace frydom {

  const Direction NORTH(FRAME_CONVENTION fc) {
    auto vect = Direction(1, 0, 0);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction NORTH_EAST(FRAME_CONVENTION fc) {
    auto vect = Direction(MU_SQRT2_2, -MU_SQRT2_2, 0);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction EAST(FRAME_CONVENTION fc) {
    auto vect =Direction(0, -1, 0);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction SOUTH_EAST(FRAME_CONVENTION fc) {
    auto vect = Direction(-MU_SQRT2_2, -MU_SQRT2_2, 0);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction SOUTH(FRAME_CONVENTION fc) {
    auto vect = Direction(-1, 0, 0);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction SOUTH_WEST(FRAME_CONVENTION fc) {
    auto vect = Direction(-MU_SQRT2_2, MU_SQRT2_2, 0);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction WEST(FRAME_CONVENTION fc) {
    auto vect = Direction(0, 1, 0);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction NORTH_WEST(FRAME_CONVENTION fc) {
    auto vect = Direction(MU_SQRT2_2, MU_SQRT2_2, 0);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction UP(FRAME_CONVENTION fc) {
    auto vect = Direction(0, 0, 1);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

  const Direction DOWN(FRAME_CONVENTION fc) {
    auto vect = Direction(0, 0, -1);
    if (IsNED(fc)) { vect = internal::SwapFrameConvention(vect); }
    return vect;
  }

} // end namespace frydom
