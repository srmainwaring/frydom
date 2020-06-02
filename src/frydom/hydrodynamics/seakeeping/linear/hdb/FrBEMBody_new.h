//
// Created by lletourn on 28/05/20.
//

#ifndef FRYDOM_FRBEMBODY_NEW_H
#define FRYDOM_FRBEMBODY_NEW_H

#include "hdb5_io/HDB5_io.h"

namespace frydom {

//  using FrBEMBody = HDB5_io::Body;

  class FrBEMBody : public HDB5_io::Body {

  };

  class FrWaveDriftPolarData : public HDB5_io::WaveDrift {

    bool HasSurge() const { return m_data->HasSerie("surge"); }

    bool HasSway() const { return m_data->HasSerie("sway"); }

    bool HasHeave() const { return m_data->HasSerie("heave"); }

    bool HasPitch() const { return m_data->HasSerie("pitch"); }

    bool HasRoll() const { return m_data->HasSerie("roll"); }

    bool HasYaw() const { return m_data->HasSerie("yaw"); }

  };

}

#endif //FRYDOM_FRBEMBODY_NEW_H
