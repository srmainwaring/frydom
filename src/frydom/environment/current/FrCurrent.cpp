// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// Base for marine current modeling
//
// =============================================================================

//#include <cmath>

//#include <frydom/core/FrConstants.h>

#include "FrCurrent.h"


namespace frydom {


    FrCurrent::FrCurrent(chrono::ChVector<> const velocity_vector,
                         FrFrame frame, FrDirectionConvention convention) {

        auto current_vector = velocity_vector;

        // Place the vector in NWU frame
        if (frame == NED) {
            current_vector = NED2NWU(current_vector);
        }

        if (convention == COMEFROM) {
            current_vector = -current_vector;
        }

        m_currentVector = current_vector;

    }

    FrCurrent::FrCurrent(double const angle,
                         double const velocity,
                         ANGLE_UNIT angleUnit,
                         SPEED_UNIT speedUnit,
                         FrFrame frame,
                         FrDirectionConvention convention) {

        auto alpha = angle;
        if (angleUnit == DEG) {
            alpha = radians(alpha);
        }

        // Ensuring a velocity unit as M/S
        auto magn = convert_velocity_unit(velocity, speedUnit, MS);

        // Building the unit vector from angle
        chrono::ChVector<double> current_vector(
                magn * cos(alpha),
                magn * sin(alpha),
                0.);

        // Managing the convention
        if (convention == COMEFROM) {
            current_vector = -current_vector;
        }

        // Managing conversion into NWU
        if (frame == NED) {
            current_vector = NED2NWU(current_vector);
        }

        m_currentVector = current_vector;

    }

    FrCurrent::FrCurrent(chrono::ChVector<> const udir,
                         double const velocity,
                         SPEED_UNIT speedUnit,
                         FrFrame frame,
                         FrDirectionConvention convention) {

        // TODO: Ensure that the vector is a unit vector

        // Building the current_vector
        auto current_vector = convert_velocity_unit(velocity, speedUnit, MS) * udir;

        // Place the vector in NWU frame
        if (frame == NED) {
            current_vector = NED2NWU(current_vector);
        }

        if (convention == COMEFROM) {
            current_vector = -current_vector;
        }

        m_currentVector = current_vector;

    }

    void FrCurrent::Update(double Time) {
//        std::cout << "Updating current model" << std::endl;
    }

    chrono::ChVector<> FrCurrent::GetFluxVector(FrFrame frame) {
        switch (frame) {
            case NED:
                return NWU2NED(m_currentVector);
            case NWU:
                return m_currentVector;
        }
    }

    chrono::ChVector<> FrCurrent::GetComeFromVector(FrFrame frame) {
        return -FrCurrent::GetFluxVector(frame);
    }

    chrono::ChVector<> FrCurrent::GetGoToVector(FrFrame frame) {
        return FrCurrent::GetFluxVector(frame);
    }

    double FrCurrent::GetAngle(FrDirectionConvention convention, FrFrame frame, ANGLE_UNIT angleUnit) {

        chrono::ChVector<> current_vector;
        switch (convention) {
            case COMEFROM:
                current_vector = GetComeFromVector(frame);
                break;
            case GOTO:
                current_vector = GetGoToVector(frame);
                break;
        }

        double angle = atan2(current_vector.y(), current_vector.x());

        switch (angleUnit) {
            case RAD:
                return angle;
            case DEG:
                return degrees(angle);
        }

    }

    double FrCurrent::GetMagnitude(SPEED_UNIT speedUnit) {
        return convert_velocity_unit(m_currentVector.Length(), MS, speedUnit);
    }

    double FrCurrent::GetMagnitude2() {
        // TODO
    }

    void FrCurrent::Set(const chrono::ChVector<>& unitDirection, double magnitude,
             FrFrame frame, FrDirectionConvention directionConvention, SPEED_UNIT speedUnit) {

        auto uDirection = unitDirection;
        uDirection /= unitDirection.Length();

        if (frame == NED) {
            uDirection = NED2NWU(uDirection);
        }

        if (directionConvention == COMEFROM) {
            uDirection = - uDirection;
        }

        auto vel = magnitude;

        if (speedUnit != MS) {
            vel = convert_velocity_unit(vel, speedUnit, MS);
        }

        // Building the current vector
        m_currentVector = uDirection * vel;

    }


    void SetCurrentVector(chrono::ChVector<double>& currentVector, FrFrame frame=NED, SPEED_UNIT speedUnit=KNOT) {
        // TODO
    }


}  // end namespace frydom
