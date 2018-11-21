//
// Created by camille on 15/11/18.
//

#include "FrFlowForce.h"

#include "frydom/core/FrBody.h"
#include "FrFlowBase.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/IO/FrLoader.h"
#include "frydom/core/FrFrame.h"
#include "MathUtils/Vector3d.h"


namespace frydom {

    FrFlowForce::FrFlowForce(const std::string& yamlFile) {
        this->ReadTable(yamlFile);
    }

    void FrFlowForce::ReadTable(const std::string& yamlFile) {

        std::vector<std::pair<double, Vector3d<double>>> polar;
        std::pair<double, Vector3d<double>> new_element;
        ANGLE_UNIT angle_unit;
        FRAME_CONVENTION fc;
        DIRECTION_CONVENTION dc;

        LoadFlowPolarCoeffFromYaml(yamlFile, polar, angle_unit, fc, dc);

        if (angle_unit == DEG) {
            for (auto it=polar.begin(); it != polar.end(); ++it) { it->first *= DEG2RAD; }
        }

        // Complete if symmetry
        auto max_angle = polar.back().first;
        auto min_angle = polar[0].first;

        if (std::abs(min_angle) < 10e-2 and std::abs(max_angle - M_PI) < 10e-2) {
            for (unsigned int i=polar.size()-2; i>=1; i--) {
                new_element.first = 2.* M_PI - polar[i].first;
                new_element.second = { polar[i].second[0], -polar[i].second[1], -polar[i].second[2] };
                polar.push_back(new_element);
            }
        } else if (std::abs(min_angle + M_PI) < 10e-2 and std::abs(max_angle) < 10e-2) {
            for (unsigned int i=polar.size()-2; i>=1; i--) {
                new_element.first = -polar[i].first;
                new_element.second = { polar[i].second[0], -polar[i].second[1], -polar[i].second[2] };
                polar.push_back(new_element);
            }
        }

        // Delete double term
        if ( std::abs(polar[0].first) < 10e-2 and std::abs(polar.back().first - 2.* M_PI) < 10e-2
                or std::abs(polar[0].first + M_PI) < 10e-2 and std::abs(polar.back().first - M_PI) < 10e-2)  {
            polar.pop_back();
        }

        // Conversion to NWU if NED convention is used
        if (fc == NED) {
            for (auto it=polar.begin(); it != polar.end(); ++it) {
                it->first = -it->first;
                it->second = { it->second[0], -it->second[1], -it->second[2] };
            }
        }

        // Conversion to GOTO if COMEFROM convention is used
        if (dc == COMEFROM) {
            for (auto it=polar.begin(); it!= polar.end(); ++it) { it->first += M_PI; }
        }

        // Normalized angle in [0, 2pi]
        for (auto it=polar.begin(); it != polar.end(); ++it) { it->first = Normalize_0_2PI(it->first); }

        // Sort element according to increasing angles
        std::sort(polar.begin(), polar.end(), [](auto const &a, auto const &b) {
            return a.first < b.first;
        });

        // Adding last term for angle equal to 2pi
        new_element.first = 2* M_PI;
        new_element.second =  polar.begin()->second;
        polar.push_back( new_element );


        // Complete lookup table
        std::vector<double> anglesL;
        std::vector<Vector3d<double>> vectL;

        for (auto it=polar.begin(); it != polar.end(); ++it) {
            anglesL.push_back(it->first);
            vectL.push_back(it->second);
        }

        m_table.SetX(anglesL);
        m_table.AddY("coeff", vectL);

        // ##CC Debug
        std::ofstream file;
        file.open("coeff_table.csv",  std::ios::out);
        file << "angle;cx;cy;cn" << std::endl;
        for (auto it=polar.begin(); it != polar.end(); ++it) {
            file << it->first << ";" << it->second[0] << ";" << it->second[1] << ";" << it->second[2] << std::endl;
        }
        file.close();
        // ##CC
    }

    void FrFlowForce::Update(double time) {

        double alpha = m_fluxVelocityInBody.GetProjectedAngleAroundZ(RAD);
        alpha = Normalize_0_2PI(alpha);

        auto coeff = m_table.Eval("coeff", alpha);
        double SquaredVelocity = m_fluxVelocityInBody.squaredNorm();
        auto force_temp = coeff * SquaredVelocity;

        SetForceTorqueInBodyAtCOG(Force(force_temp[0], force_temp[1], 0.), Torque(0., 0., force_temp[2]), NWU);

    }


    void FrCurrentForce2_::Update(double time) {

        FrFrame_ FrameAtCOG = m_body->GetFrameAtCOG(NWU);
        Velocity VelocityInWorldAtCOG =  m_body->GetCOGVelocityInWorld(NWU);

        m_fluxVelocityInBody = m_body->GetSystem()->GetEnvironment()->GetCurrent()
                ->GetRelativeVelocityInFrame(FrameAtCOG, VelocityInWorldAtCOG, NWU);

        FrFlowForce::Update(time);
    }



    void FrWindForce2_::Update(double time) {

        FrFrame_ FrameAtCOG = m_body->GetFrameAtCOG(NWU);
        Velocity VelocityInWorldAtCOG =  m_body->GetCOGVelocityInWorld(NWU);

        m_fluxVelocityInBody = m_body->GetSystem()->GetEnvironment()->GetWind()
                ->GetRelativeVelocityInFrame(FrameAtCOG, VelocityInWorldAtCOG, NWU);

        FrFlowForce::Update(time);

    }

} // end of namespace frydom