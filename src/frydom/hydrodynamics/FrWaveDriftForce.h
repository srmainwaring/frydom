//
// Created by camille on 14/12/17.
//

#ifndef FRYDOM_FRWAVEDRIFTFORCE_H
#define FRYDOM_FRWAVEDRIFTFORCE_H

#include <frydom/core/FrForce.h>
#include <frydom/environment/ocean/freeSurface/waves/FrWaveProbe.h>
#include <frydom/core/FrHydroBody.h>
#include <MathUtils/MathUtils.h>

namespace frydom {

    class FrWaveDriftForce : public FrForce {

    private:

        int m_NbModes;                                     ///< Number of modes representd in the database
        std::shared_ptr<FrLinearWaveProbe> m_waveProbe;    ///< Wave probe for local wave field characteristics
        std::vector<std::vector<std::complex<double>>> m_CmplxElevation;  ///< Wave complex elevation
        std::vector<std::vector<double>> m_waveAmplitude;
        std::vector<std::unique_ptr<mathutils::LookupTable2d<>>> m_table;                 ///< Lookup table 2D depending on freq and heading (for each mode)
        std::shared_ptr<FrHydroBody> m_body;                            ///< Hydro body to which the force is applied
        bool m_sym_x;
        bool m_sym_y;

    public:

        /// Construct a new force model from drift table coefficients
        FrWaveDriftForce(const std::string hdf5_file);

        /// Wave probe attached to the force at the application point location
        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbe>& waveProbe) { m_waveProbe = waveProbe; }

        /// Definition of the body where the force is applied
        void SetBody(std::shared_ptr<FrHydroBody> body) { m_body = body;}

        /// Compute the relative angle
        double SetRelativeAngle(const double waveDir, const double heading);

        /// Initialization of the wave drift force components
        void Initialize() override;

        /// Update procedure containing the Wave Drift Force definition
        void UpdateState() override;

        /// Definition of the prefix used in log file
        void SetLogPrefix(std::string prefix_name) override {
                if (prefix_name=="") {
                        m_logPrefix = "Fwd_" + FrForce::m_logPrefix;
                } else {
                        m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
                }
        }

    };












    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    //forward declaration

    class FrHydroDB_;

    class FrWaveDriftForceRAO_ : public FrForce_ {

    private:

        std::shared_ptr<FrHydroDB_> m_hdb;
        std::vector<std::unique_ptr<mathutils::LookupTable2d<>>> m_table;

    public:

        FrWaveDriftForceRAO_(const FrHydroDB_& hdb);

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

    private:
         void SetInterpolationTable();




    };

// end namespace frydom
}

#endif //FRYDOM_FRWAVEDRIFTFORCE_H
