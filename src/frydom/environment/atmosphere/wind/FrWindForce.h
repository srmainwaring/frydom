//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRWINDFORCE_H
#define FRYDOM_FRWINDFORCE_H

#include "frydom/core/FrForce.h"

#include "MathUtils/MathUtils.h"

namespace frydom {


    class FrWindForce : public FrForce {

    public:

        /// Constructor from yaml file
        explicit FrWindForce(const std::string& yaml_file);

        /// Read the drag and lift coefficient from yaml file
        void ReadTable(const std::string& yaml_file);

        //
        //  UPDATE
        //

        /// Update procedure containing implementation of the wind drag force model
        void UpdateState() override;

        //
        // LOG
        //

        void SetLogPrefix(std::string prefix_name) override;

    private:

        chrono::ChVector<> m_wind_relative_velocity;    ///< Relative velocity of the wind inflow
        double m_wind_relative_angle;                   ///< Relative angle of the wind
        mathutils::LookupTable1D<double, double> m_table;       ///< table of coefficient

    };








    // REFACTORING ------>>>>>>>>>>>>>>

    class FrWindForce_ : public FrForce_ {


    private:
        mathutils::LookupTable1D<double, double> m_table;

    public:

        /// Default constructor
        FrWindForce_() = default;

        /// Constructor from the yaml file
        explicit FrWindForce_(std::string yamlFile);

        //
        //  UPDATE
        //

        /// Update the state of the force
        void Update(double time) override;

        void Initialize() override {};

        void StepFinalize() override {};

    private:

        /// Read the drag and lift coefficient from yaml file
        void ReadTable(std::string yamlFile);

    };


}  // end namespace frydom
#endif //FRYDOM_FRWINDFORCE_H