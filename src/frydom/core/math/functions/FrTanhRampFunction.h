//
// Created by frongere on 07/02/19.
//

#ifndef FRYDOM_FRTANHRAMP_H
#define FRYDOM_FRTANHRAMP_H

#include "FrFunction.h"

namespace frydom {

    class FrTanhRampFunction : public FrFunction_ {

    private:

        double m_tolerance = 1e-3;

        double m_width = 1.;
        double m_height = 1.;
        double m_yOffset = 0.;

        double c_a;

    public:

        FrTanhRampFunction();

        void SetTolerance(double tolerance);

        void SetWidth(double width);

        void SetHeight(double height);

        void SetYOffset(double yOffset);

        void SetByTwoPoints(double x0, double y0, double x1, double y1);


    private:
        void ComputeA();

    protected:

        void Eval(double x) const override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRTANHRAMP_H
