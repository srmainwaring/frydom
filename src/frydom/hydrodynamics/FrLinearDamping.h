//
// Created by frongere on 11/09/17.
//

#ifndef FRYDOM_FRLINEARDAMPING_H
#define FRYDOM_FRLINEARDAMPING_H

#include "MathUtils/Matrix66.h"

#include "frydom/utils/FrEigen.h"
#include "frydom/core/FrForce.h"
#include "frydom/environment/FrFluidType.h"

namespace frydom {

    class FrLinearDamping : public FrForce {

    private:
        Eigen::MatrixXd m_dampings = Eigen::MatrixXd::Zero(6,6);
        bool m_relative2Current = false;
    public:

        FrLinearDamping() {};
        /// Setter for the whole damping matrix
        void SetDampingMatrix(Eigen::MatrixXd dampingMatrix) {
            m_dampings = dampingMatrix;
        }
        /// Setter for the diagonal components of the damping matrix
        void SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr){
            SetDiagonalTranslationDamping(Du, Dv, Dw);
            SetDiagonalRotationDamping(Dp, Dq, Dr);
        }
        /// Setter for the diagonal components in translation of the damping matrix
        void SetDiagonalTranslationDamping(double Du, double Dv, double Dw) {
            m_dampings(0,0) = Du;
            m_dampings(1,1) = Dv;
            m_dampings(2,2) = Dw;
        }
        /// Setter for the diagonal components in rotation of the damping matrix
        void SetDiagonalRotationDamping(double Dp, double Dq, double Dr) {
            m_dampings(3,3) = Dp;
            m_dampings(4,4) = Dq;
            m_dampings(5,5) = Dr;
        }
        /// Setter for a non-diagonal components of the damping matrix at indices (row,column)
        void SetNonDiagonalDamping(int row, int column, double Dnd) {
            assert(row!=column);
            assert(row<6);
            assert(column<6);
            m_dampings(row,column) = Dnd;
        }
        /// Setter for a row of non-diagonal components of the damping matrix
        void SetNonDiagonalRowDamping(int row,const double Dnd[5]){
            assert(row<6);
            int j=0;
            for (int i=1;i<6;i++) {
                if (i==row) continue;
                m_dampings(row,j) = Dnd[i];
                j++;
            }
        }
        /// Setter for a column of non-diagonal components of the damping matrix
        void SetNonDiagonalColumnDamping(int column,const double Dnd[5]){
            assert(column<6);
            int j=0;
            for (int i=1;i<6;i++) {
                if (i==column) continue;
                m_dampings(j,column) = Dnd[i];
                j++;
            }
        }
        /// Setter for the boolean : m_relativeVelocity
        void SetRelative2Current(bool relativeVelocity);
        /// Getter for the boolean : m_relativeVelocity
        bool GetRelative2Current() {return m_relative2Current;}
        /// Setter for the log prefix
        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "FlinDamp_" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }
        // Initialize
        void Initialize() override;
        /// Update the state of the linear damping force (compute the force)
        void UpdateState() override;

    };













    // REFACTORING ---------->>>>>>>>>>>>>>>>


    class FrLinearDamping_ : public FrForce_ {

    public:
        using DampingMatrix = mathutils::Matrix66<double>; // TODO : disposer d'une Matrix66 dans mathutils

    private:

        DampingMatrix m_dampingMatrix;
        FLUID_TYPE m_fluidType;
        bool m_relativeToFluid = false;  // FIXME : on doit pouvoir aussi appliquer dans l'air !!!!!

    public:

        FrLinearDamping_(FLUID_TYPE ft, bool relativeToFluid);

        void SetNull();

        /// Setter for the whole damping matrix. Translations are upper left and rotations are lower right.
        void SetDampingMatrix(const DampingMatrix& dampingMatrix);

        /// Setter for the diagonal components of the damping matrix
        void SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr);

        /// Setter for the diagonal components in translation of the damping matrix
        void SetDiagonalTranslationDamping(double Du, double Dv, double Dw);

        /// Setter for the diagonal components in rotation of the damping matrix
        void SetDiagonalRotationDamping(double Dp, double Dq, double Dr);

        /// Set a damping coefficient given its position.
        void SetDampingCoeff(unsigned int iRow, unsigned int iCol, double coeff);

        /// Setter for the boolean : m_relativeVelocity
        void SetRelativeToFluid(bool isRelative);

        /// Getter for the boolean : m_relativeVelocity
        bool GetRelativeToFluid();

        /// Setter for the log prefix
//        void SetLogPrefix(std::string prefix_name) override {
////            if (prefix_name=="") {
////                m_logPrefix = "FlinDamp_" + FrForce::m_logPrefix;
////            } else {
////                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
////            }
//        }

        void Update(double time) override;

        void Initialize() override;

        void StepFinalize() override;

    private:

        void Check() const;

    };


};  // end namespace frydom
#endif //FRYDOM_FRLINEARDAMPING_H