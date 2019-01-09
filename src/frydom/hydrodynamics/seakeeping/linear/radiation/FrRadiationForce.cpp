//
// Created by frongere on 17/10/17.
//

#include "FrRadiationForce.h"

#include "FrRadiationModel.h"


namespace frydom {


    FrRadiationForce::FrRadiationForce(std::shared_ptr<FrRadiationModel> radiationModel) : m_radiationModel(radiationModel) {}

    void FrRadiationForce::SetRadiationModel(const std::shared_ptr<FrRadiationModel> radiationModel) { m_radiationModel = radiationModel; }

    std::shared_ptr<FrRadiationModel> FrRadiationForce::GetRadiationModel() const { return m_radiationModel; }

    void FrRadiationForce::SetLogPrefix(std::string prefix_name) {
        if (prefix_name=="") {
            m_logPrefix = "Frad_" + FrForce::m_logPrefix;
        } else {
            m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
        }
    }

    FrRadiationConvolutionForce::FrRadiationConvolutionForce(
            std::shared_ptr<FrRadiationConvolutionModel> radiationConvolutionModel)
            : FrRadiationForce(radiationConvolutionModel) {}

    void FrRadiationConvolutionForce::Initialize() {
        m_radiationModel->Initialize();
        FrRadiationForce::Initialize();
    }

    void FrRadiationConvolutionForce::UpdateState() {
        // TODO: appeler le Update du RadiationModel
        m_radiationModel->Update(ChTime);  // TODO: verifier que le ChTime est le bon temps courant !!

        // Current Hydrodynamic body
        auto hydroBody = dynamic_cast<FrHydroBody*>(GetBody());

        // Get the forces
        m_radiationModel->GetRadiationForce(hydroBody, force, moment);

        moment = hydroBody->Dir_World2Body(moment);  // Moment expressed in the local coordinate frame
        // TODO: verifier que c'est la bonne fonction

        // moment in local
//            force = m_radiationModel->GetRadiationForce(hydroBody);
//            moment = m_radiationModel->GetRadiationMoment(hydroBody);

    }











    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING


    // --------------------------------------------------
    // FrRadiationForce
    // --------------------------------------------------

    FrRadiationForce_::FrRadiationForce_(const std::shared_ptr<FrRadiationModel_> radiationModel)
            : m_radiationModel(radiationModel) {}

    void FrRadiationForce_::SetRadiationModel(const std::shared_ptr<FrRadiationModel_> radiationModel) {
        m_radiationModel = radiationModel;
    }

    // --------------------------------------------------
    // FrRadiationConvolutionForce
    // --------------------------------------------------

    FrRadiationConvolutionForce_::FrRadiationConvolutionForce_(
            std::shared_ptr<FrRadiationConvolutionModel_> radiationModel)
            : m_radiationModel(radiationModel) {}

    void FrRadiationConvolutionForce_::Initialize() {
        m_radiationModel->Initialize();
        FrRadiationForce_::Initialize();
    }

    void FrRadiationConvolutionForce_::Update(double time) {
        m_radiationModel->Update(time);

        auto force = m_radiationModel->GetRadiationForce(m_body);
        auto torque = m_radiationModel->GetRadiationTorque(m_body);

        SetForceTorqueInBodyAtCOG(force, torque, NWU);
    }



}  // end namespace frydom
