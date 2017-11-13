//
// Created by frongere on 30/10/17.
//

#include "chrono/timestepper/ChTimestepper.h"

#include <frydom/hydrodynamics/FrLinearHydrostaticForce.h>
#include <frydom/hydrodynamics/FrLinearDamping.h>
#include "frydom/core/FrCore.h"
#include "frydom/hydrodynamics/FrHydroDB.h" // TODO: charger seulement un FrHydrodynamics
#include "frydom/hydrodynamics/FrLinearExcitationForce.h"

#include "frydom/environment/waves/FrWaveField.h"

#include "frydom/utils/FrIrrApp.h"


using namespace frydom;

int main(int argc, char* argv[]) {


    // TODO
    // 1 creer le systeme: il doit avoir une surface libre, un seabed, un modele de maree, un modele de courant, un modele de vent

    // TODO: le systeme devrait etre un singleton de maniere a ce que chaque objet necessitant un enregistrement aupres de systeme
    // puisse le faire de son propre chef...
    // Du coup, systeme devrait etre cree lors de l'appel a un FRyDoM_INIT()
    // Est ce qu'on ne peut pas definir un objet frydom qui soit un singleton et qui donne alors access au systeme embarque ??? --> mieux !!
    FrOffshoreSystem system;

    auto freeSurface = system.GetFreeSurface();
//    freeSurface->Initialize(-20, 20, 4);
    freeSurface->Initialize(-20, 20, 2, -20, 20, 40);
//    freeSurface->Initialize(0, 0, 40, 20, 60);

    // Set the wave field
//    freeSurface->SetLinearWaveField(LINEAR_REGULAR);
//    auto waveField = freeSurface->GetLinearWaveField();
//    waveField->SetMeanWaveDirection(0., DEG);
//    waveField->SetRegularWaveHeight(2);
//    waveField->SetRegularWavePeriod(6, S);


    // TODO: permettre de regler le hs et le tp...
    freeSurface->SetLinearWaveField(LINEAR_IRREGULAR);
    auto waveField = freeSurface->GetLinearWaveField();
    waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
    double wmin = 0.1;
    double wmax = 3.;
    unsigned int nbFreq = 120;
    waveField->SetWavePulsations(wmin, wmax, nbFreq, RADS);
    waveField->GetWaveSpectrum()->SetHs(1.);
    waveField->GetWaveSpectrum()->SetTp(6.);

    waveField->GetWaveRamp()->SetDuration(40.);
    waveField->GetWaveRamp()->SetIncrease();


//    freeSurface->SetLinearWaveField(LINEAR_DIRECTIONAL);
//    auto waveField = freeSurface->GetLinearWaveField();
//    waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
//    double wmin = 0.2;
//    double wmax = 2.;
//    unsigned int nbFreq = 70;
//    waveField->SetWavePulsations(wmin, wmax, nbFreq, RADS);

    freeSurface->UpdateAssetON();

//    waveField->GetWaveSpectrum()->Eval(1.);
//    waveField->SetRegularWaveHeight(0.5);
//    waveField->SetRegularWavePeriod(5., S);
//    waveField->GetWaveSpectrum()->



    // Get a waveProbe for excitation force on body
    auto waveProbe = waveField->NewWaveProbe(0, 0);


    // 2 creer un corps hydro
    auto cylinder = std::make_shared<FrHydroBody>();
    cylinder->SetName("Cylinder");
    cylinder->SetHydroMesh("../src/frydom/tests/data/Cylinder.obj", true);
    cylinder->SetLateralUnderWaterArea(100.);
    cylinder->SetTransverseUnderWaterArea(100.);
    cylinder->SetLpp(10.);
    cylinder->SetInertiaXX(chrono::ChVector<double>(5e7, 5e7, 1e6));
    cylinder->SetMass(795e3 * 2.);
    cylinder->SetCOG(chrono::ChVector<double>(0., 0., -7.5));
//    cylinder->SetBodyFixed(true);  // TODO: retirer
    system.AddBody(cylinder);

    // ===========================================
    // Hydrodynamics
    // ===========================================

    // Set added mass





    // Hydrostatics
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    auto hstStiffness = hstForce->GetStiffnessMatrix();
    double K33 = 7.7922E+05;
    double K44 = 3.4146E+07;
    hstStiffness->SetDiagonal(K33, K44, K44);
    cylinder->AddForce(hstForce);

    // Linear Damping
    auto hydroDampingForce = std::make_shared<FrLinearDamping>();
    hydroDampingForce->SetManeuveuringDampings(chrono::ChVector<double>(1e8, 1e8, 1e9));
    hydroDampingForce->SetSeakeepingDampings(chrono::ChVector<double>(1e6, 1e8, 1e8));
    cylinder->AddForce(hydroDampingForce);

    // Importer une base de donnees hydro
    FrHydroDB HDB = LoadHDB5("../tools/frydom_hdb.h5");

    // Linking the physical floating body to its counterpart from the HDB
    cylinder->SetBEMBody(HDB.GetBody(0));

    // Creer un modele de force d'excitation et l'ajouter au corps hydro
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    cylinder->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);
    excForce->Initialize();


//    double dt = 0.01;
//    for (int i=0; i<10; ++i) {
//        system.DoStepDynamics(dt);
//    }

    // TODO: tester le solveur EULER_IMPLICIT_PROJECTED
    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    auto app = FrIrrApp(system, 30);

    app.Run();


    return 0;
}