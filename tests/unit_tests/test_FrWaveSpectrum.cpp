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

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrWaveSpectrum, FrCos2sDirectionalModel) {


  double thetaMean = 0 * M_PI;
  std::vector<double> thetaVect;
  thetaVect.clear();
  thetaVect = linspace(0., 2. * M_PI, 21);

  // test FrCos2sDirectionalModel
  FrCos2sDirectionalModel Cos2sDirModel(10);

  auto Cos2SDirfunc = Cos2sDirModel.GetSpreadingFunction(thetaVect, thetaMean);

  double s = Cos2sDirModel.GetSpreadingFactor();
  double two_s = 2. * s;
  double c_s = (pow(2., two_s - 1.) / M_PI) * pow(std::tgamma(s + 1.), 2.) / std::tgamma(two_s + 1.);

  double spreading_fcn;
  for (int iv = 0; iv < thetaVect.size(); ++iv) {
    spreading_fcn = c_s * pow(cos(0.5 * (thetaVect[iv] - thetaMean)), two_s);
    EXPECT_NEAR(spreading_fcn, Cos2SDirfunc[iv], 1.E-8);
  }

  double theta_min, theta_max;
  Cos2sDirModel.GetDirectionBandwidth(theta_min, theta_max, thetaMean);
//    std::cout<<"theta_min = "<<theta_min*RAD2DEG<<", theta_max = "<<theta_max*RAD2DEG
//             <<", theta_mean-theta_min = "<<(thetaMean-theta_min)*RAD2DEG<<std::endl;
  EXPECT_NEAR(thetaMean - theta_min, theta_max - thetaMean, 1.E-8);

}


TEST(FrWaveSpectrum, Jonswap) {
  double Hs = 3.;
  double Tp = 9.;
  double Gamma = 3.3;

  std::vector<double> wVect;
  wVect.clear();
  wVect = linspace(0.5, 2., 21);

  FrJonswapWaveSpectrum JonswapSpectrum(Hs, Tp, Gamma);

  auto waveAmp = JonswapSpectrum.GetWaveAmplitudes(wVect);

  double FromPython[21] = {0.1350065, 0.26988954, 0.47368477, 0.57775458, 0.36633141,
                           0.28137033, 0.24320673, 0.21093058, 0.18274343, 0.15865051,
                           0.138254, 0.12103303, 0.10647771, 0.09413711, 0.08363016,
                           0.07464189, 0.06691458, 0.06023814, 0.05444138, 0.04938449,
                           0.04495295};

  for (int i = 0; i < 21; ++i) {
    EXPECT_NEAR(FromPython[i], waveAmp[i], 1.E-8);
  }

  double wmin, wmax;
  JonswapSpectrum.GetFrequencyBandwidth(wmin, wmax);

  double wminFromPython = 0.431574732929136, wmaxFromPython = 2.38620060411016;

  EXPECT_NEAR(wminFromPython, wmin, 1.E-8);
  EXPECT_NEAR(wmaxFromPython, wmax, 1.E-8);

}


TEST(FrWaveSpectrum, OchiHubble) {

  auto to_file = false;
  double Hs = 3.;

  auto wVect = linspace(0.5, 2., 21);

  FrOchiHubbleWaveSpectrum spectrum(Hs);

  std::vector<double> w = {0.28876404494382035, 0.40674157303370795, 0.4539325842696631, 0.47415730337078665,
                           0.49101123595505625, 0.504494382022472, 0.5112359550561798, 0.5280898876404495,
                           0.5415730337078652, 0.551685393258427, 0.5719101123595507, 0.60561797752809,
                           0.6359550561797754, 0.6629213483146068, 0.6764044943820225, 0.703370786516854,
                           0.7235955056179777, 0.7505617977528091, 0.7775280898876405, 0.8382022471910114,
                           0.8955056179775281, 0.9730337078651686, 1.0707865168539326, 1.165168539325843,
                           1.2528089887640452, 1.350561797752809, 1.4752808988764046, 1.5831460674157305,
                           1.7314606741573035, 1.903370786516854, 2.001123595505618};
  std::vector<double> fromOchiPaper = {0.010068296981713853, 0.015598149372108239, 0.10094734523022675,
                                       0.257413527208636, 0.41394580304031736, 0.6019167217448778, 0.7743335536461773,
                                       0.9465521039876621, 1.1737387089667326, 1.3931482705441725, 1.6045164133068959,
                                       1.7215025335977086, 1.6111037673496365, 1.469398545935228, 1.3279576999339064,
                                       1.123507380480282, 0.9427186604979068, 0.8166997135933021, 0.5887199823749727,
                                       0.4463538224278478, 0.36679885437321014, 0.3260630094734518, 0.2927737387089664,
                                       0.24386428728794884, 0.17940074906366998, 0.13042520378938072, 0.096607182198722,
                                       0.06311962987442188, 0.01315267680105725, 0.0019387530293015764,
                                       0.00786516853932584};

  std::ofstream myfile;
  if (to_file) {
    // Print to file
    myfile.open("OchiHubble.csv", std::ios::out);
    myfile << "w;Ochi-Hubble;ref" << std::endl;
    myfile << "rad/s;;" << std::endl;
  }

  for (int jw = 0; jw < w.size(); jw++) {
    EXPECT_NEAR(fromOchiPaper[jw], spectrum.Eval(w[jw]), 0.16);
    if (to_file) {
      myfile << w[jw] << ";" << spectrum.Eval(w[jw]) << ";" << fromOchiPaper[jw] << std::endl;
    }
  }
  if (to_file) {
    myfile.close();
  }

}


