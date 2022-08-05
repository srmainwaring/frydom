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


#include "FrBodyEasy.h"

#include "FrBody.h"
#include "frydom/collision/FrCollisionModel.h"


namespace frydom {

  void makeItBox(std::shared_ptr<FrBody> body,
                 double xSize,
                 double ySize,
                 double zSize,
                 double mass,
                 FrMaterialSurface *mat) {

    // Properties of the box
    double xSize2 = xSize * xSize;
    double ySize2 = ySize * ySize;
    double zSize2 = zSize * zSize;

    // inertia
    double Ixx = (1. / 12.) * mass * (ySize2 + zSize2);
    double Iyy = (1. / 12.) * mass * (xSize2 + zSize2);
    double Izz = (1. / 12.) * mass * (xSize2 + ySize2);

    // Building the chrono body
    body->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., Position(), NWU));

    // Collision
    if (mat) {
      auto collisionModel = std::make_shared<FrCollisionModel>();
      collisionModel->ClearModel();
      collisionModel->AddBox(mat, xSize * 0.5, ySize * 0.5, zSize * 0.5, Position(), FrRotation());
      body->SetCollisionModel(collisionModel);
    }

    // Asset
    body->AddBoxShape(xSize, ySize, zSize, {0., 0., 0.}, NWU);

  }

  void makeItCylinder(std::shared_ptr<FrBody> body,
                      double radius,
                      double height,
                      double mass,
                      FrMaterialSurface *mat) {

    // Properties of the cylinder
    double r2 = radius * radius;
    double h2 = height * height;
    double Ixx = (1. / 12.) * mass * (3. * r2 + h2);  // FIXME : attention, on a pas les bons ordres !!
    double Iyy = 0.5 * mass * r2;
    double Izz = Ixx;

    // Building the chrono body
    body->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., Position(), NWU));

    // Collision
    if (mat) {
      auto collisionModel = std::make_shared<FrCollisionModel>();
      collisionModel->ClearModel();
      collisionModel->AddCylinder(mat, radius, radius, height * 0.5, Position(), FrRotation());
      body->SetCollisionModel(collisionModel);
    }

    // Asset
    body->AddCylinderShape(radius, height, {0., 0., 0.}, NWU);

  }

  void makeItSphere(std::shared_ptr<FrBody> body,
                    double radius,
                    double mass,
                    FrMaterialSurface *mat) {

    // Properties of the sphere
    double inertia = (2.0 / 5.0) * mass * radius * radius;

    // Building the Chrono body
    body->SetInertiaTensor(FrInertiaTensor(mass, inertia, inertia, inertia, 0., 0., 0., Position(), NWU));

    // Collision
    if (mat) {
      auto collisionModel = std::make_shared<FrCollisionModel>();
      collisionModel->ClearModel();
      collisionModel->AddSphere(mat, radius, Position());
      body->SetCollisionModel(collisionModel);
    }

    // Asset
    body->AddSphereShape(radius, {0., 0., 0.}, NWU);

  }

  std::shared_ptr<FrBody> make_BoxBody(const std::string &name,
                                       FrOffshoreSystem *system,
                                       double xSize,
                                       double ySize,
                                       double zSize,
                                       double mass,
                                       FrMaterialSurface *mat) {

    auto box = std::make_shared<FrBody>(name, system);
    makeItBox(box, xSize, ySize, zSize, mass, mat);
    return box;
  }

  std::shared_ptr<FrBody> make_CylinderBody(const std::string &name,
                                            FrOffshoreSystem *system,
                                            double radius,
                                            double height,
                                            double mass,
                                            FrMaterialSurface *mat) {

    auto cylinder = std::make_shared<FrBody>(name, system);
    makeItCylinder(cylinder, radius, height, mass, mat);
    return cylinder;
  }

  std::shared_ptr<FrBody> make_SphereBody(const std::string &name,
                                          FrOffshoreSystem *system,
                                          double radius,
                                          double mass,
                                          FrMaterialSurface *mat) {

    auto sphere = std::make_shared<FrBody>(name, system);
    makeItSphere(sphere, radius, mass, mat);
    return sphere;
  }

}  // end namespace frydom
