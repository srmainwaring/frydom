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

#include <iostream>
#include <frydom/logging/FrEventLogger.h>

#include "FrSeabed.h"

#include "frydom/asset/FrSeabedGridAsset.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"

#include "frydom/core/body/FrBodyEasy.h"

#include "frydom/cable/FrAnchor.h"


namespace frydom {

  // FrSeabed descriptions

  FrSeabed::FrSeabed(FrOcean *ocean) : m_ocean(ocean) {
  }

  FrOcean *FrSeabed::GetOcean() const { return m_ocean; }

  bool FrSeabed::IsInfiniteDepth() { return m_is_infinite_depth; }

  bool FrSeabed::IsAboveSeabed(const Position &world_position, FRAME_CONVENTION fc) const {
    if (m_is_infinite_depth)
      return true; // Always true...

    double dz = world_position.z() - GetBathymetry(world_position.x(), world_position.y(), fc);
    if (IsNED(fc)) {
      dz = -dz;
    }

    return dz > 0.;
  }

  bool FrSeabed::IsOnSeabed(const Position &world_position,
                            FRAME_CONVENTION fc,
                            const double rtol,
                            const double atol) const {
    if (m_is_infinite_depth)
      return false;  // Always false

    return mathutils::IsClose(world_position.z(), GetBathymetry(world_position.x(), world_position.y(), fc), rtol,
                              atol);
  }

  // TODO: voir a utiliser un FrAnchor...
  std::shared_ptr<FrNode> FrSeabed::NewAnchor(const std::string &name, double x, double y, FRAME_CONVENTION fc) {

    if (m_is_infinite_depth) {
      event_logger::error("Seabed", "",
                          "Trying to place an anchor with an infinite seabed is impossible (while placing anchor named {})",
                          name);
      exit(EXIT_FAILURE);
    }

    Position anchor_position = {x, y, GetBathymetry(x, y, fc)};

    // FIXME: plutot permettre d'ajouter des asset sur les FrNode !! Du coup, on ne cree pas de nouveau corps pour visualiser.
    auto anchor_body = m_ocean->GetEnvironment()->GetSystem()->NewBody(name + "_body");
    anchor_body->SetPosition(anchor_position, fc);
    makeItBox(anchor_body, 1, 1, 1, 1);
    anchor_body->SetFixedInWorld(true);
    // TODO: mettre asset d'ancre...

    auto anchor_node = anchor_body->NewNode(name);

    m_anchors.push_back(anchor_node);

    return anchor_node;

  }

//  //------------------------------------------------------------------------------------------------------------------
//  // FrNullSeabed descriptions
//
//  FrSeabedGridAsset *FrNullSeabed::GetSeabedGridAsset() {
//    try { throw FrException("a null seabed cannot return a seabed asset."); }
//    catch (FrException &e) {
//      std::cout << e.what() << std::endl;
//      exit(EXIT_FAILURE);
//    }
//  }
//
//  FrNullSeabed::FrNullSeabed(FrOcean *ocean) : FrSeabed(ocean) { m_is_infinite_depth = true; }
//
//  void FrNullSeabed::SetBathymetry(double bathymetry, FRAME_CONVENTION fc) {
//    if (m_is_infinite_depth) {
//      event_logger::error("Seabed", "", "Request on bathymetry when water depth is infinite is forbidden");
//      exit(EXIT_FAILURE);
//    }
//  }
//
//  double FrNullSeabed::GetBathymetry(FRAME_CONVENTION fc) const {
//    if (m_is_infinite_depth) {
//      event_logger::error("Seabed", "", "Request on bathymetry when water depth is infinite is forbidden");
//      exit(EXIT_FAILURE);
//    }
//  }
//
//  double FrNullSeabed::GetBathymetry(double x, double y, FRAME_CONVENTION fc) const {
//    if (m_is_infinite_depth) {
//      event_logger::error("Seabed", "", "Request on bathymetry when water depth is infinite is forbidden");
//      exit(EXIT_FAILURE);
//    }
//  }
//
//  void FrNullSeabed::Update(double time) {
//
//  }
//
//  void FrNullSeabed::Initialize() {
//
//  }
//
//  void FrNullSeabed::StepFinalize() {
//
//  }

  //------------------------------------------------------------------------------------------------------------------
  // FrFlatSeabed descriptions

  FrFlatSeabed::FrFlatSeabed(FrOcean *ocean) :
      FrSeabed(ocean),
      m_showSeabed(false),
      m_bathymetry(0.) {

    m_is_infinite_depth = true;
    m_SeabedGridAsset = std::make_shared<FrSeabedGridAsset>(this);

  }

  void FrFlatSeabed::Show(bool val) {
    if (val) {
      m_showSeabed = true;
    } else {
      m_SeabedGridAsset = nullptr;
      m_showSeabed = false;
    }
  }

  void FrFlatSeabed::SetBathymetry(double bathymetry, FRAME_CONVENTION fc) {
    if (IsNED(fc)) { bathymetry = -bathymetry; }
    event_logger::info("Seabed", "Flat seabed", "Set to {} meters", bathymetry);
    m_bathymetry = bathymetry;
    m_is_infinite_depth = false;
  }

  double FrFlatSeabed::GetBathymetry(FRAME_CONVENTION fc) const {
    if (m_is_infinite_depth) {
      std::cerr << "Requesting bathymetry on an infinite seabed" << std::endl; // TODO: a retirer
      event_logger::error("Seabed", "", "Requesting bathymetry on an infinite seabed");
      assert(false); // TODO: RETIRER
      exit(EXIT_FAILURE);
    }

    double bathy = m_bathymetry;
    if (IsNED(fc)) { bathy = -bathy; }
    return bathy;
  }

  double FrFlatSeabed::GetBathymetry(double x, double y, FRAME_CONVENTION fc) const {
    if (m_is_infinite_depth) {
      std::cerr << "Requesting bathymetry on an infinite seabed" << std::endl; // TODO: a retirer
      event_logger::error("Seabed", "", "Requesting bathymetry on an infinite seabed");
      assert(false); // TODO: RETIRER
      exit(EXIT_FAILURE);
    }

    double bathy = m_bathymetry;
    if (IsNED(fc)) { bathy = -bathy; }
    return bathy;
  }

  void FrFlatSeabed::Update(double time) {}

  void FrFlatSeabed::Initialize() {
    if (m_showSeabed) {
      m_SeabedGridAsset->Initialize();
      m_ocean->GetEnvironment()->GetSystem()->GetWorldBody()->AddAsset(m_SeabedGridAsset);
    } else {
//      m_SeabedGridAsset->SetNoGrid();
    }

    CreateContactBox();
  }

  void FrFlatSeabed::StepFinalize() {}

  void FrFlatSeabed::CreateContactBox() {
    auto system = GetOcean()->GetEnvironment()->GetSystem();

    // Making the seabed contactable
    m_seabed_body = std::make_shared<FrBody>("seabed_body", system);
    m_seabed_body->SetFixedInWorld(true);
    m_seabed_body->LogThis(false);

    // FIXME: rendre possible le reglage externe de ces infos !!

    // Chrono defaults
//        young_modulus(2e5),
//        poisson_ratio(0.3f),
//        static_friction(0.6f),
//        sliding_friction(0.6f),
//        restitution(0.4f),
//        constant_adhesion(0),
//        adhesionMultDMT(0),
//        kn(2e5),
//        kt(2e5),
//        gn(40),
//        gt(20)
    auto surface_material = std::make_shared<chrono::ChMaterialSurfaceSMC>();
    surface_material->SetYoungModulus(2e12f);
    surface_material->SetFriction(0.3f); // Devrait venir du modele de cable...
    surface_material->SetRestitution(0.0f);
    surface_material->SetAdhesion(0);
    surface_material->SetKn(2e12);
    surface_material->SetGn(1e6);
//    surface_material->SetKt(2e6);


    auto collision_model = m_seabed_body->GetChronoBody()->GetCollisionModel();
    collision_model->ClearModel();

    double hx = 2000;
    double hy = 2000;
    double hz = 100;

    collision_model->AddBox(0.5*hx, 0.5*hy, 0.5*hz, {0., 0., -0.5*hz});
    collision_model->BuildModel();

    m_seabed_body->AllowCollision(true);
    m_seabed_body->SetSmoothContact();

//    m_seabed_body->AddBoxShape(hx, hy, hz, {0, 0, -0.5*hz}, NWU);

    system->Add(m_seabed_body);

    m_seabed_body->SetPosition({0., 0., GetBathymetry(NWU)}, NWU);

  }

  FrSeabedGridAsset *FrFlatSeabed::GetSeabedGridAsset() { return m_SeabedGridAsset.get(); }


}  // end namespace frydom
