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

#ifndef FRYDOM_FRMESHTRAITS_H
#define FRYDOM_FRMESHTRAITS_H

#include <iostream>

#include "OpenMesh/Core/IO/MeshIO.hh"
#include "OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"

using namespace OpenMesh;

namespace frydom {
  namespace mesh {


    enum IntegrandType {
      // TODO: voir http://www.drdobbs.com/when-enum-just-isnt-enough-enumeration-c/184403955 pour une meilleure
      // gestion des enums
      UNDEFINED_INTEGRAND,
      POLY_1,
      POLY_X,
      POLY_Y,
      POLY_Z,
      POLY_YZ,
      POLY_XZ,
      POLY_XY,
      POLY_X2,
      POLY_Y2,
      POLY_Z2,
      POLY_X3,
      POLY_Y3,
      POLY_Z3,
      POLY_X2Y,
      POLY_Y2Z,
      POLY_Z2X,
      POLY_Y2X,
//            INFINITE_DEPTH_GREEN_FUNCTION, // TODO : en parler avec Camille et Lucas et PYW
//            FINITE_DEPTH_GREEN_FUNCTION
    };

    enum VertexPosition {  // N'a a priori de sens que lors de la decoupe... Mettre dans FrMeshClipper ?
      // On pourrait du coup plutot utiliser les fonctions d'ajout dynamique de proprietes !!
      VP_ABOVE_SURFACE = 0,
      VP_ON_SURFACE = 1,
      VP_UNDER_SURFACE = 2,
      VP_UNDEFINED = -1
    };

    struct FrMeshTraits : public DefaultTraits {
      typedef Vec3d Point;

      VertexAttributes(Attributes::Normal |
                       Attributes::Status
      );

      FaceAttributes(Attributes::Normal |
                     Attributes::TAGGED |
                     Attributes::FEATURE |
                     Attributes::Status
      );

      HalfedgeAttributes(Attributes::PrevHalfedge |
                         Attributes::Status |
                         Attributes::TAGGED
      );

      EdgeAttributes(Attributes::Status);

      EdgeTraits
      {
       private:
        double m_length = 0.0;

       public:
        EdgeT() {}

        double GetLength() const {
          return m_length;
        }

        void SetLength(double length) {
          m_length = length;
        }

      };

      FaceTraits
      {
       private:
        Point m_center = {0.0, 0.0, 0.0};

        struct SurfaceIntegrals {
          double m_int_1 = 0.; // TODO: calculer int_1, pas fait encore

          double m_int_x = 0.;
          double m_int_y = 0.;
          double m_int_z = 0.;

          double m_int_yz = 0.;
          double m_int_xz = 0.;
          double m_int_xy = 0.;

          double m_int_x2 = 0.;
          double m_int_y2 = 0.;
          double m_int_z2 = 0.;

          double m_int_x3 = 0.;
          double m_int_y3 = 0.;
          double m_int_z3 = 0.;

          double m_int_x2y = 0.;
          double m_int_y2z = 0.;
          double m_int_z2x = 0.;
        };

        SurfaceIntegrals m_integrals;

       public:
        FaceT() {}

        const Point &Center() const { return m_center; }

        /// This function sets the position of the face centroid.
        void SetCenter(const Point &center) { m_center = center; }

        /// This function gives the surface integral of a face.
        double GetSurfaceIntegral(
            IntegrandType type) const { // TODO: abandonner les enums pour les integrandes et preferer les accessors voir mieux, des fonctors...
          double val;
          switch (type) {
            case POLY_1:
              val = m_integrals.m_int_1; // This is the surface area...
              break;
            case POLY_X:
              val = m_integrals.m_int_x;
              break;
            case POLY_Y:
              val = m_integrals.m_int_y;
              break;
            case POLY_Z:
              val = m_integrals.m_int_z;
              break;
            case POLY_YZ:
              val = m_integrals.m_int_yz;
              break;
            case POLY_XZ:
              val = m_integrals.m_int_xz;
              break;
            case POLY_XY:
              val = m_integrals.m_int_xy;
              break;
            case POLY_X2:
              val = m_integrals.m_int_x2;
              break;
            case POLY_Y2:
              val = m_integrals.m_int_y2;
              break;
            case POLY_Z2:
              val = m_integrals.m_int_z2;
              break;
            case POLY_X3:
              val = m_integrals.m_int_x3;
              break;
            case POLY_Y3:
              val = m_integrals.m_int_y3;
              break;
            case POLY_Z3:
              val = m_integrals.m_int_z3;
              break;
            case POLY_X2Y:
              val = m_integrals.m_int_x2y;
              break;
            case POLY_Y2Z:
              val = m_integrals.m_int_y2z;
              break;
            case POLY_Z2X:
              val = m_integrals.m_int_z2x;
              break;
            case UNDEFINED_INTEGRAND:
              std::cerr << "Cannot return value of an UNDEFINED_INTEGRAND" << std::endl;
              break;
            default:
              throw std::runtime_error("Cannot return value of an UNDEFINED_INTEGRAND");
          }
          return val;
        }

        void SetSurfaceIntegral(IntegrandType type, const double &val) {
          switch (type) {
            case POLY_1:
              m_integrals.m_int_1 = val;
              break;
            case POLY_X:
              m_integrals.m_int_x = val;
              break;
            case POLY_Y:
              m_integrals.m_int_y = val;
              break;
            case POLY_Z:
              m_integrals.m_int_z = val;
              break;
            case POLY_YZ:
              m_integrals.m_int_yz = val;
              break;
            case POLY_XZ:
              m_integrals.m_int_xz = val;
              break;
            case POLY_XY:
              m_integrals.m_int_xy = val;
              break;
            case POLY_X2:
              m_integrals.m_int_x2 = val;
              break;
            case POLY_Y2:
              m_integrals.m_int_y2 = val;
              break;
            case POLY_Z2:
              m_integrals.m_int_z2 = val;
              break;
            case POLY_X3:
              m_integrals.m_int_x3 = val;
              break;
            case POLY_Y3:
              m_integrals.m_int_y3 = val;
              break;
            case POLY_Z3:
              m_integrals.m_int_z3 = val;
              break;
            case POLY_X2Y:
              m_integrals.m_int_x2y = val;
              break;
            case POLY_Y2Z:
              m_integrals.m_int_y2z = val;
              break;
            case POLY_Z2X:
              m_integrals.m_int_z2x = val;
              break;
            case UNDEFINED_INTEGRAND:
              std::cerr << "Cannot return value of an UNDEFINED_INTEGRAND" << std::endl;
              break;
            default:
              throw std::runtime_error("Cannot return value of an UNDEFINED_INTEGRAND");
          }
        }

      };

      VertexTraits
      {
       private:
        VertexPosition m_position = VP_UNDEFINED;
        // Remarque: pour les doubles noeuds, on utilisera les attributs de feature sur les elements
        // Il faut etre capable de specifier les features du maillage de maniere externe ou iterne
        // VTK propose un filre permettat a priori de trouver les features d'un maillage

       public:
        VertexT() {}

        const VertexPosition &Position() const { return m_position; }

        /// This function gives the type of position of the vertex (above/below/on the clipping surface).
        void SetPositionType(VertexPosition vPos) { m_position = vPos; }

        /// This function
        void SetAbove() { m_position = VP_ABOVE_SURFACE; }

        void SetOn() { m_position = VP_ON_SURFACE; }

        void SetUnder() { m_position = VP_UNDER_SURFACE; }

        void SetUndefined() { m_position = VP_UNDEFINED; }

        inline bool IsAbove() const {
          return m_position == VP_ABOVE_SURFACE;
        }

        inline bool IsOn() const {
          return m_position == VP_ON_SURFACE;
        }

        inline bool IsUnder() const {
          return m_position == VP_UNDER_SURFACE;
        }

        inline bool IsUndefined() const {
          return m_position == VP_UNDEFINED;
        }
      };

    };

  } // end namespace frydom::mesh
} // end namespace frydom



#endif //FRYDOM_FRMESHTRAITS_H
