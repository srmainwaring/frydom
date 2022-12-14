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
#include <utility>
#include "FrPolygon.h"
#include "FrMesh.h"
#include "FrMeshClipper.h"

namespace frydom {

  namespace mesh {

    FrPolygon::FrPolygon(const std::vector<Position> &vertexList, FRAME_CONVENTION fc) {

      m_vertexList = vertexList;
      if (IsNED(fc)) {
        for (auto &vertex:m_vertexList)
          vertex = internal::SwapFrameConvention(vertex);
      }

      c_planar = CheckPlanar();

      ComputeSurfacePolynomialIntegrals();
    }

    std::vector<Position> FrPolygon::GetVertexList(FRAME_CONVENTION fc) const {

      auto vertexList = m_vertexList;
      if (IsNED(fc)) {
        for (auto &vertex:vertexList)
          vertex = internal::SwapFrameConvention(vertex);
      }

      return vertexList;

//            std::vector<Position> vertexList;
//
//            auto vertex = OpenMeshPointToVector3d<Position>(m_mesh->point(m_mesh->from_vertex_handle(m_polygon[0])));
//            vertexList.push_back(vertex);
//
//            for (auto& heh : m_polygon) {
//                vertex = OpenMeshPointToVector3d<Position>(m_mesh->point(m_mesh->to_vertex_handle(heh)));
//                vertexList.push_back(vertex);
//            }
//
//            return vertexList;
    }

    double FrPolygon::GetArea() const {
      return GetSurfaceIntegral(POLY_1);
    }

    void FrPolygon::ComputeSurfacePolynomialIntegrals() {

      assert(IsPlanar());

      double Int1, IntX, IntY, IntXY, IntX2, IntY2, IntX2Y, IntY2X, IntX3, IntY3;
      Int1 = IntX = IntY = IntXY = IntX2 = IntY2 = 0;

      Position pos0, pos1;
      double x0, x1, y0, y1;
      double dx, dy, px, py, a, b;

      auto vertexInPlane = GetVertexInPlane();

      pos0 = vertexInPlane[0];

      for (long unsigned int i = 1; i < vertexInPlane.size(); i++) {

        pos1 = vertexInPlane[i];
        x0 = pos0[0];
        y0 = pos0[1];

        x1 = pos1[0];
        y1 = pos1[1];

        dx = x1 - x0;
        dy = y1 - y0;
        px = x0 + x1;
        py = y0 + y1;
        a = x0 * x0 + x1 * x1;
        b = y0 * y0 + y1 * y1;

        Int1 += dy * px;
        IntX += dy * (px * px - x0 * x1);
        IntY += dx * (py * py - y0 * y1);
//        IntXY += dy * (py * a + 2 * px * (x0 * y0 + x1 * y1)); missing terms from FR analytical dev.
        IntXY += dy * (py * px * px + 2 * (y0 * x0 * x0 + y1 * x1 * x1));
        IntX2 += dy * a * px;
        IntY2 += dx * b * py;
        IntX2Y += dy *
                  (py * std::pow(px, 3.) + 3. * std::pow(x0, 3.) * y0 + 3. * std::pow(x1, 3.) * y1 - x0 * x0 * x1 * y1 -
                   x0 * x1 * x1 * y0);
        IntY2X += dx *
                  (px * std::pow(py, 3.) + 3. * std::pow(y0, 3.) * x0 + 3. * std::pow(y1, 3.) * x1 - y0 * y0 * y1 * x1 -
                   y0 * y1 * y1 * x0);
        IntX3 += dy * (std::pow(x0, 4.) + std::pow(x0, 3.) * x1 + x0 * std::pow(x1, 3.) + std::pow(x1, 4.));
        IntY3 += dx * (std::pow(y0, 4.) + std::pow(y0, 3.) * y1 + y0 * std::pow(y1, 3.) + std::pow(y1, 4.));

        pos0 = pos1;

      }

      Int1 /= 2.;
      IntX /= 6.;
      IntY /= -6.;
      IntXY /= 24.;
      IntX2 /= 12.;
      IntY2 /= -12.;
      IntX2Y /= 60.;
      IntY2X /= -60.;
      IntX3 /= 20.;
      IntY3 /= -20.;

      c_surfaceIntegrals = PolygonSurfaceIntegrals(Int1, IntX, IntY, IntXY, IntX2, IntY2, IntX2Y, IntY2X, IntX3, IntY3);

    }

    PolygonSurfaceIntegrals FrPolygon::GetSurfaceIntegrals() const {
      return c_surfaceIntegrals;
    }

    double FrPolygon::GetSurfaceIntegral(IntegrandType type) const {
      return c_surfaceIntegrals.GetSurfaceIntegral(type);
    }

//        bool FrPolygon::CheckBoundaryPolygon(FrClippingPlane *plane) const {
//
//            auto valid = !m_polygon.empty();
//
//            for (auto& heh : m_polygon) {
//                auto P1 = m_mesh->point(m_mesh->from_vertex_handle(heh));
//                auto distance = plane->GetDistance(P1);
//                valid &= (distance<1E-8);
//            }
//
//            return false;
//        }

    bool FrPolygon::CheckPlanar() const {

      auto plane = GetPlane();

      bool planar = true;

      for (auto &vertex : m_vertexList) {
        auto distance = plane.GetDistanceToPoint(vertex, NWU);
        planar &= (distance < 1E-4);
      }

      return planar;
    }

    bool FrPolygon::IsPlanar() const {
      return c_planar;
    }

    std::vector<Position> FrPolygon::GetVertexInPlane() const {
      std::vector<Position> vertexInPlane;

      auto plane = GetPlane();

      for (auto &vertex : m_vertexList) {
        auto pos = plane.GetFrame().GetPointPositionInFrame(vertex, NWU);
        vertexInPlane.push_back(pos);
      }

      return vertexInPlane;
    }

    geom::FrPlane FrPolygon::GetPlane() const {
      return geom::FrPlane(m_vertexList, NWU);
    }

    Fr2DAABB FrPolygon::GetBoundingBox() const {
      Fr2DAABB bbox;

      //FIXME : plut??t utiliser GetVertexInPlane? --> semi-OOBB?

      bbox.xmin = m_vertexList[0].GetX();
      bbox.xmax = m_vertexList[0].GetX();
      bbox.ymin = m_vertexList[0].GetY();
      bbox.ymax = m_vertexList[0].GetY();

      for (auto &vertex : m_vertexList) {
        bbox.xmin = fmin(bbox.xmin, vertex.GetX());
        bbox.xmax = fmax(bbox.xmax, vertex.GetX());
        bbox.ymin = fmin(bbox.ymin, vertex.GetY());
        bbox.ymax = fmax(bbox.ymax, vertex.GetY());
      }
      return bbox;
    }


  } // end namespace frydom::mesh

} // end namespace frydom
