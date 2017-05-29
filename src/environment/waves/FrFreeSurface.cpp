// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// Base class for a water free surface system
//
// =============================================================================

#include <chrono/assets/ChTriangleMeshShape.h>
#include "FrFreeSurface.h"
#include "../../misc/FrTriangleMeshConnected.h"

namespace frydom {
namespace environment{

    FrFreeSurface::FrFreeSurface(double p_mean_height) : m_mean_height(p_mean_height){
        plane.pos[1] = p_mean_height;  // The free surface plane reference has the altitude the mean FS height
    }

    double FrFreeSurface::getMeanHeight() const {
       return m_mean_height;
    }

    void FrFreeSurface::Initialize(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

        // Making sure the mesh is clean
        m_mesh.Clear();

        // Building the grid
        build_mesh_grid(xmin, xmax, dx, ymin, ymax, dy);

        if (m_vis_enabled) {
            auto mesh_shape = std::make_shared<chrono::ChTriangleMeshShape>();
            mesh_shape->SetMesh(m_mesh);
            mesh_shape->SetName("Free surface");
        }

    }

    void FrFreeSurface::Initialize(double lmin, double lmax, double dl){
        FrFreeSurface::Initialize(lmin, lmax, dl, lmin, lmax, dl);
    }

    FrTriangleMeshConnected FrFreeSurface::getMesh(void) const {
        return m_mesh;
    }

    void FrFreeSurface::build_mesh_grid(double xmin, double xmax, double dx,
                                        double ymin, double ymax, double dy) {

        // FIXME: Une instance de amillage a deja ete cree a l'instanciation de la classe FrFreeSurface... Il faut la
        // remplir directement

        int nx(int((xmax - xmin) / dx) + 1);
        int ny(int((ymax - ymin) / dy) + 1);

        // Building the vertices list
        std::vector<chrono::ChVector<double>> vertices;
        double xi = xmin,
                yi = ymin;

        for (int iy = 0; iy < ny; iy++) {
            for (int ix = 0; ix < nx; ix++) {
                chrono::ChVector<double> vertex(xi, yi, m_mean_height);
                vertices.push_back(vertex);
                xi += dx;
            }
            yi += dy;
            xi = xmin;
        }
        // Adding the vertices list to the mesh
        m_mesh.addVertex(vertices);

        // Building faces of the cartesian grid
        std::vector<chrono::ChVector<int>> triangles;
        for (int iy = 0; iy < ny - 1; iy++) {
            for (int ix = 0; ix < nx - 1; ix++) {
                int i0(iy * nx + ix);
                int i1(i0 + 1);
                int i2(i1 + nx);
                int i3(i2 + 1);

                chrono::ChVector<int> triangle_1(i0, i1, i2);
                chrono::ChVector<int> triangle_2(i0, i2, i3);

                triangles.push_back(triangle_1);
                triangles.push_back(triangle_2);
            }
        }
        // Adding the triangle list to the mesh
        m_mesh.addTriangle(triangles);

        std::cout << m_mesh.m_vertices.size() << std::endl;
        std::cout << m_mesh.RepairDuplicateVertexes() << std::endl;
        std::cout << m_mesh.m_vertices.size() << std::endl;
        std::cout << m_mesh.getNumTriangles();

    }

}  // end namespace environment
}  // end namespace frydom