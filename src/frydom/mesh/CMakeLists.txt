message(STATUS "    ...mesh")

target_sources(frydom PRIVATE
        FrTriangleMeshConnected.cpp
        FrCache.cpp
        FrMesh.cpp
        ../hydrodynamics/hydrostatic/FrHydrostaticsProperties.cpp
        FrMeshClipper.cpp
        FrHydroMesh.cpp
        FrIncrementalMeshWriter.cpp
        FrPlane.cpp
        FrPolygon.cpp
        )
