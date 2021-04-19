message(STATUS "    ...shape")

target_sources(frydom PRIVATE
        FrBoxShape.cpp
        FrCylinderShape.cpp
        FrSphereShape.cpp
        FrTriangleMeshShape.cpp
        )
