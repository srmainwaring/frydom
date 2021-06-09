add_subdirectory(bspline)
add_subdirectory(functions)

target_sources(frydom PRIVATE
    Fr3DGeometry.cpp
        FrMatrix.cpp
        FrVector.cpp
        FrTorsor.cpp
        )
