message(STATUS "    ...Cable")

set(SRC_FILES
        common/FrCableProperties.cpp
        common/FrCableBase.cpp
        common/FrCableShapeInitializer.cpp

        catenary/FrCatenaryLineBase.cpp
        catenary/FrCatenaryLine.cpp
        catenary/FrCatenaryLineSeabed.cpp

        lumped/FrLumpedMassCable.cpp

        mooring_components/FrAnchor.cpp
        mooring_components/FrMooringBuoy.cpp
        mooring_components/FrClumpedMass.cpp

        fea/FrFEACable.cpp
        fea/FrFEACableLoads.cpp
        fea/FrFEACableBuilder.cpp

)

set(FRYDOM_CABLE_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_CABLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

#message(${FRYDOM_CABLE_SOURCES})

set(FRYDOM_CABLE_SOURCES ${FRYDOM_CABLE_SOURCES} PARENT_SCOPE)
