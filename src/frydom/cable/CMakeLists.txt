message(STATUS "    ...Cable")

set(SRC_FILES
        FrCableProperties.cpp
        FrCableBase.cpp
        FrCatenaryLineBase.cpp
        FrCatenaryLine.cpp
        FrCatenaryLineSeabed.cpp
        FrMooringBuoy.cpp
        FrLumpedMassCable.cpp
        FrCableShapeInitializer.cpp
        FrAnchor.cpp
        FrClumpedMass.cpp

        FrFEACable.cpp
        FrFEACableLoads.cpp
        FrFEACableBuilder.cpp


)

set(FRYDOM_CABLE_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_CABLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

#message(${FRYDOM_CABLE_SOURCES})

set(FRYDOM_CABLE_SOURCES ${FRYDOM_CABLE_SOURCES} PARENT_SCOPE)
