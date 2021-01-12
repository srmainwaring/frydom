message(STATUS "    ...buoyancy")

set(SRC_FILES
        FrBarElement.cpp
        FrBuoyancyBarElements.cpp
        BuoyancyForce.cpp
        )

set(FRYDOM_BUOYANCY_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_BUOYANCY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

set(FRYDOM_BUOYANCY_SOURCES ${FRYDOM_BUOYANCY_SOURCES} PARENT_SCOPE)