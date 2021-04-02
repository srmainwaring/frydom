#BOOST
message(STATUS "******* FINDING BOOST dependency from ${PROJECT_NAME} (requested minimal version: ${boost_TAG}) *******")

FIND_PACKAGE(Boost ${boost_TAG} REQUIRED)
IF(Boost_FOUND)
    INCLUDE_DIRECTORIES ( ${Boost_INCLUDE_DIR} )
    MESSAGE(STATUS "Boost found with version ${Boost_VERSION_STRING}")
ELSE(Boost_FOUND)
    MESSAGE(FATAL_ERROR "Cannot build Boost project without Boost. Please set Boost_DIR.")
ENDIF(Boost_FOUND)