
#BOOST
FIND_PACKAGE(Boost 1.66 REQUIRED)
IF(Boost_FOUND)
    INCLUDE_DIRECTORIES ( ${Boost_INCLUDE_DIR} )
    MESSAGE(STATUS "...Boost found")
ELSE(Boost_FOUND)
    MESSAGE(FATAL_ERROR "Cannot build Boost project without Boost. Please set Boost_DIR.")
ENDIF(Boost_FOUND)