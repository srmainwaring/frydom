
message(STATUS "    ...Core")

add_subdirectory(body)
add_subdirectory(common)
add_subdirectory(contact)
add_subdirectory(force)
add_subdirectory(link)
add_subdirectory(math)
add_subdirectory(misc)
add_subdirectory(statics)


target_sources(frydom PRIVATE
        FrOffshoreSystem.cpp
        )


#set(SRC_FILES
#        FrOffshoreSystem.cpp
#        )
#
## BODY
#list(APPEND SRC_FILES
#        body/FrBody.cpp
#        body/FrBodyEasy.cpp
#        body/FrInertiaTensor.cpp
#        body/FrAssembly.cpp
#        )
#
## CONTACT
#list(APPEND SRC_FILES
#        contact/FrContactSMC.cpp
#        contact/FrContactNSC.cpp
#        )
#
## COMMON
#list(APPEND SRC_FILES
#        common/FrException.cpp
#        common/FrFrame.cpp
#        common/FrNode.cpp
#        common/FrObject.cpp
#        common/FrPhysicsItem.cpp
#        common/FrFEAMesh.cpp
#        common/FrRotation.cpp
#        common/FrVariablesBodyBase.cpp
#        common/FrVariablesAddedMass.cpp
#        )
#
## FORCE
#list(APPEND SRC_FILES
#        force/FrForce.cpp
#        force/FrConstantForce.cpp
#        )
#
## LINK // TODO : cmakiser les repertoires !!
#list(APPEND SRC_FILES
#
#        link/FrLinkBase.cpp
#        link/FrLinkMaskBase.cpp
#
#        link/links_lib/actuators/FrActuator.cpp
#        link/links_lib/actuators/FrLinearActuator.cpp
#        link/links_lib/actuators/FrAngularActuator.cpp
#
#        link/constraint/FrConstraint.cpp
#        link/constraint/FrCGeometrical.cpp
#        link/constraint/FrConstraintTwoBodiesBase.cpp
#
#        link/links_lib/FrLink.cpp
#        link/links_lib/FrCylindricalLink.cpp
#        link/links_lib/FrFixedLink.cpp
#        link/links_lib/FrFreeLink.cpp
#        link/links_lib/FrPrismaticLink.cpp
#        link/links_lib/FrRevoluteLink.cpp
#        link/links_lib/FrScrewLink.cpp
#        link/links_lib/FrSphericalLink.cpp
#        link/links_lib/FrDOFMaskLink.cpp
#        link/links_lib/FrPrismaticRevoluteLink.cpp
#        )
#
## MATH
#list(APPEND SRC_FILES
#        math/FrMatrix.cpp
#        math/FrVector.cpp
#        math/Torsor.cpp
#
#        math/Fr3DGeometry.cpp
#
#        math/functions/FrFunctionBase.cpp
#
#        math/functions/ramp/FrLinearRampFunction.cpp
#        math/functions/ramp/FrTanhRampFunction.cpp
#        math/functions/ramp/FrCosRampFunction.cpp
#
#        math/functions/lib/FrPowFunction.cpp
#        math/functions/lib/FrLinearFunction.cpp
#        math/functions/lib/FrSaturateFunction.cpp
#        math/functions/lib/FrClampFunction.cpp
#        math/functions/lib/FrPolynomialFunction.cpp
#        math/functions/lib/FrTrigonometricFunctions.cpp
#        math/functions/lib/FrHyperbolicTrigonometricFunctions.cpp
#        math/functions/lib/FrMiscFunctions.cpp
#
##        math/BSpline/FrBSpline.cpp
#        )
#
## MISC
#list(APPEND SRC_FILES
#        misc/FrColors.cpp
#        )
#
## STATICS
#list(APPEND SRC_FILES
#        statics/FrStaticAnalysis.cpp
#        ../hydrodynamics/hydrostatic/FrHydrostaticEquilibrium.cpp
#        )
#
#target_sources(frydom PRIVATE ${SRC_FILES})

#set(FRYDOM_CORE_SOURCES "")
#foreach (file ${SRC_FILES})
#    list(APPEND FRYDOM_CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
#endforeach ()
#
#
#set(FRYDOM_CORE_SOURCES ${FRYDOM_CORE_SOURCES} PARENT_SCOPE)
