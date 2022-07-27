# Changelog

This is the Changelog of the FRyDom framework.
This file should be kept up to date following [these guidelines](https://keepachangelog.com/en/1.0.0/)

## [Unreleased]

### Added

### Changed

### Fixed

## [4.0] 2022-07-26

### Major change

Update the chrono version to v5.0

### Added

- New Abkowitz manoeuvring model (Yoshimura 2012)
- New solver type from chrono v5 : `GMRES`
- Add solver max iterations and tolerance setters/getters in `FrOffshoreSystem` (consequently of the update to chrono v5) : `SetSolverMaxIterations` and `GetSolverMaxIterations` set and get the maximum number of iterations of the iterative solver. `SetSolverTolerance` and `GetSolverTolerance` set and get the maximum tolerance threshold of the iterative solver (definition depends on the iterative solver type).

### Changed
- Update to chrono v5 :
With this update, the vectors `Translation`, `Position`, `CardanAngles`, `GeneralizedPosition`, `Direction`, `Velocity`, `AngularVelocity`, `GeneralizedVelocity`, `Acceleration`, `AngularAcceleration`, `GeneralizedAcceleration`, `Force`, `Torque` and `GeneralizedForce` defined in `FrForce.h` which depend previously on `mathutils::vector`, depend now on `chrono::VectorN`. That imply a modification of vector manipulation on frydom "Base Class", with eigen vector operations, since `chrono::VectorN` derived on eigen matrix. That avoid modification of the type of vector between base class (derived from chrono) and frydom class. Note that `FrForce` is still based on `chrono::ChVector` since manipulation of force in chrono has not been  merged with eigen.
- Eigen3 is loaded directly by FRyDoM by `Add_Eigen3` instead mathutils or chrono. Version number must be greater or equal than 3.3.7. 
- When using `SwapFrameConvention` it is recommanded to use the return value of the function instead the argument vector since error on execution has been observed when no return value is used.
- The method `GetConstraintViolation` in `FrLink` has been deactivated since this method is no longer present in the base class. 
- The `Initialize` method of `FrOffshoreSystem` has been modified to be consitent with the modification of `chrono::ChSystem`. `SetupInitialize`is used instead `ExecuteControlforUpdate`.
- The new version of chrono enables partial update during time integration. In FRyDoM, update of all components at each time step is forced.
- The log of `violationResidual` and `LagrangeResidual` are deactivated in offshore system log since there are not present in the base class.
- The list of solver name has been updated to be consistent with the new solver names used by chrono (`SOR`->`PSOR`, `SYMMSOR`->`PSSOR`, `JACOBI`->`PJACOBI`). Solvers `PCG` and `SOLVER_SMC` are no longer used and have been removed from the list.
- The method `GetProjectedAngleAroundZ` which was previously a method of vector (`Direction`) is now a static method applied to a vector. 
- The function `SetSolverMaxIterSpeed` in `FrOffshoreSystem` has been deactivated and replaced by `SetSolverMaxIterations` (consequently of the update to chrono v5)
- The function `SetSolverMaxIterStab` in `FrOffshoreSystem` has been removed (seems it was not used).
- The function `SetSolverGeometricToloreance`in `FrOffshoreSystem` has been removed (seems it was not used).
- Propulsion models updated to follow acme evolution
- Morison Model extended with a simple model for added mass

### Fixed
- Prop/Rudder interaction model : signs of prop to rudder and cog to rudder distance
- Added mass radiation model : fix the projection of the added mass when the body frame is not aligned with the world reference frame

## [3.5] - 2021-11-10

### Added
- New propulsion and steering model forces, using the external library acme
- New Sutulo manoeuvring model
### Changed
- Upgrading Geographiclib to 1.52 and patching it to disconnect useless targets such as examples, doc or js
- Using tgz archive for chrono instead of directly git repository cloning so that patch can perform correctly during fetching
- Making BUILD_SHARED_LIBS global CMake option better propagate on frydom and its dependencies
### Fixed
- FrRadiationConvolutionModel::SetImpulseResponseSize and Initialize concerning the size of the TimeRecorder
- FrAiryIrregularWaveField::GetMeanWaveDirectionAngle
- FrBody::GetHeadingFrame
- BodyDOFMask: DOF locked are according to body frame, not world frame
- missing breaks in switch statements

## [3.4] - 2021-04-08

### Added
- New recursive convolution for the radiation model, based on modal coefficients
- New catenary line model, replacing the old one : FrCatenaryLine
- New finite element line model, FrFEACable, replacing the old one : FrDynamicCable
- New field with a height variation, for current and wind models : FrHeightVaryingField
- New contact formulation : non smooth (complementarity-based) method
- Bodies can now get contact forces applied on them
- Ochi-Hubble wave spectrum is now available, with specifying either the global significant wave height, or the six parameters

### Changed
- Hydrodynamic databases read using HDB5_IO
- New HDB format version 3.0
- HDB5Tool updated to new HDB format
- Morison model extended, with added mass terms
- Logs mechanisms : de/activation, every n timesteps

### Fixed
- Excitation force computation for hdb with a forceMask
- Forward speed correction for the radiation force
- Added mass variables definition for morison elements (de-allocation problems)
- FrRadiationConvolutionModel::SetImpulseResponseSize and Initialize concerning the size of the TimeRecorder, previously recalculated, now get it from the HDB IRF. It can't be larger than the IRF one.

## [3.1] - 2020-02-06

### Changed
- minor changes in the hydrostatic equilibrium solver
- minor changes in the equilibrium frame

## [3.0] - 2020-02-04

### Major changes
Refactoring of FRyDoM entire architecture, lead by the new logging system

### Added
Hydrostatic equilibrium solver, validated on several cases.
 

### Changed
- mesh manipulation methods for the nonlinear hydrostatic model
- minor refactoring of the equilibrium frame
- reorganisation of the StepFinalize procedure
- docs updated
 

### Fixed
- FrNode stay fix in the body reference frame, when moving the body COG reference frame
- integral calculations on the clipped mesh corrected
- unnecessary data removed
- resource path missing for HexagonalArticulatedBuoy
- log FrNonLinearHydrostaticForce
- timezone target fixed
- HDB5Tool updated

## [2.1] - 2019-08-06

### Added
- New constraint type have been added
- New demos

### Changed
- New architecture for hydrodynamic interactions, validation and complex test cases
- Refactoring of Nonlinear hydrostatics and Froude-Krylov
- Refactoring of HDB5Tool python utility
- Update of the documentation with nice images (thanks Caroline Leguludec ;-) 
- Benchmark data have been moved to Amazon S3 cloud storage

### Fixed
- Unnecessary data removed
- Log FrNonlinearHydrostaticForce
- Resource path for demo_HexagonalArticulatedBuoy
- Timezone target fixed
