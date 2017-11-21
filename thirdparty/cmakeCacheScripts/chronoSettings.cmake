set(BUILD_DEMOS "ON" CACHE BOOL "")
set(BUILD_DEMOS_BASE "ON" CACHE BOOL "")
set(BUILD_DEMOS_FEA "ON" CACHE BOOL "")
set(BUILD_DEMOS_IRRLICHT "ON" CACHE BOOL "")
set(BUILD_DEMOS_POSTPROCESS "ON" CACHE BOOL "")
set(BUILD_TESTING "ON" CACHE BOOL "")
set(BUILD_TESTS_BASE "ON" CACHE BOOL "")
set(BUILD_TESTS_FEA "ON" CACHE BOOL "")

set(ENABLE_MODULE_CASCADE "OFF" CACHE BOOL "")
set(ENABLE_MODULE_COSIMULATION "OFF" CACHE BOOL "")
set(ENABLE_MODULE_FEA "ON" CACHE BOOL "")
set(ENABLE_MODULE_FSI "OFF" CACHE BOOL "")
set(ENABLE_MODULE_IRRLICHT "ON" CACHE BOOL "")
set(ENABLE_MODULE_MATLAB "OFF" CACHE BOOL "")
set(ENABLE_MODULE_MKL "OFF" CACHE BOOL "")
set(ENABLE_MODULE_OPENGL "OFF" CACHE BOOL "")
set(ENABLE_MODULE_PARALLEL "OFF" CACHE BOOL "")
set(ENABLE_MODULE_POSTPROCESS "ON" CACHE BOOL "")
set(ENABLE_MODULE_PYTHON "OFF" CACHE BOOL "")
set(ENABLE_MODULE_VEHICLE "OFF" CACHE BOOL "")
set(ENABLE_OPENMP "ON" CACHE BOOL "")


set(CH_COMPILER "COMPILER_GCC_X64" CACHE STRING "")
set(CMAKE_COLOR_MAKEFILE "ON" CACHE BOOL "")
set(CMAKE_CXX_FLAGS "" CACHE STRING "")
set(CMAKE_CXX_FLAGS_DEBUG "-g" CACHE STRING "")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG" CACHE STRING "")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG" CACHE STRING "")
set(CMAKE_C_FLAGS "" CACHE STRING "")
set(CMAKE_C_FLAGS_DEBUG "-g" CACHE STRING "")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG" CACHE STRING "")
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS "" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL "" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "" CACHE STRING "")
set(CMAKE_EXPORT_COMPILE_COMMANDS "OFF" CACHE BOOL "")
set(CMAKE_MODULE_LINKER_FLAGS "" CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "" CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS_MINSIZEREL "" CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "" CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO "" CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS "" CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "" CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL "" CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "" CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "" CACHE STRING "")
set(CMAKE_SKIP_INSTALL_RPATH "OFF" CACHE BOOL "")
set(CMAKE_SKIP_RPATH "OFF" CACHE BOOL "")
set(CMAKE_STATIC_LINKER_FLAGS "" CACHE STRING "")
set(CMAKE_STATIC_LINKER_FLAGS_DEBUG "" CACHE STRING "")
set(CMAKE_STATIC_LINKER_FLAGS_MINSIZEREL "" CACHE STRING "")
set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "" CACHE STRING "")
set(CMAKE_STATIC_LINKER_FLAGS_RELWITHDEBINFO "" CACHE STRING "")
set(CMAKE_VERBOSE_MAKEFILE "OFF" CACHE BOOL "")
set(COVERAGE_EXTRA_FLAGS "-l" CACHE STRING "")
set(CTEST_SUBMIT_RETRY_COUNT "3" CACHE STRING "")
set(CTEST_SUBMIT_RETRY_DELAY "5" CACHE STRING "")
set(CVSCOMMAND "CVSCOMMAND-NOTFOUND" CACHE FILEPATH "")
set(CVS_UPDATE_OPTIONS "-d -A -P" CACHE STRING "")
set(DART_TESTING_TIMEOUT "1500" CACHE STRING "")

set(MEMORYCHECK_SUPPRESSIONS_FILE "" CACHE FILEPATH "")
set(MPIEXEC_MAX_NUMPROCS "2" CACHE STRING "")
set(MPIEXEC_NUMPROC_FLAG "-np" CACHE STRING "")
set(MPIEXEC_POSTFLAGS "" CACHE STRING "")
set(MPIEXEC_PREFLAGS "" CACHE STRING "")
set(MPI_CXX_COMPILE_FLAGS "" CACHE STRING "")
set(MPI_CXX_LINK_FLAGS "" CACHE STRING "")
set(MPI_C_COMPILE_FLAGS "" CACHE STRING "")
set(MPI_C_LINK_FLAGS "" CACHE STRING "")
set(OpenMP_CXX_FLAGS "-fopenmp" CACHE STRING "")
set(OpenMP_C_FLAGS "-fopenmp" CACHE STRING "")
set(P4COMMAND "P4COMMAND-NOTFOUND" CACHE FILEPATH "")
set(SLURM_SBATCH_COMMAND "SLURM_SBATCH_COMMAND-NOTFOUND" CACHE FILEPATH "")
set(SLURM_SRUN_COMMAND "SLURM_SRUN_COMMAND-NOTFOUND" CACHE FILEPATH "")
set(USE_PARALLEL_CUDA "OFF" CACHE STRING "")
