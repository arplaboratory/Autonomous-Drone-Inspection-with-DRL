cmake_minimum_required(VERSION 3.8)

set(Darknet_MAJOR_VERSION 0)
set(Darknet_MINOR_VERSION 2)
set(Darknet_PATCH_VERSION 5)
set(Darknet_TWEAK_VERSION 1)
set(Darknet_VERSION ${Darknet_MAJOR_VERSION}.${Darknet_MINOR_VERSION}.${Darknet_PATCH_VERSION}.${Darknet_TWEAK_VERSION})

option(CMAKE_VERBOSE_MAKEFILE "Create verbose makefile" OFF)
option(CUDA_VERBOSE_BUILD "Create verbose CUDA build" OFF)
option(BUILD_SHARED_LIBS "Create dark as a shared library" ON)
option(BUILD_AS_CPP "Build Darknet using C++ compiler also for C files" OFF)
option(BUILD_USELIB_TRACK "Build uselib_track" ON)
option(MANUALLY_EXPORT_TRACK_OPTFLOW "Manually export the TRACK_OPTFLOW=1 define" OFF)
option(ENABLE_OPENCV "Enable OpenCV integration" ON)
option(ENABLE_CUDA "Enable CUDA support" ON)
option(ENABLE_CUDNN "Enable CUDNN" ON)
option(ENABLE_CUDNN_HALF "Enable CUDNN Half precision" ON)
option(ENABLE_ZED_CAMERA "Enable ZED Camera support" ON)
option(ENABLE_VCPKG_INTEGRATION "Enable VCPKG integration" ON)

if(ENABLE_VCPKG_INTEGRATION AND DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
  message(STATUS "VCPKG found: $ENV{VCPKG_ROOT}")
  message(STATUS "Using VCPKG integration")
endif()

project(darknet_ros VERSION ${Darknet_VERSION})

if(WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(USE_INTEGRATED_LIBS "TRUE" CACHE BOOL "Use libs distributed with this repo")
else()
  set(USE_INTEGRATED_LIBS "FALSE" CACHE BOOL "Use libs distributed with this repo")
endif()

enable_language(C)
enable_language(CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}" CACHE PATH "Install prefix" FORCE)
endif()

set(INSTALL_BIN_DIR      "${CMAKE_CURRENT_LIST_DIR}" CACHE PATH "Path where exe and dll will be installed")
set(INSTALL_LIB_DIR      "${CMAKE_CURRENT_LIST_DIR}" CACHE PATH "Path where lib will be installed")
set(INSTALL_INCLUDE_DIR  "include/darknet"           CACHE PATH "Path where headers will be installed")
set(INSTALL_CMAKE_DIR    "share/darknet"             CACHE PATH "Path where cmake configs will be installed")

if(${CMAKE_VERSION} VERSION_LESS "3.9.0")
  message(WARNING "To build with CUDA support you need CMake 3.9.0+")
  set(ENABLE_CUDA "FALSE" CACHE BOOL "Enable CUDA support" FORCE)
else()
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER AND ENABLE_CUDA)
    set(CUDA_ARCHITECTURES "Auto" CACHE STRING "\"Auto\" detects local machine GPU compute arch at runtime, \"Common\" and \"All\" cover common and entire subsets of architectures, \"Names\" is a list of architectures to enable by name, \"Numbers\" is a list of compute capabilities (version number) to enable")
    set_property(CACHE CUDA_ARCHITECTURES PROPERTY STRINGS "Auto" "Common" "All" "Kepler Maxwell Kepler+Tegra Maxwell+Tegra Pascal" "3.0 7.5")
    enable_language(CUDA)
    find_package(CUDA REQUIRED)
    if(CUDA_VERSION VERSION_LESS "9.0")
      message(STATUS "Unsupported CUDA version, please upgrade to CUDA 9+. Disabling CUDA support")
      set(ENABLE_CUDA "FALSE" CACHE BOOL "Enable CUDA support" FORCE)
    else()
      cuda_select_nvcc_arch_flags(CUDA_ARCH_FLAGS ${CUDA_ARCHITECTURES})
      message(STATUS "Building with CUDA flags: " "${CUDA_ARCH_FLAGS}")
      if (NOT "arch=compute_75,code=sm_75" IN_LIST CUDA_ARCH_FLAGS)
        set(ENABLE_CUDNN_HALF "FALSE" CACHE BOOL "Enable CUDNN Half precision" FORCE)
        message(STATUS "Your setup does not supports half precision (it requires CC >= 7.5)")
      endif()
    endif()
  else()
    set(ENABLE_CUDA "FALSE" CACHE BOOL "Enable CUDA support" FORCE)
  endif()
endif()

if (WIN32 AND ENABLE_CUDA AND CMAKE_MAKE_PROGRAM MATCHES "ninja")
  option(SELECT_OPENCV_MODULES "Use only few selected OpenCV modules to circumvent 8192 char limit when using Ninja on Windows" ON)
else()
  option(SELECT_OPENCV_MODULES "Use only few selected OpenCV modules to circumvent 8192 char limit when using Ninja on Windows" OFF)
endif()

if(USE_INTEGRATED_LIBS)
  set(PThreads_windows_DIR ${CMAKE_CURRENT_LIST_DIR}/3rdparty/pthreads CACHE PATH "Path where pthreads for windows can be located")
endif()
set(Stb_DIR ${CMAKE_CURRENT_LIST_DIR}/3rdparty/stb CACHE PATH "Path where Stb image library can be located")

set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_THREAD_PREFER_PTHREAD ON)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    actionlib
    image_transport
    darknet_ros_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    actionlib
    rospy
    std_msgs
    image_transport
    darknet_ros_msgs
  DEPENDS
    Boost
)

include_directories(
  src
  include
  3rdparty/stb/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Find X11
message ( STATUS "Searching for X11..." )
find_package ( X11 REQUIRED )
if ( X11_FOUND )
  include_directories ( ${X11_INCLUDE_DIR} )
  link_libraries ( ${X11_LIBRARIES} )
  message ( STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR} )
  message ( STATUS " X11_LIBRARIES: " ${X11_LIBRARIES} )
endif ( X11_FOUND )

find_package(Threads REQUIRED)
if(MSVC)
  find_package(PThreads_windows REQUIRED)
endif()
if(ENABLE_OPENCV)
  find_package(OpenCV 3)
  if(OpenCV_FOUND)
    if(SELECT_OPENCV_MODULES)
      if(TARGET opencv_world)
        list(APPEND OpenCV_LINKED_COMPONENTS "opencv_world")
      else()
        if(TARGET opencv_core)
          list(APPEND OpenCV_LINKED_COMPONENTS "opencv_core")
        endif()
        if(TARGET opencv_highgui)
          list(APPEND OpenCV_LINKED_COMPONENTS "opencv_highgui")
        endif()
        if(TARGET opencv_imgproc)
          list(APPEND OpenCV_LINKED_COMPONENTS "opencv_imgproc")
        endif()
        if(TARGET opencv_video)
          list(APPEND OpenCV_LINKED_COMPONENTS "opencv_video")
        endif()
        if(TARGET opencv_videoio)
          list(APPEND OpenCV_LINKED_COMPONENTS "opencv_videoio")
        endif()
        if(TARGET opencv_imgcodecs)
          list(APPEND OpenCV_LINKED_COMPONENTS "opencv_imgcodecs")
        endif()
        if(TARGET opencv_text)
          list(APPEND OpenCV_LINKED_COMPONENTS "opencv_text")
        endif()
      endif()
    else()
      list(APPEND OpenCV_LINKED_COMPONENTS ${OpenCV_LIBS})
    endif()
  endif()
endif()
find_package(Stb REQUIRED)
if(${CMAKE_VERSION} VERSION_LESS "3.11.0")
  message(WARNING "To build with OpenMP support you need CMake 3.11.0+")
else()
  find_package(OpenMP)
endif()

set(ADDITIONAL_CXX_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wfatal-errors -Wno-deprecated-declarations -Wno-write-strings")
set(ADDITIONAL_C_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wfatal-errors -Wno-deprecated-declarations -Wno-write-strings")

if(MSVC)
  set(ADDITIONAL_CXX_FLAGS "/wd4013 /wd4018 /wd4028 /wd4047 /wd4068 /wd4090 /wd4101 /wd4113 /wd4133 /wd4190 /wd4244 /wd4267 /wd4305 /wd4477 /wd4996 /wd4819 /fp:fast")
  set(ADDITIONAL_C_FLAGS "/wd4013 /wd4018 /wd4028 /wd4047 /wd4068 /wd4090 /wd4101 /wd4113 /wd4133 /wd4190 /wd4244 /wd4267 /wd4305 /wd4477 /wd4996 /wd4819 /fp:fast")
  set(CMAKE_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${ADDITIONAL_C_FLAGS} ${CMAKE_C_FLAGS}")
  string(REGEX REPLACE "/O2" "/Ox" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
  string(REGEX REPLACE "/O2" "/Ox" CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})
endif()

if(CMAKE_COMPILER_IS_GNUCC OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    if (UNIX AND NOT APPLE)
      set(CMAKE_CXX_FLAGS "-pthread ${CMAKE_CXX_FLAGS}")  #force pthread to avoid bugs in some cmake setups
      set(CMAKE_C_FLAGS "-pthread ${CMAKE_C_FLAGS}")
    endif()
  endif()
  set(CMAKE_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${ADDITIONAL_C_FLAGS} ${CMAKE_C_FLAGS}")
  string(REGEX REPLACE "-O0" "-Og" CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})
  string(REGEX REPLACE "-O3" "-Ofast" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
  string(REGEX REPLACE "-O0" "-Og" CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
  string(REGEX REPLACE "-O3" "-Ofast" CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffp-contract=fast -mavx -mavx2 -msse3 -msse4.1 -msse4.2 -msse4a")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffp-contract=fast -mavx -mavx2 -msse3 -msse4.1 -msse4.2 -msse4a")
endif()

if(OpenCV_FOUND)
  if(ENABLE_CUDA AND NOT OpenCV_CUDA_VERSION)
    set(BUILD_USELIB_TRACK "FALSE" CACHE BOOL "Build uselib_track" FORCE)
    message(STATUS "  ->  darknet is fine for now, but uselib_track has been disabled!")
    message(STATUS "  ->  Please rebuild OpenCV from sources with CUDA support to enable it")
  elseif(ENABLE_CUDA AND OpenCV_CUDA_VERSION)
    if(TARGET opencv_cudaoptflow)
      list(APPEND OpenCV_LINKED_COMPONENTS "opencv_cudaoptflow")
    endif()
    if(TARGET opencv_cudaimgproc)
      list(APPEND OpenCV_LINKED_COMPONENTS "opencv_cudaimgproc")
    endif()
  endif()
endif()

if(ENABLE_CUDA)
  find_package(CUDNN)
  if(NOT CUDNN_FOUND)
    set(ENABLE_CUDNN "FALSE" CACHE BOOL "Enable CUDNN" FORCE)
  endif()
endif()

if(ENABLE_CUDA)
  if (MSVC)
    set(ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} /DGPU")
    if(CUDNN_FOUND)
      set(ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} /DCUDNN")
    endif()
    if(OpenCV_FOUND)
      set(ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} /DOPENCV")
    endif()
    string(REPLACE " " "," ADDITIONAL_CXX_FLAGS_COMMA_SEPARATED "${ADDITIONAL_CXX_FLAGS}")
    set(CUDA_HOST_COMPILER_FLAGS "-Wno-deprecated-declarations -Xcompiler=\"${ADDITIONAL_CXX_FLAGS_COMMA_SEPARATED}\"")
  else()
    set(ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} -DGPU")
    if(CUDNN_FOUND)
      set(ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} -DCUDNN")
    endif()
    if(OpenCV_FOUND)
      set(ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} -DOPENCV")
    endif()
    set(CUDA_HOST_COMPILER_FLAGS "--compiler-options \" ${ADDITIONAL_CXX_FLAGS} -fPIC -fopenmp -Ofast \"")
  endif()

  string (REPLACE ";" " " CUDA_ARCH_FLAGS_SPACE_SEPARATED "${CUDA_ARCH_FLAGS}")
  set(CMAKE_CUDA_FLAGS "${CUDA_ARCH_FLAGS_SPACE_SEPARATED} ${CUDA_HOST_COMPILER_FLAGS} ${CMAKE_CUDA_FLAGS}")
  message(STATUS "CMAKE_CUDA_FLAGS: ${CMAKE_CUDA_FLAGS}")
endif()

if(ENABLE_CUDA)
  if(ENABLE_ZED_CAMERA)
    find_package(ZED 2 QUIET)
    if(ZED_FOUND)
      include_directories(${ZED_INCLUDE_DIRS})
      link_directories(${ZED_LIBRARY_DIR})
      message(STATUS "ZED SDK enabled")
    else()
      message(STATUS "ZED SDK not found")
      set(ENABLE_ZED_CAMERA "FALSE" CACHE BOOL "Enable ZED Camera support" FORCE)
    endif()
  endif()
else()
  message(STATUS "ZED SDK not enabled, since it requires CUDA")
  set(ENABLE_ZED_CAMERA "FALSE" CACHE BOOL "Enable ZED Camera support" FORCE)
endif()

set(DARKNET_INSTALL_INCLUDE_DIR ${INSTALL_INCLUDE_DIR})
# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/src/version.h.in"
  "${CMAKE_CURRENT_LIST_DIR}/src/version.h"
)

#look for all *.h files in src folder
file(GLOB headers "${CMAKE_CURRENT_LIST_DIR}/src/*.h")
#add also files in the include folder
list(APPEND headers
  ${CMAKE_CURRENT_LIST_DIR}/include/darknet.h
  ${CMAKE_CURRENT_LIST_DIR}/include/yolo_detector.h
)
#remove windows only files
if(NOT WIN32)
  list(REMOVE_ITEM headers
    ${CMAKE_CURRENT_LIST_DIR}/src/gettimeofday.h
    ${CMAKE_CURRENT_LIST_DIR}/src/getopt.h
  )
endif()
#set(exported_headers ${headers})

#look for all *.c files in src folder
file(GLOB sources "${CMAKE_CURRENT_LIST_DIR}/src/*.c")
#add also .cpp files
list(APPEND sources
  ${CMAKE_CURRENT_LIST_DIR}/src/http_stream.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/image_opencv.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/yolo_detector_node.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/yolo_detector.cpp
)
#remove darknet.c file which is necessary only for the executable, not for the lib
list(REMOVE_ITEM sources
  ${CMAKE_CURRENT_LIST_DIR}/src/darknet.c
)
#remove windows only files
if(NOT WIN32)
  list(REMOVE_ITEM sources
    ${CMAKE_CURRENT_LIST_DIR}/src/gettimeofday.c
    ${CMAKE_CURRENT_LIST_DIR}/src/getopt.c
  )
endif()

if(ENABLE_CUDA)
  file(GLOB cuda_sources "${CMAKE_CURRENT_LIST_DIR}/src/*.cu")
endif()

if(BUILD_AS_CPP)
  set_source_files_properties(${sources} PROPERTIES LANGUAGE CXX)
endif()

add_library(dark ${CMAKE_CURRENT_LIST_DIR}/include/yolo_detector.h ${CMAKE_CURRENT_LIST_DIR}/include/yolo_v2_class.hpp ${CMAKE_CURRENT_LIST_DIR}/src/yolo_v2_class.cpp ${sources} ${headers} ${cuda_sources})
set_target_properties(dark PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(ENABLE_CUDA)
  set_target_properties(dark PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()
if(BUILD_SHARED_LIBS)
  target_compile_definitions(dark PRIVATE LIB_EXPORTS=1)
endif()
if(BUILD_AS_CPP)
  set_target_properties(dark PROPERTIES LINKER_LANGUAGE CXX)
endif()

if(OpenCV_FOUND AND OpenCV_VERSION VERSION_GREATER "3.0" AND BUILD_USELIB_TRACK)
  add_executable(uselib_track ${CMAKE_CURRENT_LIST_DIR}/src/yolo_console_dll.cpp)
endif()

add_executable(uselib ${CMAKE_CURRENT_LIST_DIR}/src/yolo_console_dll.cpp)
if(BUILD_AS_CPP)
  set_target_properties(uselib PROPERTIES LINKER_LANGUAGE CXX)
endif()

add_executable(darknet_ros ${CMAKE_CURRENT_LIST_DIR}/src/yolo_detector_node.cpp ${sources} ${headers} ${cuda_sources})
if(BUILD_AS_CPP)
  set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/src/yolo_detector_node.cpp PROPERTIES LANGUAGE CXX)
  set_target_properties(darknet_ros PROPERTIES LINKER_LANGUAGE CXX)
endif()

target_include_directories(darknet_ros PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src> $<INSTALL_INTERFACE:${DARKNET_INSTALL_INCLUDE_DIR}> $<BUILD_INTERFACE:${Stb_INCLUDE_DIR}>)
target_include_directories(dark PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src> $<INSTALL_INTERFACE:${DARKNET_INSTALL_INCLUDE_DIR}> $<BUILD_INTERFACE:${Stb_INCLUDE_DIR}>)
target_include_directories(uselib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src> $<INSTALL_INTERFACE:${DARKNET_INSTALL_INCLUDE_DIR}> $<BUILD_INTERFACE:${Stb_INCLUDE_DIR}>)

target_compile_definitions(darknet_ros PRIVATE -DUSE_CMAKE_LIBS)
target_compile_definitions(dark PRIVATE -DUSE_CMAKE_LIBS)
target_compile_definitions(uselib PRIVATE -DUSE_CMAKE_LIBS)

if(OpenCV_FOUND AND OpenCV_VERSION VERSION_GREATER "3.0" AND BUILD_USELIB_TRACK AND NOT MANUALLY_EXPORT_TRACK_OPTFLOW)
  target_compile_definitions(dark PUBLIC TRACK_OPTFLOW=1)
endif()


target_link_libraries(darknet_ros PRIVATE ${catkin_LIBRARIES})
target_link_libraries(dark PRIVATE ${catkin_LIBRARIES})

if(CUDNN_FOUND)
  target_link_libraries(darknet_ros PRIVATE CuDNN::CuDNN)
  target_link_libraries(dark PRIVATE CuDNN::CuDNN)
  target_compile_definitions(darknet_ros PRIVATE -DCUDNN)
  target_compile_definitions(dark PUBLIC -DCUDNN)
  if(ENABLE_CUDNN_HALF)
    target_compile_definitions(darknet_ros PRIVATE -DCUDNN_HALF)
    target_compile_definitions(dark PUBLIC -DCUDNN_HALF)
  endif()
endif()

if(OpenCV_FOUND)
  target_link_libraries(darknet_ros PRIVATE ${OpenCV_LINKED_COMPONENTS})
  target_link_libraries(uselib PRIVATE ${OpenCV_LINKED_COMPONENTS})
  target_link_libraries(dark PUBLIC ${OpenCV_LINKED_COMPONENTS})
  target_include_directories(dark PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_compile_definitions(darknet_ros PRIVATE -DOPENCV)
  target_compile_definitions(dark PUBLIC -DOPENCV)
endif()

if(OPENMP_FOUND)
  target_link_libraries(darknet_ros PRIVATE OpenMP::OpenMP_CXX)
  target_link_libraries(darknet_ros PRIVATE OpenMP::OpenMP_C)
  target_link_libraries(dark PUBLIC OpenMP::OpenMP_CXX)
  target_link_libraries(dark PUBLIC OpenMP::OpenMP_C)
endif()

if(CMAKE_COMPILER_IS_GNUCC)
  target_link_libraries(darknet_ros PRIVATE m)
  target_link_libraries(dark PUBLIC m)
endif()

if(MSVC)
  target_link_libraries(darknet_ros PRIVATE PThreads_windows::PThreads_windows)
  target_link_libraries(darknet_ros PRIVATE wsock32 ws2_32)
  target_link_libraries(dark PUBLIC PThreads_windows::PThreads_windows)
  target_link_libraries(dark PUBLIC wsock32 ws2_32)
  target_link_libraries(uselib PRIVATE PThreads_windows::PThreads_windows)
  target_compile_definitions(darknet_ros PRIVATE -D_CRT_RAND_S -DNOMINMAX -D_USE_MATH_DEFINES)
  target_compile_definitions(dark PRIVATE -D_CRT_RAND_S -DNOMINMAX -D_USE_MATH_DEFINES)
  target_compile_definitions(dark PUBLIC -D_CRT_SECURE_NO_WARNINGS)
  target_compile_definitions(uselib PRIVATE -D_CRT_RAND_S -DNOMINMAX -D_USE_MATH_DEFINES)
endif()

target_link_libraries(darknet_ros PRIVATE Threads::Threads)
target_link_libraries(dark PUBLIC Threads::Threads)
target_link_libraries(uselib PRIVATE Threads::Threads)

if(ENABLE_ZED_CAMERA)
  target_link_libraries(darknet_ros PRIVATE ${ZED_LIBRARIES})
  target_link_libraries(dark PUBLIC ${ZED_LIBRARIES})
  target_link_libraries(uselib PRIVATE ${ZED_LIBRARIES})
  target_compile_definitions(darknet_ros PRIVATE -DZED_STEREO)
  target_compile_definitions(uselib PRIVATE -DZED_STEREO)
  target_compile_definitions(dark PUBLIC -DZED_STEREO)
endif()

if(ENABLE_CUDA)
  target_include_directories(darknet_ros PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
  target_include_directories(dark PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
  target_link_libraries(darknet_ros PRIVATE curand cublas cuda)
  target_link_libraries(dark PRIVATE curand cublas cuda)
  set_target_properties(dark PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS ON)
  target_compile_definitions(darknet_ros PRIVATE -DGPU)
  target_compile_definitions(dark PUBLIC -DGPU)
endif()

if(USE_INTEGRATED_LIBS)
  target_compile_definitions(darknet PRIVATE -D_TIMESPEC_DEFINED)
  target_compile_definitions(dark PRIVATE -D_TIMESPEC_DEFINED)
endif()

target_link_libraries(uselib PRIVATE dark)
if(OpenCV_FOUND AND OpenCV_VERSION VERSION_GREATER "3.0" AND BUILD_USELIB_TRACK)
  target_link_libraries(uselib_track PRIVATE dark)
  target_compile_definitions(uselib_track PRIVATE TRACK_OPTFLOW=1)
  target_compile_definitions(uselib_track PRIVATE -DUSE_CMAKE_LIBS)
  if(BUILD_AS_CPP)
    set_target_properties(uselib_track PROPERTIES LINKER_LANGUAGE CXX)
  endif()
  target_include_directories(uselib_track PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
  target_link_libraries(uselib_track PRIVATE ${OpenCV_LINKED_COMPONENTS})
  if(ENABLE_ZED_CAMERA)
    target_link_libraries(uselib_track PRIVATE ${ZED_LIBRARIES})
    target_compile_definitions(uselib_track PRIVATE -DZED_STEREO)
  endif()
  if(MSVC)
    target_link_libraries(uselib_track PRIVATE PThreads_windows::PThreads_windows)
    target_compile_definitions(uselib_track PRIVATE -D_CRT_RAND_S -DNOMINMAX -D_USE_MATH_DEFINES)
  endif()
  target_link_libraries(uselib_track PRIVATE Threads::Threads)
endif()

#set_target_properties(dark PROPERTIES PUBLIC_HEADER "${exported_headers};${CMAKE_CURRENT_LIST_DIR}/include/yolo_v2_class.hpp")
set_target_properties(dark PROPERTIES PUBLIC_HEADER "${CMAKE_CURRENT_LIST_DIR}/include/darknet.h;${CMAKE_CURRENT_LIST_DIR}/include/yolo_detector.h;${CMAKE_CURRENT_LIST_DIR}/include/yolo_v2_class.hpp")

set_target_properties(dark PROPERTIES CXX_VISIBILITY_PRESET hidden)

set_target_properties(darknet_ros PROPERTIES PUBLIC_HEADER "${CMAKE_CURRENT_LIST_DIR}/include/darknet.h;${CMAKE_CURRENT_LIST_DIR}/include/yolo_detector.h;${CMAKE_CURRENT_LIST_DIR}/include/yolo_v2_class.hpp")


add_dependencies(dark
  darknet_ros_msgs_generate_messages_cpp
)

add_dependencies(darknet_ros
  darknet_ros_msgs_generate_messages_cpp
)

install(TARGETS dark
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(TARGETS darknet_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS darknet_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


install(TARGETS dark EXPORT DarknetTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
  COMPONENT dev
)
install(TARGETS uselib darknet_ros
  DESTINATION "${INSTALL_BIN_DIR}"
)
if(OpenCV_FOUND AND OpenCV_VERSION VERSION_GREATER "3.0" AND BUILD_USELIB_TRACK)
  install(TARGETS uselib_track
    DESTINATION "${INSTALL_BIN_DIR}"
  )
endif()

install(EXPORT DarknetTargets
  FILE DarknetTargets.cmake
  NAMESPACE Darknet::
  DESTINATION "${INSTALL_CMAKE_DIR}"
)

# Export the package for use from the build-tree (this registers the build-tree with a global CMake-registry)
export(PACKAGE Darknet)

# Create the DarknetConfig.cmake
# First of all we compute the relative path between the cmake config file and the include path
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(DarknetConfig.cmake.in "${PROJECT_BINARY_DIR}/DarknetConfig.cmake" @ONLY)
set(CONF_INCLUDE_DIRS "\${Darknet_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(DarknetConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/DarknetConfig.cmake" @ONLY)

# Create the DarknetConfigVersion.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_BINARY_DIR}/DarknetConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/DarknetConfig.cmake"
  "${PROJECT_BINARY_DIR}/DarknetConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}"
)

set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
set(FILE "${PATH}/yolov3.weights")
message(STATUS "Checking and downloading yolov3.weights if needed ...")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget http://pjreddie.com/media/files/yolov3.weights -P ${PATH})
endif()
