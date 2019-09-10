cmake_minimum_required(VERSION 2.8.3)
PROJECT(K4a_grabber)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  add_compile_options(/std:c++latest)
else()
  add_compile_options(-std=c++11)
endif()

macro(process_ext_sdk)
  message(STATUS "K4A SDK found in ext/sdk!")
  message(STATUS "!! Not searching for SDK in system path !!")

  # mark that the K4A SDK will need to be copied to the output binary folder
  # this is needed even on Linux if the SDK is not installed to the system path
  set(K4A_INSTALL_NEEDED true)

  get_target_property(K4A_INCLUDE_DIRS k4a::k4a INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(K4A_CONFIGS k4a::k4a IMPORTED_CONFIGURATIONS)
  message("Configs: ${K4A_CONFIGS}")

  # TODO: if we find more than one configuration, we should fail
  # TODO: potentially clean this logic up
  foreach(imported_config ${K4A_CONFIGS})
    if (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
      get_target_property(K4A_CONFIG_LIBS k4a::k4a IMPORTED_IMPLIB_${imported_config})
      get_target_property(K4A_CONFIG_DLLS k4a::k4a IMPORTED_LOCATION_${imported_config})
      list(APPEND K4A_LIBS ${K4A_CONFIG_LIBS})
      list(APPEND K4A_DLL_FILES ${K4A_CONFIG_DLLS})
    elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
      get_target_property(K4A_CONFIG_LIBS k4a::k4a IMPORTED_LOCATION_${imported_config})
      get_target_property(K4A_CONFIG_DLLS k4a::k4a IMPORTED_LOCATION_${imported_config})
      list(APPEND K4A_LIBS ${K4A_CONFIG_LIBS})
      list(APPEND K4A_DLL_FILES ${K4A_CONFIG_DLLS})
    endif()
  endforeach()

  if(DEFINED DEPTHENGINE_DLL)
    ## Find the depth engine DLL
    file(GLOB_RECURSE DEPTHENGINE_DLL_FILE
      "${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk/*/${DEPTHENGINE_DLL}" )
    list(APPEND K4A_DLL_FILES ${DEPTHENGINE_DLL_FILE})
  endif()
endmacro()

# Define the names of some files that we are going to try and find
if (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  set(DEPTHENGINE_DLL "depthengine_1_0.dll")
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(DEPTHENGINE_DLL "libdepthengine.so.1.0")
else()
  message(FATAL_ERROR "Platform ${CMAKE_SYSTEM_NAME} is not supported")
endif()

message("Finding K4A SDK binaries")

unset(k4a_DIR CACHE)
unset(azure-kinect-sensor-sdk_DIR CACHE)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  message("Linux mode: Searching for K4A in system path and ./ext/sdk")

  # Test if we can find the k4a library in the system path
  find_package(k4a 1.1.1 QUIET)

  if (${k4a_FOUND})
    message("K4A SDK found in system libraries!")
    # For system libraries on linux, we can just append the target name (k4a::k4a)
    # to what will be passed into target_link_libraries()
    list(APPEND K4A_LIBS k4a::k4a)
  else()
    message("K4A SDK not found in system libraries. Searching ./ext/sdk...")
    find_package(k4a 1.1.1 PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk")

    if(${k4a_FOUND})
      set(K4A_INSTALL_NEEDED true)
      message("K4A SDK found in ./ext/sdk!")
      process_ext_sdk()
    else()
      message(FATAL_ERROR "K4A SDK not found in system libraries or ./ext/sdk. Please install the Azure Kinect SDK.")
    endif()

  endif()

elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  # Windows will always need K4A install for all targets
  set(K4A_INSTALL_NEEDED true)

  # Try to find K4A installed to Program Files
  find_package(azure-kinect-sensor-sdk 1.1.1 EXACT QUIET PATHS "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  if (${azure-kinect-sensor-sdk_FOUND})
    message("K4A SDK found in Program Files!")

    list(APPEND K4A_LIBS ${azure-kinect-sensor-sdk_LIBRARIES})
    list(APPEND K4A_BINS ${azure-kinect-sensor-sdk_BINARIES})
    set(K4A_INCLUDE_DIRS ${azure-kinect-sensor-sdk_INCLUDE_DIRS})

    # Create a list of all K4A DLLs
    list(APPEND K4A_DLLS ${azure-kinect-sensor-sdk_LIBRARIES})
    list(APPEND K4A_DLLS ${azure-kinect-sensor-sdk_BINARIES})

    foreach(DLL ${K4A_DLLS})
      set(prop "PROPERTY-NOTFOUND")
      get_property(prop TARGET ${DLL} PROPERTY IMPORTED_LOCATION)
      message(STATUS "IMPORTED_LOCATION: ${prop}")

      if((NOT prop) OR (prop EQUAL "PROPERTY-NOTFOUND"))
        message(FATAL_ERROR "Target '${DLL}' in package azure-kinect-sensor-sdk does not contain an IMPORTED_LOCATION property")
      endif()

      list(APPEND K4A_DLL_FILES ${prop})
    endforeach(DLL)
  else()
    # Try to find K4A installed to ./ext/sdk
    find_package(k4a 1.1.1 QUIET PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk")

    if (${k4a_FOUND})
      message("K4A SDK found in ./ext/sdk!")
      process_ext_sdk()
    else()
      message(FATAL_ERROR "K4A SDK not found in Program Files or ./ext/sdk. Please install the Azure Kinect SDK.")
    endif()
  endif()  
endif()

message("K4A Include Dirs: ${K4A_INCLUDE_DIRS}")
message("K4A Libs: ${K4A_LIBS}")
message("K4A DLLs: ${K4A_DLL_FILES}")
message("K4A Install Needed: ${K4A_INSTALL_NEEDED}")

if (${K4A_INSTALL_NEEDED})
  # Tell cmake that we need to reconfigure if any of the DLL files change
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${K4A_DLL_FILES})

  # We need to copy the DLLs into the CATKIN_PACKAGE_LIB_DESTINATION so
  # the node executable can find them on launch, and CATKIN_PACKAGE_BIN_DESTINATION
  # so the nodelet can find them on launch
  set(DLL_COPY_DIRECTORY "${PROJECT_BINARY_DIR}/bin/Release")

  foreach(DIRECTORY ${DLL_COPY_DIRECTORY})
    file(MAKE_DIRECTORY "${DIRECTORY}")
  endforeach(DIRECTORY)

  foreach(DLL ${K4A_DLL_FILES})
    foreach(DIRECTORY ${DLL_COPY_DIRECTORY})
      file(COPY "${DLL}" DESTINATION "${DIRECTORY}")
      get_filename_component(DLL_NAME ${DLL} NAME)
      message(STATUS "Copied dll from ${DLL_NAME} to ${DIRECTORY}")
      # Tell cmake that we need to clean up these DLLs on a "make clean"
      set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES "${DIRECTORY}/${DLL_NAME}")
    endforeach(DIRECTORY)
  endforeach(DLL)
  
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "/Z7")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO /DEBUG /OPT:REF /OPT:ICF")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4100 /wd4127 /wd4150 /wd4191 /wd4242 /wd4244 /wd4251 /wd4305 /wd4365 /wd4512 /wd4514 /wd4548 /wd4571 /wd4619 /wd4625 /wd4626 /wd4628 /wd4668 /wd4710 /wd4711 /wd4738 /wd4820 /wd4946 /wd4986 /wd4819 /wd4996 /wd4267")

endif()

find_package(PCL 1.9.1 REQUIRED COMPONENTS io visualization)

# Find GLM. You have to set the glm_room path with your environment
set(GLM_DIR "D:/glm-0.9.7.1/my/install")
set(GLM_ROOT "D:/glm-0.9.7.1/my/install")
IF(NOT GLM_ROOT)
    MESSAGE("ERROR: GLM_ROOT must be set!")
ENDIF(NOT GLM_ROOT)

FIND_PATH(GLM_INCLUDE_DIRS DOC "Path to GLM include directory." NAMES glm/glm.hpp PATHS ${GLM_ROOT}/include)

add_subdirectory(PCL_Grabber_OneKinect)
add_subdirectory(PCL_Grabber_TwoKinects)