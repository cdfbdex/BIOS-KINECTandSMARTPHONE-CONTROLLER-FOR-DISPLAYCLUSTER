# Install script for directory: C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/BIOS-Full_Interaction")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/3rdparty/cvblob/cmake_install.cmake")
  include("C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/apps/InteractiveVideoWall/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

file(WRITE "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/${CMAKE_INSTALL_MANIFEST}" "")
foreach(file ${CMAKE_INSTALL_MANIFEST_FILES})
  file(APPEND "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
endforeach()