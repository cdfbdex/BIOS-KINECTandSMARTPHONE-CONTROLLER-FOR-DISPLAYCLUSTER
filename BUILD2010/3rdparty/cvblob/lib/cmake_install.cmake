# Install script for directory: C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/3rdparty/cvblob/cvBlob

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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/3rdparty/cvblob/cvBlob/cvblob.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/lib/Debug/cvblob.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/lib/Release/cvblob.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/lib/MinSizeRel/cvblob.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/lib/RelWithDebInfo/cvblob.lib")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/lib/Debug/cvblob.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/lib/Release/cvblob.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/lib/MinSizeRel/cvblob.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/LUISVELEZ/Documentos/APLICACIONES_MURO/TUIO+Kinect VRPN Controller/KinectTUIO-Full_Interaction/BUILD2010/lib/RelWithDebInfo/cvblob.dll")
  endif()
endif()

