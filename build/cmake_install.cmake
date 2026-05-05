# Install script for directory: F:/代码/ku/AstraAlgebra

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/AstraAlgebra")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "D:/msys64/ucrt64/bin/objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/代码/ku/AstraAlgebra/build/libAstraAlgebra.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "F:/代码/ku/AstraAlgebra/include/AstraAlgebra" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/AstraAlgebra/AstraAlgebraTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/AstraAlgebra/AstraAlgebraTargets.cmake"
         "F:/代码/ku/AstraAlgebra/build/CMakeFiles/Export/c380d3045a68b49e489664a931cc2386/AstraAlgebraTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/AstraAlgebra/AstraAlgebraTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/AstraAlgebra/AstraAlgebraTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/AstraAlgebra" TYPE FILE FILES "F:/代码/ku/AstraAlgebra/build/CMakeFiles/Export/c380d3045a68b49e489664a931cc2386/AstraAlgebraTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/AstraAlgebra" TYPE FILE FILES "F:/代码/ku/AstraAlgebra/build/CMakeFiles/Export/c380d3045a68b49e489664a931cc2386/AstraAlgebraTargets-debug.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/AstraAlgebra" TYPE FILE FILES
    "F:/代码/ku/AstraAlgebra/build/AstraAlgebraConfig.cmake"
    "F:/代码/ku/AstraAlgebra/build/AstraAlgebraConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/代码/ku/AstraAlgebra/build/AstraAlgebra.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/AstraAlgebra" TYPE FILE FILES
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/AstraAlgebra.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Core/MathUtils.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Core/GPU.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Core/Serialization.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Core/Transform.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Vector/Vector2.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Vector/Vector3.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Vector/Vector4.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Vector/Vector3d.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Matrix/Matrix2x2.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Matrix/Matrix3x3.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Matrix/Matrix4x4.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Matrix/Matrix4x4d.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Quaternion/Quaternion.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Ray.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Plane.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Sphere.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Bounds.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Triangle.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Noise.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Cylinder.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Cone.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Capsule.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Geometry/Line.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Numerics/MatrixDecomposition.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Numerics/ColorSpace.h"
    "F:/代码/ku/AstraAlgebra/include/AstraAlgebra/Numerics/Random.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "F:/代码/ku/AstraAlgebra/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
