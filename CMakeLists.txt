# Copyright 2018, Steve Tonneau,CNRS / University of Edinburgh
#
# This file is part of hpp-ik.
# hpp-constraints is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# hpp-constraints is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-constraints. If not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

CMAKE_POLICY (SET CMP0012 NEW)
SET(CXX_DISABLE_WERROR TRUE)
SET(DOXYGEN_USE_MATHJAX YES)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/test.cmake)

SET(PROJECT_NAME hpp-ik)
SET(PROJECT_DESCRIPTION
  "High level methods over hpp-constraints to perform inverse kinematics")
SET(PROJECT_URL "https://github.com/stonneau/hpp-ik")

# Add a cache variable to allow not compiling and running tests
set (RUN_TESTS TRUE CACHE BOOL "compile and run unit tests")

SET(${PROJECT_NAME}_HEADERS
  include/hpp/ik/definitions.hh
  include/hpp/ik/hidden.hh
  include/hpp/ik/ik-constraint.hh
)

# Activate hpp-util logging if requested
SET (HPP_CONSTRAINTS_NUMERIC_DEBUG ${HPP_DEBUG} CACHE BOOL "trigger hpp-util debug output")
IF (HPP_CONSTRAINTS_NUMERIC_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_IK_NUMERIC_DEBUG")
ENDIF()


SETUP_HPP_PROJECT()

SET(Eigen_REQUIRED "eigen3 >= 3.2.4")
SEARCH_FOR_EIGEN()
ADD_REQUIRED_DEPENDENCY("hpp-pinocchio >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-statistics")
ADD_REQUIRED_DEPENDENCY("hpp-constraints")
ADD_REQUIRED_DEPENDENCY("hpp-core")

ADD_SUBDIRECTORY (src)

IF (RUN_TESTS)
  #~ ADD_REQUIRED_DEPENDENCY("romeo_description")
  SET(BOOST_COMPONENT math unit_test_framework)
  SEARCH_FOR_BOOST()
  ADD_SUBDIRECTORY(tests)
ENDIF ()

PKG_CONFIG_APPEND_LIBS("hpp-ik")

SETUP_HPP_PROJECT_FINALIZE()
