cmake_minimum_required( VERSION 2.8 )

#~ include_directories("${EIGEN3_INCLUDE_DIR}")
#~ include_directories("${INCLUDE_DIR}")

ADD_REQUIRED_DEPENDENCY("eigenpy")

#~ include_directories("${PROJECT_SOURCE_DIR}/include")
#~ FILE(GLOB_RECURSE HeaderFiles "${PROJECT_SOURCE_DIR}/include/hpp/ik/*.h")

# Define the wrapper library that wraps our library
add_library( hppy_ik SHARED hpp-ik-python.cc )
target_link_libraries( hppy_ik ${Boost_LIBRARIES} ${PROJECT_NAME} )
# don't prepend wrapper library name with lib
set_target_properties( hppy_ik PROPERTIES PREFIX "" )

IF(APPLE)
	# We need to change the extension for python bindings
	SET_TARGET_PROPERTIES(hppy_ik PROPERTIES SUFFIX ".so")
ENDIF(APPLE)
PKG_CONFIG_USE_DEPENDENCY(hppy_ik eigenpy)

INSTALL(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/hppy_ik.so	DESTINATION ${PYTHON_SITELIB}
)
