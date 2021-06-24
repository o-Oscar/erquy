cmake_minimum_required(VERSION 3.10)

set(BUILD_VERSION 1.0)

# set the project name
project(erquy VERSION ${BUILD_VERSION})

# setting up boost and python
	FIND_PACKAGE(Boost COMPONENTS python${PYTHON_VERSION_SUFFIX})
	FIND_PACKAGE(PythonInterp 3)
	FIND_PACKAGE(PythonLibs 3 REQUIRED)
	# FIND_PACKAGE(Boost COMPONENTS numpy3 REQUIRED)
	set(_Boost_NUMPY_DEPENDENCIES python${component_python_version})

	message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
	message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
	message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")
	message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")

	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
	LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

# add the core subdirectory
add_subdirectory(core)

# add the python bindings
add_subdirectory(python)



if (OFF)
# add the examples subdirectory
option(BUILD_EXAMPLES "Build the C++ examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
endif()
# --- End of the actual CMakeLists.txt... That was easy ! ---





# set (PKG_CONFIG_PATH /opt/openrobots/lib/pkgconfig/)
# find_package(pinocchio REQUIRED NO_MODULE)
# find_package(Eigen3 REQUIRED NO_MODULE) 

# add the executable
# add_executable(main "examples/first/main.cpp")

# target_include_directories(main PUBLIC /opt/openrobots/include)
# target_include_directories(main PUBLIC /opt/openrobots/lib/pkgconfig/)

# target_link_libraries(main PUBLIC core)

# target_link_libraries(main PUBLIC pinocchio::pinocchio Eigen3::Eigen)
# target_link_libraries(main PUBLIC erquy_core)

# include the .h files
#include_directories(core/include)
