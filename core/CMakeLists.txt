

project(erquy_core VERSION ${BUILD_VERSION})

# wierd that I have to do it multiple times
find_package(pinocchio REQUIRED NO_MODULE)
find_package(Eigen3 REQUIRED NO_MODULE) 

# adding the src
set (SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/erquy.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/solver.cpp"
	)
add_library(erquy_core SHARED ${SRC})

# adding the include
target_include_directories(erquy_core PUBLIC include)

# wierd that I have to do it multiple times
target_link_libraries(erquy_core PUBLIC pinocchio::pinocchio Eigen3::Eigen)

