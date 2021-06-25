

project(erquy_examples VERSION ${BUILD_VERSION})

# wierd that I have to do it multiple times
find_package(pinocchio REQUIRED NO_MODULE)
find_package(Eigen3 REQUIRED NO_MODULE) 


# add the executables
# add_executable(first "first/main.cpp")


# wierd that I have to do it multiple times
target_link_libraries(first PUBLIC pinocchio::pinocchio Eigen3::Eigen)
target_link_libraries(first PUBLIC erquy_core)
