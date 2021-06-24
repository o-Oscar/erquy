PYTHON_ADD_MODULE(erquy_py binding.cpp)
FILE(COPY usage.py DESTINATION .)

# wierd that I have to do it multiple times
find_package(pinocchio REQUIRED NO_MODULE)
find_package(Eigen3 REQUIRED NO_MODULE) 

# wierd that I have to do it multiple times
target_link_libraries(erquy_py PUBLIC pinocchio::pinocchio Eigen3::Eigen)

# linking to the core
target_link_libraries(erquy_py PUBLIC erquy_core)