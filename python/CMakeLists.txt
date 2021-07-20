PYTHON_ADD_MODULE(erquy_py binding.cpp)

FILE(COPY visualizer.py DESTINATION .)
FILE(COPY usage.py DESTINATION .)
FILE(COPY test.py DESTINATION .)

# wierd that I have to do it multiple times
find_package(pinocchio REQUIRED NO_MODULE)
find_package(Eigen3 REQUIRED NO_MODULE) 
find_package(eigenpy 2.0.2 REQUIRED NO_MODULE NO_CMAKE_SYSTEM_PATH)

# wierd that I have to do it multiple times
target_link_libraries(erquy_py PUBLIC pinocchio::pinocchio Eigen3::Eigen)

# linking to the core
target_link_libraries(erquy_py PUBLIC erquy_core)

# linking to the eigenpy
target_link_libraries(erquy_py PUBLIC eigenpy::eigenpy)