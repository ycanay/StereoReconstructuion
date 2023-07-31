cmake_minimum_required(VERSION 2.8)
project(Stereo_Reconstruction)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CGAL)
find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories("include")


add_executable(Main 
    source/DatasetLoader.cpp 
    source/ExtrinsicsCalculator.cpp
    source/ReconstructionManager.cpp
    source/DensePointCloudCreator.cpp
    source/NormalCalculator.cpp
    source/MeshCreator.cpp
    tests/Main.cpp)
target_link_libraries( Main ${OpenCV_LIBS} )
target_link_libraries( Main Eigen3::Eigen )
target_link_libraries( Main ${PCL_LIBRARIES})
target_link_libraries(Main CGAL::CGAL)