cmake_minimum_required(VERSION 2.8)
project(Stereo_Reconstruction)


find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories("include")

add_library(DataSetLoader source/DatasetLoader.cpp )
target_link_libraries( DataSetLoader ${OpenCV_LIBS} )
target_link_libraries( DataSetLoader Eigen3::Eigen )

add_executable(DataSetLoaderTest source/DatasetLoader.cpp tests/DataSetLoaderTest.cpp )
target_link_libraries( DataSetLoaderTest ${OpenCV_LIBS} )
target_link_libraries( DataSetLoaderTest Eigen3::Eigen )

add_library(ExtrinsicsCalculator source/ExtrinsicsCalculator.cpp )
target_link_libraries( ExtrinsicsCalculator ${OpenCV_LIBS} )
target_link_libraries( ExtrinsicsCalculator Eigen3::Eigen )

add_executable(ExtrinsicsCalculatorTest source/DatasetLoader.cpp tests/ExtrinsicsCalculatorTest.cpp source/ExtrinsicsCalculator.cpp )
target_link_libraries( ExtrinsicsCalculatorTest ${OpenCV_LIBS} )
target_link_libraries( ExtrinsicsCalculatorTest Eigen3::Eigen )