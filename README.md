# StereoReconstructuion
3D Scanning and Motion Capture course TUM, Group 12.

## Needed Libraries and Software
```
sudo apt-get install cmake g++ wget unzip libeigen3-dev libgflags-dev libgoogle-glog-dev libatlas-base-dev
```
Next you need to install ceres solver.
```
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build && cd build
cmake ..
make -j4
make test
sudo make install
```
Next you need to install opencv
```
sudo apt update && sudo apt install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip
mkdir -p build && cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x
cmake --build .
sudo make install
```

If vscode can't find the include files you need to add them by yourself.
In my system I have added them as :
```
"C_Cpp.default.includePath": [
  "/usr/local/include/opencv4/**",
  "/usr/local/include/opencv4",
  "/usr/local/include/eigen3/",
  "/usr/local/include/eigen3/Eigen/src/Core",
  "/usr/local/include/ceres"
] 
```
To build the code
```
cd StereoReconstructuion
mkdir build
cd build
cmake ..
make all
```
