
# StereoReconstructuion
3D Scanning and Motion Capture course TUM, Group 12.

## Initial install
Firstly, decide on a single folder in which you want to do the install (e.g. $HOME). Every separate highlighted block should start in this $HOME folder. 
```bash
sudo apt-get install cmake g++ wget unzip libeigen3-dev libgflags-dev libgoogle-glog-dev libatlas-base-dev
```
Next you need to install ceres solver.
```bash
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build && cd build
cmake ..
make -j4
make test
sudo make install
```

Next you need to install opencv.

```bash
mkdir opencv && cd opencv
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
When finished it should look like this:

![image](https://github.com/ycanay/StereoReconstructuion/assets/18121684/beec095d-12f8-420a-a21f-bac51c74846e)



If vscode can't find the include files you need to add them by yourself.
In my system I have added them as :
```json
"C_Cpp.default.includePath": [
  "/usr/local/include/opencv4/**",
  "/usr/local/include/opencv4",
  "/usr/local/include/eigen3/",
  "/usr/local/include/eigen3/Eigen/src/Core",
  "/usr/local/include/ceres"
] 
```

Clone project repo and try to build

```bash
git clone https://github.com/ycanay/StereoReconstructuion
cd StereoReconstructuion
mkdir build
cd build
cmake ..
make all
```

In total it should look something like this

![image](https://github.com/ycanay/StereoReconstructuion/assets/18121684/a95f9f32-52a8-445b-a82d-5a9223f535c7)


## Build and execute
```bash
cd build
cmake ..
make all
./Main
```
You will find the output images in this build directory.
