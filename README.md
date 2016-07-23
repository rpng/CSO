# calibration_stereo_odom (CSO)
Calibration the rigid transformation between the stereo and odometry

 ![image](https://github.com/doomzzju/CSO/blob/master/img.png)

**Authors:** Doom

Calibration_stereo_odom (CSO) is a tool to calibrate the rigid body transformation between a stereo camera and robot odometry.

#1. Before installation
This tool is compiled under Ubuntu(14.04) and g2o, libviso2, Suitesparse, libpng++ are required. 

## C++11 or C++0x Compiler
Use the new thread and chrono functionalities of C++11.

## G2O
The G2O(General Graph Optimization) is used to perform non-linear optimizations. 
[g2o](https://github.com/RainerKuemmerle/g2o). Make sure you have installed g2o into your PC.

## libviso2
The visual odometry library, used to calculate the stereo-vo to estimate the stereo's movement. The code has already involved in the ThirdParty folder of CSO.
**Website:** www.cvlibs.net

## Suitesparse, libpng++
use the command line below to install this lib.
```
$ sudo apt-get install libsuitesparse-dev libpng++-dev
```
#2. Building CSO
Clone the repository into your workspace, e.g CSO:
```
git clone https://github.com/doomzju/calibration_stereo_odom.git CSO
```
I provide a script `build.sh` to build the *ThirdParty* libraries and *calibration_stereo_odom*. Please make sure you have installed all required dependencies (see section 1). Execute:
```
cd CSO
chmod +x build.sh
./build.sh
```
after that you can find the execute file in the build folder.

#3. Usage and Examples.
In this tool, we offer two different ways to load stereo vo, one is using libviso2 the other is load pose file directly.
The Usage of this tool is:
```
./calibration_stereo_odom USE_VISO IMAGE_FOLDER USE_CLOSEFORM
```
If you want use libviso2 and your own image sequences to calculate and only use non-linear optimization only, e.g:
```
./calibration_stereo_odom true /home/doom/zed
```
If you do not want use libvsio2, please specify the vo pose folder, e.g:
```
./calibration_stereo_odom false /home/doom/zed
```
If you want use both graph optimization and closed form solution, e.g:
```
./calibration_stereo_odom true /home/doom/zed true
or
./calibration_stereo_odom false /home/doom/zed true
```

#4. Data format:
There are some test data in the /data folder of this project.
