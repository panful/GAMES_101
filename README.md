**GAMES 101 作业**

## Windows
### OpenCV
下载[OpenCV](https://opencv.org/releases/)安装包，安装好之后设置环境变量`OpenCV_DIR=yourpath/opencv/build`，path中添加`yourpath/opencv/build/x64/vc16/bin`
### Eigen
下载[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)源码，使用cmake编译并安装，设置环境变量`Eigen3_DIR=yourpath/Eigen3/share/eigen3/cmake`
```
cmake ..
cmake --build . --config=Release
cmake --install . # 默认安装路径 C:\Program Files (x86)\Eigen3
```
## Linux
### OpenCV
```shell
sudo apt install libopencv-dev
```
### Eigen
```shell
sudo apt install libeigen3-dev
```