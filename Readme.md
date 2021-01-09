# ICP实验代码

本代码为ICP实验代码，算法使用SVD方法对ICP进行求解，具体原理的数学推导见《视觉SLAM十四讲：从理论到实践》7.9节。



### 系统

本实验环境为MacOS Mojave 10.14.6，在其他版本的MacOS以及Linux中也应当适用。



### 配置环境

安装cmake环境，参考[cmake官网](https://cmake.org/download/)。

安装pcl、Eigen库，本实验使用pcl-1.9和Eigen-3.3.7。

mac中安装指令如下

```bash
$ brew install pcl
$ brew install Eigen
```

根据Eigen库的具体安装路径，修改`CMakeLists.txt`:

```cmake
include_directories( "/usr/local/Cellar/eigen/3.3.7/include/eigen3/" ) 
```



### 编译文件

```bash
$ cd {代码目录}
$ mkdir build
$ cmake ..
$ make
```



### 运行代码

1. 将原始点云文件(后缀.asc)放入`data`目录中，依次命名为`C1.asc`、`C2.asc`…。
2. 依次执行如下指令

```bash
$ cd {代码目录}
$ ./build/ICP_PROC 
```

3. 输出文件存储为`data/output.asc`和`data/output.pcd`，前者可使用Geometric软件查看，后者可通过命令行指令查看

```bash
$ pcl_viewer ./data/output.pcd
# 界面上输入数字3可对其进行颜色渲染，便于可视化
```



### TroubleShooting

#### 1. 编译时出现“ld: library not found for -lflann clang: error: linker command failed with exit code 1”

修改/usr/local/share/pcl-1.9/Modules/FindFLANN.cmake文件，参考[解决方案](https://github.com/introlab/rtabmap/issues/351#issuecomment-453882844)。





