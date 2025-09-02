# 项目名称
cartographe 用于AMR进行建图和定位； 

## 项目安装

### x86_64编译
本地以来安装，需要注意，它作为`cartographer-ros`的依赖库安装在`cartographer/install`目录下保证`cartographer`与
`cartographer_ros`工程的`workspace`所在目录同级，否则默认会索引不到。
- **`debs` 依赖包**
```Bash
sudo apt-get update
sudo apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    lsb-release \
    ninja-build \
    stow
```

- `abseil`安装
```bash
git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
git checkout d902eb869bcfacc1bad14933ed9af4bed006d481
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_CXX_STANDARD=11 \
  -DCMAKE_CXX_STANDARD_REQUIRED=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl
  ..
ninja
sudo ninja install
cd /usr/local/stow
sudo stow absl
```

- `ceres`安装
```bash
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout 1.13.0
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCXX11=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTING=OFF \
  ..
ninja
CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install
```

- `cartographer` 编译
```bash
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PWD/../install \
  -DCMAKE_CXX_STANDARD=11 \
  -DCMAKE_CXX_STANDARD_REQUIRED=ON \
  ..
ninja
ninja install
```

#### 编译安装arm64
如上安装debs依赖包。
需要在arm64的机器上编译，编译指令如下, 需要在 10.4.35.36 USM服务器上,进入docker容器内部
将文件目录拷贝到 byd_app目录下的cartographer_ros目录下使用`cartographer_ros`的编译指令即可


## 项目使用


## 项目维护


#### 2025.07.22
- 继续LOG日志输出，打印调度队列耗时

#### 2025.09.2
- 新增cartographer支持CXX17编译，在`ubuntu-22.04`系统默认使用`17`，要指定`abseil-cpp`使用`17`进行编译。

## 项目贡献

## 项目协议