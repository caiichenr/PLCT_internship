# EIGEN Benchmarks 测试

## 测试环境
### 硬件信息
1. 处理器：AMD Ryzen 5 3550H
2. 内存：16 GB

### 软件信息
1. 宿主机OS：Arch Linux x86_64
2. 虚拟机OS：[openEuler-24.03-x86](https://mirror.iscas.ac.cn/openeuler/openEuler-24.03-LTS/virtual_machine_img/x86_64/) ; Ubuntu-Desktop-22.04


### 测试信息
1. Eigen 版本：[3.3.8](https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.bz2)
2. Benchmars 情况：https://eigen.tuxfamily.org/index.php?title=How_to_run_the_benchmark_suite

```
# create a working directory
$ mkdir build
$ cd build
# create the makefiles
$ cmake <path_to_eigen_source> -DCMAKE_BUILD_TYPE=Release -DEIGEN_BUILD_BTL=ON -DEIGEN3_INCLUDE_DIR=<path_to_eigen_source>
# compile the benchmarks
$ cd bench/btl
$ make -j4
# run the benchmark (on a single core)
# VERY IMPORTANT : logout, log into a console (ctrl+shift+F1) and shutdown your X server (e.g.: sudo init 3), and stop as most as services as you can
$ OMP_NUM_THREADS=1 ctest -V
# - go sleep -
# copy the results into the same directory
$ mkdir data/set1
$ cp libs/*/*.dat data/set1
# build the nice plots
$ cd data
$ ./go_mean set1/
# the plots (png and pdf files) are in set1/
```


## EIGEN on OpenEuler vs. Ubuntu

### OpenEuler 测试结果详见目录 `intern_tasks\task32\OpenEuler24.03_set1\`
### Ubuntu 测试结果详见目录 `intern_tasks\task32\Ubuntu22.04_set1\`



## EIGEN on x86 vs. arm vs. riscv


