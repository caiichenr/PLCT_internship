# ROS2 Humble 在 openEuler-24.03-x86_64 上的测试

## 测试环境
### 硬件信息
1. 处理器：AMD Ryzen 5 3550H
2. 内存：16 GB

### 软件信息
1. 宿主机OS：Arch Linux x86_64
2. 虚拟机OS：openEuler-24.03-x86
3. 镜像地址：https://mirror.iscas.ac.cn/openeuler/openEuler-24.03-LTS/virtual_machine_img/x86_64/

## 启动虚拟机镜像
> 关于系统安装，也可选择下载 ISO 镜像进行系统安装，可参考 [该教程](https://wiki.251.sh/openeuler_risc-v_qemu_install#%E5%9C%A8_qemu_%E5%86%85%E8%BF%90%E8%A1%8C_openeuler_risc-v_%E7%A7%BB%E6%A4%8D%E7%89%88)
>
>
> 关于QEMU，此处为了便于操作选用 Virt Manager，此处也可参考 [该教程](https://gitee.com/openeuler/ros/tree/master/user_doc/ROS-humble-oerv24.03-x86#ros2-humble-%E5%9C%A8-openeuler-2403-x86-%E4%B8%8A%E7%9A%84%E6%B5%8B%E8%AF%95)，直接在命令行中使用 qemu 进行虚拟机的配置

1. 下载已安装 openEuler 的 QEMU 架构镜像，并于 Virt Manager 中配置安装。
   
    ![图1_在QEMU中打开镜像](.\imgs\0_install_Euler_in_QEMU.png)

    ![图2_在QEMU中打开镜像](.\imgs\0_install_Euler_in_QEMU2.png)

2. 登录用户名及密码：
   1. 用户名：`root`
   2. 密码：`openEuler12#$`

## 测试环境安装

### XFCE 桌面安装
> 安装过程请以 [官网安装文档](https://docs.openeuler.org/zh/docs/24.03_LTS/docs/desktop/Install_XFCE.html) 为准

1. 安装字库
   ```
   sudo dnf install dejavu-fonts liberation-fonts gnu-*-fonts google-*-fonts
   ```
2. 安装Xorg
   ```
   sudo dnf install xorg-*
   ```
3. 安装XFCE及组件
   ```
   sudo dnf install xfwm4 xfdesktop xfce4-* xfce4-*-plugin network-manager-applet *fonts
   ```
5. 安装登录管理器
   ```
   sudo dnf install lightdm lightdm-gtk
   ```
6. 设置默认桌面为XFCE 通过root权限用户设置
   ```
   echo 'user-session=xfce' >> /etc/lightdm/lightdm.conf.d/60-lightdm-gtk-greeter.conf
   ```
7. 使用登录管理器登录XFCE
   ```
   sudo systemctl start lightdm
   ```
8. 设置开机自启动图形界面
   ```
   sudo systemctl enable lightdm
   sudo systemctl set-default graphical.target
   ```
9. 重启验证
    ```
    sudo reboot
    ```


### ROS 安装
> 安装过程请以 [官网安装文档](https://docs.openeuler.org/zh/docs/24.03_LTS/docs/ROS/%E5%AE%89%E8%A3%85%E4%B8%8E%E9%83%A8%E7%BD%B2.html) 为准

1. 修改软件源
   ```
    bash -c 'cat << EOF > /etc/yum.repos.d/ROS.repo
    [openEulerROS-humble]
    name=openEulerROS-humble
    baseurl=http://121.36.84.172/dailybuild/EBS-openEuler-24.03-LTS/EBS-openEuler-24.03-LTS/EPOL/multi_version/ROS/humble/x86_64/
    enabled=1
    gpgcheck=0
    EOF'
   ```

2. 安装 ROS2-humble
   ```
   dnf install "ros-humble-*" --skip-broken --exclude=ros-humble-generate-parameter-library-example
   ```

3. 在 shell 启动配置中加入 ros 环境激活配置
   ```
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

## 测试内容及测试结果
> 本文档基于 ROS2 提供的教程内容进行依次测试

> 以下内容基于教程内容： Beginner: CLI tools

### 1. Using turtlesim, ros2, and rqt

| 测试用例名 | 测试状态 |
| --- | --- |
| 测试 turtlesim 功能 | 成功 |
| 测试 rqt 功能| 失败 |

1. 测试 turtlesim 功能
   
   在两个终端中分别运行下述命令，可以启动 turtlesim 模拟界面，并可在 `turtle_teleop_key` 终端中控制海龟运动，测试通过。
   ```
   ros2 run turtlesim turtlesim_node
   ```
   ```
   ros2 run turtlesim turtle_teleop_key
   ```
   ![turtlesim测试](imgs\1_turtlesim.png)

2. 测试 rqt 功能
   
   在终端中运行下述命令。在打开的图形化界面中缺少相应的插件。测试失败，缺少ros-humble-rqt-common-plugins 包，与 [问题14](https://github.com/jiuyewxy/weloveinterns/issues/16) 中 problem10 一致
   ```
   rqt
   ```
   ![rqt测试](imgs\1_rqt.png)


### 2. Understanding nodes
| 测试用例名 | 测试状态 |
| --- | --- |
| 测试 node list 功能 | 成功 |
| 测试 node info 功能 | 成功 |

1. 打开 turtlesim
   ```
   ros2 run turtlesim turtlesim_node
   ```
   ```
   ros2 run turtlesim turtle_teleop_key
   ```
2. 运行 `node list`

    在新终端中输入以下命令，返回当前运行中的 nodes。测试通过。
   ```
   ros2 node list
   ```
   ![nodelist](imgs\2_node_list.png)

3. 重新分配 node 属性
   
   利用 remapping 更改 node 的属性。测试通过。
   ```
   ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
   ```

4. 运行 `node info`

    在新终端中输入以下命令，返回指定 node 的属性信息。测试通过。
    ```
   ros2 node info /my_turtle
   ```
   ![nodelist](imgs\2_node_info.png)

### 3. Understanding topics
| 测试用例名 | 测试状态 |
| --- | --- |
| 测试 rqt_graph | 失败 |
| 测试 topic list| 成功 |
| 测试 topic echo| 成功 |
| 测试 topic info| 成功 |
| 测试 interface show| 成功 |
| 测试 topic pub | 成功 |
| 测试 topic hz  | 成功 |

1. 打开 rqt_graph可视化 node 与 topic
   
   运行以下命令，或在 `rqt` 中选择 `Plugins > Introspection > Node Graph`。测试 失败。

2. 测试 topic list，可以返回当前系统中激活的topics
   
   运行以下命令，在命令后加上 `-t` 参数会在 topics 后加上对应的类型。测试通过。
   ```
   ros2 topic list
   ```
   ![topiclist](imgs\3_topics_list.png)

3. 测试 topic echo
   
   运行以下命令，测试通过。
   ```
   ros2 topic echo /turtle1/cmd_vel
   ```
   ![topicecho](imgs\3_topics_echo.png)

4. 测试 topic info
   
   运行以下命令，测试通过。
   ```
   ros2 topic info /turtle1/cmd_vel
   ```
   ![topicinfo](imgs\3_topics_info.png)

5. 测试 interface show
   
   运行以下命令，测试通过。
   ```
   ros2 interface show geometry_msgs/msg/Twist
   ```
   ![topic_interface_show](imgs\3_topics_interface_show.png)

6. 测试 topic pub
   
   运行以下命令，可以观察到海龟的移动。测试通过。
   ```
   ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
   ```
   ![topicpub](imgs\3_topics_pub.png)

7. 测试 topic hz
   
   运行以下命令。测试通过。
   ```
   ros2 topic hz /turtle1/pose
   ```
   ![topichz](imgs\3_topics_hz.png)


### 4. Understanding services
| 测试用例名 | 测试状态 |
| --- | --- |
| service list | 通过 |
| service type | 通过 |
| service find | 通过 |
| interface show | 通过 |
| service call | 通过 |


1. 在两个终端中打开 turtlesim。

2. 测试 topic list
   
   运行以下命令，测试通过。
   ```
   ros2 service list
   ```
   ![servicelist](imgs\4_services_list.png)

3. 测试 topic type
   
   运行以下命令，测试通过。
   ```
   ros2 service type /clear
   ```
   ![servicetype](imgs\4_services_type.png)

4. 测试 topic list -t
   
   运行以下命令，测试通过。
   ```
   ros2 service list -t
   ```
   ![servicelistt](imgs\4_services_list_t.png)

5. 测试 topic find
   
   运行以下命令，测试通过。
   ```
   ros2 service find std_srvs/srv/Empty
   ```
   ![servicefind](imgs\4_services_find.png)

6. 测试 interface show
   
   运行以下命令，测试通过。
   ```
   ros2 interface show turtlesim/srv/Spawn
   ```
   ![serviceinterfaceshow](imgs\4_services_interface_show.png)

7. 测试 interface service call
   
   运行以下命令，测试通过。
   ```
   ros2 service call /clear std_srvs/srv/Empty
   ```
   ![servicecall](imgs\4_services_call.png)


### 5. Understanding parameters
| 测试用例名 | 测试状态 |
| --- | --- |
| param list | 成功 |
| param  get | 成功 |
| param  set | 成功 |
| param dump | 成功 |
| param load | 成功 |

1. 在两个终端中打开 turtlesim。
   
2. 测试 param list
   
   运行以下命令，测试通过。
   ```
   ros2 param list
   ```
   ![paramlist](imgs\5_param_list.png)

3. 测试 param get
   
   运行以下命令，测试通过。
   ```
   ros2 param get /turtlesim background_g
   ```
   ![paramget](imgs\5_param_get.png)

4. 测试 param set
   
   运行以下命令，测试通过。
   ```
   ros2 param set /turtlesim background_r 150
   ```
   ![paramset](imgs\5_param_set.png)

5. 测试 param dump
   
   运行以下命令，测试通过。
   ```
   ros2 param dump /turtlesim > turtlesim.yaml
   ```
   ![paramdump](imgs\5_param_dump.png)

6. 测试 param load
   
   运行以下命令，测试通过。
   ```
   ros2 param load /turtlesim turtlesim.yaml
   ```
   ![paramload](imgs\5_param_load.png)


### 6. Understanding actions
| 测试用例名 | 测试状态 |
| --- | --- |
| action list | 成功 |
| action info | 成功 |
| interface show | 成功 |
| action send_goal | 成功 |

1. 在两个终端中打开 turtlesim。
   
   通过 /teleop_turtle 结点控制小海龟的移动，证明 action 可以使用。

2. 测试 action list
   
   运行以下命令，测试通过。
   ```
   ros2 action list
   ```
   ![actionlist](imgs\6_action_list.png)

3. 测试 action info
   
   运行以下命令，测试通过。
   ```
   ros2 action info /turtle1/rotate_absolute
   ```
   ![actioninfo](imgs\6_action_info.png)

4. 测试 interface show
   
   运行以下命令，测试通过。
   ```
   ros2 interface show turtlesim/action/RotateAbsolute
   ```
   ![actioninterface](imgs\6_action_interface_show.png)

5. 测试 action send_goal
   
   运行以下命令，测试通过。
   ```
   ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
   ```
   ![actionsendgoal](imgs\6_action_send_goal.png)

### 7. Using rqt_console to view logs
| 测试用例名 | 测试状态 |
| --- | --- |
| 测试 rqt_console 功能 | 成功 |

1. 打开 `rqt_console`
   
   ```
   ros2 run rqt_console rqt_console
   ```
    ![rqt_console](imgs\7_rqt_console.png)

2. 在 `rqt_console` 中查看 log
   
   打开 turtlesim 并运行以下命令。可以在终端中看到海龟撞墙的log信息。测试通过
   ```
   ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
   ```
    ![rqt_console_msg](imgs\7_rqt_console_log.png)


### 8. Launching nodes
| 测试用例名 | 测试状态 |
| --- | --- |
| 测试 Launching 功能 | 成功 |

1. 测试运行启动一个 Launch File
   
   在新终端中运行以下命令。测试通过。
   ```
   ros2 launch turtlesim multisim.launch.py
   ```
    ![rqt_console_msg](imgs\8_launching_nodes.png)
