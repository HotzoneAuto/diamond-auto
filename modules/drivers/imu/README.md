LPMS惯性导航模块Cyber-RT组件(component)
(目前仅有文件框架，具体实现需要IMU参数、算法、数据等）
==================================================

一.文件组成结构
---------------

1.driver:

    -LPMS_driver_component.h

    -LPMS_driver_component.cc

    -BUILD

2.Dag:

    -LPMS.dag

3.Launch:

    -LPMS.launch

二.文件功能
-----------

1.LPMS_driver_component.h

基于component模板类派生出LPMS_driver_component类，声明Init()和Proc()函数

2.LPMS_driver_component.cc

实现对虚函数Init(), Proc()的重载，发布消息

3.BUILD文件

bazel编译工具使用，创建组件库和共享库文件

4.Dag文件

提供配置项

5.Launch文件

cyber_launch启动组件


