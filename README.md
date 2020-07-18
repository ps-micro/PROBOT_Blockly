# PROBOT Blockly功能包
为教育线PROBOT机械臂控制系统提供Blockly图形化编程功能。

## 组织结构
- block_libs： 使用Blockly Factory工具设计Block并导出的Block library，可导入该工具继续开发
- blockly_demo： Blockly图形化编程例程。由Blockly网页的编程视图区导出，可在网页中直接导入该程序文件进行使用
- frontend： Blockly网页前端实现源码
    + blockly
        * blocks： 存放block代码块源码
        * generators/python： Block-->Python 代码翻译器源码
    + pages/blockly.html： Blockly网页源码
- launch： PROBOT Blockly功能包的launch启动文件
- scripts： PROBOT Blockly功能包的启动脚本，由launch文件调用

## 使用
PROBOT Blockly是一个完整并独立的ROS功能包
- 创建ROS工作空间，下载源码、编译并启动
```
$ mkdir -p ~/blockly_ws/src
$ git clone https://gitee.com/ps-micro/probot_blockly.git
$ cd ~/blockly_ws
$ catkin_make
$ source ~/blockly_ws/devel/setup.bash
$ roslaunch probot_blockly probot_blockly.launch
```
- 打开浏览器，输入网址 `127.0.0.1:1234`，即可看到Blockly图形化编程界面

## 技术文档
- [【腾讯文档】PROBOT Blockly开发笔记](https://docs.qq.com/doc/DYnBYR1ZadEhWTkZ6)
