'use strict';

goog.provide('Blockly.Python.probot');
goog.require('Blockly.Python');

Blockly.Python['arm_init'] = function(block) {
  var code = "\n\
import moveit_commander, os, sys\n\
import subprocess\n\
import math, numpy\n\
\n\
from geometry_msgs.msg import PoseStamped, Pose\n\
from copy import deepcopy\n\
\n\
# ctrl commands for topic probot_controller_ctrl probot_msgs/ControllerCtrl\n\
CONTROLLER_CTRL_SIMULATION              = 1\n\
CONTROLLER_CTRL_LIVE                    = 2\n\
CONTROLLER_CTRL_START_MANUALLY_CONTROL  = 3\n\
CONTROLLER_CTRL_STOP_MANUALLY_CONTROL   = 4\n\
CONTROLLER_CTRL_START                   = 5\n\
CONTROLLER_CTRL_STOP                    = 6\n\
CONTROLLER_CTRL_PAUSE                   = 7\n\
CONTROLLER_CTRL_AUTO                    = 8\n\
CONTROLLER_CTRL_MANUAL                  = 9\n\
CONTROLLER_CTRL_SUPPLY_POWER            = 10\n\
CONTROLLER_CTRL_CLEAR_ERROR             = 11\n\
CONTROLLER_CTRL_DEMO_START              = 12\n\
CONTROLLER_CTRL_DEMO_STOP               = 13\n\
CONTROLLER_CTRL_ZERO_CALIBRATION_AUTO   = 14\n\
CONTROLLER_CTRL_ZERO_CALIBRATION_MANUAL = 15\n\
\n\
control_mode = 0\n\
probot_initialized = 1\n\
\n\
joint_tolerance_default = 0.001\n\
acc_scale_default = 0.5\n\
vel_scale_default = 0.5\n\
pos_tolerance_default = 0.01\n\
ori_tolerance_default = 0.01\n\
\n\
# 初始化move_group\n\
moveit_commander.roscpp_initialize(sys.argv)\n\
\n\
try:\n\
  # 初始化需要使用move group控制的机械臂中的arm group\n\
  arm = moveit_commander.MoveGroupCommander('manipulator')\n\
except Exception as e:\n\
  rospy.logerr(\"Run program failed! Please check the PROBOT runtime is running now...\")\n\
  sys.exit()\n\
\n\
# 设置机械臂运动的允许误差值\n\
arm.set_goal_joint_tolerance(joint_tolerance_default)\n\
\n\
# 设置允许的最大速度和加速度\n\
arm.set_max_acceleration_scaling_factor(acc_scale_default)\n\
arm.set_max_velocity_scaling_factor(vel_scale_default)\n\
\n\
# 获取终端link的名称\n\
end_effector_link = arm.get_end_effector_link()\n\
\n\
# 设置目标位置所使用的参考坐标系\n\
reference_frame = 'base_link'\n\
arm.set_pose_reference_frame(reference_frame)\n\
\n\
# 当运动规划失败后，允许重新规划\n\
arm.allow_replanning(True)\n\
\n\
# 设置位置(单位：米)和姿态（单位：弧度）的允许误差\n\
arm.set_goal_position_tolerance(pos_tolerance_default)\n\
arm.set_goal_orientation_tolerance(ori_tolerance_default)\n\
\n\
# 读取控制器数字IO输入电平\n\
def get_digital_input(io_num):\n\
  if probot_initialized != 1:\n\
    rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
    sys.exit()\n\
  if control_mode != CONTROLLER_CTRL_LIVE:\n\
    rospy.logerr(\"Run program failed! IO Blocks shoul be used in Live mode.\")\n\
    return -1\n\
  if io_num > 16:\n\
    rospy.logerr(\"Run program failed! The IO number specified is out of range(1-16).\")\n\
    return -1\n\
\n\
  m_pipe = subprocess.Popen(\"rostopic echo -n 1 probot_status | grep inputIOs\", shell=True, stdout=subprocess.PIPE)\n\
  m_pipe_out = m_pipe.stdout.readlines()\n\
  # line format:\n\
  # inputIOs: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n\
  for line in m_pipe_out:\n\
    input_val_str = line.strip()\n\
    io_level = input_val_str[11+3*(io_num-1)]\n\
    rospy.loginfo(\"Get digtial input[{}] status: {}\".format(io_num, io_level))\n\
    return int(io_level)\n\
";

  var dropdown_mode = block.getFieldValue('mode');
  var checkbox_enable = block.getFieldValue('enable') == 'TRUE';

  code += "\n# 设置控制模式为真机/仿真";
  if (dropdown_mode == 0) {
    code += "\n\
control_mode = CONTROLLER_CTRL_LIVE\n\
shell_cmd = \"rostopic pub -1 probot_controller_ctrl probot_msgs/ControllerCtrl -- '{ctrl: \" + \
  str(CONTROLLER_CTRL_LIVE) + \"}' >/dev/null 2>&1\"\n\
";
  } else {
    code += "\n\
control_mode = CONTROLLER_CTRL_SIMULATION\n\
shell_cmd = \"rostopic pub -1 probot_controller_ctrl probot_msgs/ControllerCtrl -- '{ctrl: \" + \
  str(CONTROLLER_CTRL_SIMULATION) + \"}' >/dev/null 2>&1\"\n\
";
  }
  code += "os.system(shell_cmd)\n";

  code += "\n# 使能机械臂控制";
  if (checkbox_enable) {
    code += "\n\
if control_mode == CONTROLLER_CTRL_LIVE:\n\
  shell_cmd = \"rostopic pub -1 probot_controller_ctrl probot_msgs/ControllerCtrl -- '{ctrl: \" + \
    str(CONTROLLER_CTRL_START_MANUALLY_CONTROL) + \"}' >/dev/null 2>&1\"\n\
  os.system(shell_cmd)\n\
";
  }

  return code;
};

Blockly.Python['arm_deinit'] = function(block) {
  var checkbox_stop = block.getFieldValue('stop') == 'TRUE';
  var checkbox_gohome = block.getFieldValue('gohome') == 'TRUE';
  var checkbox_disable = block.getFieldValue('disable') == 'TRUE';

  var code = "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
";
  if (checkbox_stop) {
    code += "\n\
# 机械臂停止运动\n\
if control_mode == CONTROLLER_CTRL_LIVE:\n\
  shell_cmd = \"rostopic pub -1 probot_controller_ctrl probot_msgs/ControllerCtrl -- '{ctrl: \" + \
    str(CONTROLLER_CTRL_STOP) + \"}' >/dev/null 2>&1\"\n\
  os.system(shell_cmd)\n\
";
  }
  if (checkbox_gohome) {
    code += "\n\
# 控制机械臂先回到初始化位置\n\
arm.set_named_target('home')\n\
arm.go()\n\
rospy.sleep(1)\n\
";
  }
  if (checkbox_disable) {
    code += "\n\
# 除能机械臂控制\n\
if control_mode == CONTROLLER_CTRL_LIVE:\n\
  shell_cmd = \"rostopic pub -1 probot_controller_ctrl probot_msgs/ControllerCtrl -- '{ctrl: \" + \
    str(CONTROLLER_CTRL_STOP_MANUALLY_CONTROL) + \"}' >/dev/null 2>&1\"\n\
  os.system(shell_cmd)\n\
";
  }
  code += "\n\
# 关闭并退出moveit\n\
moveit_commander.roscpp_shutdown()\n\
";

  return code;
};

Blockly.Python['jog_joint'] = function(block) {
  var text_joint1 = block.getFieldValue('joint1');
  var text_joint2 = block.getFieldValue('joint2');
  var text_joint3 = block.getFieldValue('joint3');
  var text_joint4 = block.getFieldValue('joint4');
  var text_joint5 = block.getFieldValue('joint5');
  var text_joint6 = block.getFieldValue('joint6');
  var checkbox_traj_execute = block.getFieldValue('traj_execute') == 'TRUE';

  var angle_joint1 = Number(text_joint1)
  if (angle_joint1 >= -360 && angle_joint1 <= 360) {
  } else {
    angle_joint1 = 0;
  }
  var angle_joint2 = Number(text_joint2)
  if (angle_joint2 >= -360 && angle_joint2 <= 360) {
  } else {
    angle_joint2 = 0;
  }
  var angle_joint3 = Number(text_joint3)
  if (angle_joint3 >= -360 && angle_joint3 <= 360) {
  } else {
    angle_joint3 = 0;
  }
  var angle_joint4 = Number(text_joint4)
  if (angle_joint4 >= -360 && angle_joint4 <= 360) {
  } else {
    angle_joint4 = 0;
  }
  var angle_joint5 = Number(text_joint5)
  if (angle_joint5 >= -360 && angle_joint5 <= 360) {
  } else {
    angle_joint5 = 0;
  }
  var angle_joint6 = Number(text_joint6)
  if (angle_joint6 >= -360 && angle_joint6 <= 360) {
  } else {
    angle_joint6 = 0;
  }

  var pi = 3.141592654;
  var joint1_pos = angle_joint1 * pi / 180;
  var joint2_pos = angle_joint2 * pi / 180;
  var joint3_pos = angle_joint3 * pi / 180;
  var joint4_pos = angle_joint4 * pi / 180;
  var joint5_pos = angle_joint5 * pi / 180;
  var joint6_pos = angle_joint6 * pi / 180;
  
  var code = "";
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）\n\
joint_positions = [" + joint1_pos.toString() + ", " + joint2_pos.toString() + ", " + joint3_pos.toString() + 
  ", " + joint4_pos.toString() + ", " + joint5_pos.toString() + ", " + joint6_pos.toString() + "]\n\
try:\n\
  arm.set_joint_value_target(joint_positions)\n\
";
  if (checkbox_traj_execute) {
    code += "\
  # 控制机械臂完成运动\n\
  arm.go()\n\
";
  }
  code += "\
except Exception as e:\n\
  rospy.logerr(\"Run program failed! Please check the target you set.\")\n\
";

  return code;
};

Blockly.Python['go_home'] = function(block) {
  var code = "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 控制机械臂回到初始化位置\n\
arm.set_named_target('home')\n\
arm.go()\n\
";
  return code;
};

Blockly.Python['set_max_velocity_scaling_factor'] = function(block) {
  var text_velocity_scale = block.getFieldValue('velocity_scale');
  var velocity_scale = Number(text_velocity_scale);
  if (velocity_scale >= 0 && velocity_scale <= 100) {
    velocity_scale = velocity_scale / 100;
  } else {
    velocity_scale = 0;
  }

  var code = "\nvelocity_scale = " + velocity_scale.toString();
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 设置运动速度系数\n\
arm.set_max_velocity_scaling_factor(velocity_scale)\n\
";
  return code;
};

Blockly.Python['set_max_acceleration_scaling_factor'] = function(block) {
  var text_acceleration_scale = block.getFieldValue('acceleration_scale');
  var acceleration_scale = Number(text_acceleration_scale);
  if (acceleration_scale >= 0 && acceleration_scale <= 100) {
    acceleration_scale = acceleration_scale / 100;
  } else {
    acceleration_scale = 0;
  }

  var code = "\nacceleration_scale = " + acceleration_scale.toString();
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 设置运动加速度系数\n\
arm.set_max_acceleration_scaling_factor(acceleration_scale)\n\
";
  return code;
};

Blockly.Python['sleep_s'] = function(block) {
  var text_time_sleep_s = block.getFieldValue('time_sleep_s');
  var time_sleep_s = Number(text_time_sleep_s);
  if (time_sleep_s < 0) {
    time_sleep_s = 0;
  }

  var code = "\ntime_sleep_s = " + time_sleep_s.toString();
  code += "\n\
# 系统休眠一段时间，机械臂停留不动\n\
rospy.sleep(time_sleep_s)\n\
";
  return code;
};

Blockly.Python['set_goal_joint_tolerance'] = function(block) {
  var angle_goal_joint_tolerance = block.getFieldValue('goal_joint_tolerance');
  var pi = 3.141592654;
  var joint_tolerance_rad = angle_goal_joint_tolerance * pi / 180;

  var code = "\njoint_tolerance_rad = " + joint_tolerance_rad.toString();
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 设置关节运动的允许误差\n\
arm.set_goal_joint_tolerance(joint_tolerance_rad)\n\
";
  return code;
};

Blockly.Python['jog_pose'] = function(block) {
  var text_pos_x = block.getFieldValue('pos_x');
  var text_pos_y = block.getFieldValue('pos_y');
  var text_pos_z = block.getFieldValue('pos_z');
  var text_pos_w = block.getFieldValue('pos_w');
  var target_pos_x = Number(text_pos_x);
  if (target_pos_x < 0) {
    target_pos_x = 0;
  }
  var target_pos_y = Number(text_pos_y);
  if (target_pos_y < 0) {
    target_pos_y = 0;
  }
  var target_pos_z = Number(text_pos_z);
  if (target_pos_z < 0) {
    target_pos_z = 0;
  }
  var target_pos_w = Number(text_pos_w);
  if (target_pos_w >= 0 && target_pos_w <= 1) {
  } else {
    target_pos_w = 1;
  }

  var code = "";
  code += "\ntarget_pos_x = " + target_pos_x.toString();
  code += "\ntarget_pos_y = " + target_pos_y.toString();
  code += "\ntarget_pos_z = " + target_pos_z.toString();
  code += "\ntarget_pos_w = " + target_pos_w.toString();
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，\n\
# 姿态使用四元数描述，基于 reference_frame 坐标系\n\
target_pose = PoseStamped()\n\
target_pose.header.frame_id = reference_frame\n\
target_pose.header.stamp = rospy.Time.now()\n\
target_pose.pose.position.x = target_pos_x\n\
target_pose.pose.position.y = target_pos_y\n\
target_pose.pose.position.z = target_pos_z\n\
target_pose.pose.orientation.w = target_pos_w\n\
\n\
try:\n\
  # 设置机器臂当前的状态作为运动初始状态\n\
  arm.set_start_state_to_current_state()\n\
  # 设置机械臂终端运动的目标位姿\n\
  arm.set_pose_target(target_pose, end_effector_link)\n\
";
  var checkbox_traj_plan = block.getFieldValue('traj_plan') == 'TRUE';
  var checkbox_traj_execute = block.getFieldValue('traj_execute') == 'TRUE';
  if (checkbox_traj_plan) {
    code += "\
  # 规划运动路径\n\
  traj = arm.plan()\n\
";
  }
  if (checkbox_traj_execute) {
      if (!checkbox_traj_plan) {
    code += "\
  # 规划运动路径\n\
  traj = arm.plan()\n\
";
      }
    code += "\
  # 按照规划的运动路径控制机械臂运动\n\
  arm.execute(traj)\n\
";
  }
  code += "\
except Exception as e:\n\
  rospy.logerr(\"Run program failed! Please check the target you set.\")\n\
";
  return code;
};

Blockly.Python['jog_pose_rpy'] = function(block) {
  var text_pos_x = block.getFieldValue('pos_x');
  var text_pos_y = block.getFieldValue('pos_y');
  var text_pos_z = block.getFieldValue('pos_z');
  var text_angle_roll = block.getFieldValue('angle_roll');
  var text_angle_pitch = block.getFieldValue('angle_pitch');
  var text_angle_yaw = block.getFieldValue('angle_yaw');

  var target_pos_x = Number(text_pos_x);
  if (target_pos_x < 0) {
    target_pos_x = 0;
  }
  var target_pos_y = Number(text_pos_y);
  if (target_pos_y < 0) {
    target_pos_y = 0;
  }
  var target_pos_z = Number(text_pos_z);
  if (target_pos_z < 0) {
    target_pos_z = 0;
  }

  var angle_roll = Number(text_angle_roll)
  if (angle_roll >= -360 && angle_roll <= 360) {
  } else {
    angle_roll = 0;
  }
  var angle_pitch = Number(text_angle_pitch)
  if (angle_pitch >= -360 && angle_pitch <= 360) {
  } else {
    angle_pitch = 0;
  }
  var angle_yaw = Number(text_angle_yaw)
  if (angle_yaw >= -360 && angle_yaw <= 360) {
  } else {
    angle_yaw = 0;
  }
  var pi = 3.141592654;
  var target_roll_rad  = angle_roll * pi / 180;
  var target_pitch_rad = angle_pitch * pi / 180;
  var target_yaw_rad   = angle_yaw * pi / 180;

  var code = "";
  code += "\ntarget_pos_x = " + target_pos_x.toString();
  code += "\ntarget_pos_y = " + target_pos_y.toString();
  code += "\ntarget_pos_z = " + target_pos_z.toString();
  code += "\ntarget_roll_rad  = " + target_roll_rad.toString();
  code += "\ntarget_pitch_rad = " + target_pitch_rad.toString();
  code += "\ntarget_yaw_rad   = " + target_yaw_rad.toString();
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，\n\
# 姿态使用航向角描述，基于 reference_frame 坐标系\n\
target_pose = PoseStamped()\n\
target_pose.header.frame_id = reference_frame\n\
target_pose.header.stamp = rospy.Time.now()\n\
target_pose.pose.position.x = target_pos_x\n\
target_pose.pose.position.y = target_pos_y\n\
target_pose.pose.position.z = target_pos_z\n\
\n\
target_rpy = []\n\
target_rpy.append(target_roll_rad)\n\
target_rpy.append(target_pitch_rad)\n\
target_rpy.append(target_yaw_rad)\n\
\n\
try:\n\
  # 设置机器臂当前的状态作为运动初始状态\n\
  arm.set_start_state_to_current_state()\n\
  # 设置机械臂终端运动的目标位姿\n\
  arm.set_pose_target(target_pose, end_effector_link)\n\
  arm.set_rpy_target(target_rpy, end_effector_link)\n\
";
  var checkbox_traj_plan = block.getFieldValue('traj_plan') == 'TRUE';
  var checkbox_traj_execute = block.getFieldValue('traj_execute') == 'TRUE';
  if (checkbox_traj_plan) {
    code += "\
  # 规划运动路径\n\
  traj = arm.plan()\n\
";
  }
  if (checkbox_traj_execute) {
      if (!checkbox_traj_plan) {
    code += "\
  # 规划运动路径\n\
  traj = arm.plan()\n\
";
      }
    code += "\
  # 按照规划的运动路径控制机械臂运动\n\
  arm.execute(traj)\n\
";
  }
  code += "\
except Exception as e:\n\
  rospy.logerr(\"Run program failed! Please check the target you set.\")\n\
";
  return code;
};

Blockly.Python['set_allow_replanning'] = function(block) {
  var checkbox_allow_replanning = block.getFieldValue('allow_replanning') == 'TRUE';
  var code = "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 当运动规划失败后，是否允许重新规划\
";
  if (checkbox_allow_replanning) {
    code += "\n\
arm.allow_replanning(True)\n\
";
  } else {
    code += "\n\
arm.allow_replanning(False)\n\
";
  }
  return code;
};

Blockly.Python['set_goal_position_tolerance'] = function(block) {
  var text_goal_position_tolerance = block.getFieldValue('goal_position_tolerance');
  var position_tolerance = Number(text_goal_position_tolerance);
  if (position_tolerance < 0) {
    position_tolerance = 0.001;
  }

  var code = "";
  code += "\nposition_tolerance = " + position_tolerance.toString();
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 设置运动位置的允许误差(m)\n\
arm.set_goal_position_tolerance(position_tolerance)\n\
";
  return code;
};

Blockly.Python['set_goal_orientation_tolerance'] = function(block) {
  var angle_goal_orientation_tolerance = block.getFieldValue('goal_orientation_tolerance');
  var pi = 3.141592654;
  var ori_tolerance_rad = angle_goal_orientation_tolerance * pi / 180;

  var code = "";
  code += "\nori_tolerance_rad = " + ori_tolerance_rad.toString();
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 设置运动姿态的允许误差(弧度)\n\
arm.set_goal_orientation_tolerance(ori_tolerance_rad)\n\
";
  return code;
};

Blockly.Python['set_digital_output'] = function(block) {
  var text_io_num = block.getFieldValue('io_num');
  var dropdown_io_level = block.getFieldValue('io_level');

  var io_num = Number(text_io_num)
  var io_level = Number(dropdown_io_level)

  var code = "";
  code += "\nio_num   = " + io_num.toString();
  code += "\nio_level = " + io_level.toString();
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
if control_mode != CONTROLLER_CTRL_LIVE:\n\
  rospy.logerr(\"Run program failed! IO Blocks shoul be used in Live mode.\")\n\
  sys.exit()\n\
if io_num > 16:\n\
  rospy.logerr(\"Run program failed! The IO number specified is out of range(1-16).\")\n\
  sys.exit()\n\
# 设置控制器数字IO输出电平\n\
shell_cmd = \"rostopic pub -1 probot_set_output_io probot_msgs/SetOutputIO -- '{mask: \" + \
  str(1<<(io_num-1)) + \", status: \" + str(io_level<<(io_num-1)) + \"}' >/dev/null 2>&1\"\n\
os.system(shell_cmd)\n\
";
  return code;
};

Blockly.Python['get_digital_input'] = function(block) {
  var text_io_num = block.getFieldValue('io_num');
  var io_num = Number(text_io_num)

  var code = "get_digital_input(" + text_io_num + ")";
  return [code, Blockly.Python.ORDER_FUNCTION_CALL];
};

Blockly.Python['if_digital_input'] = function(block) {
  var text_io_num = block.getFieldValue('io_num');
  var dropdown_io_level = block.getFieldValue('io_level');
  var statements_name = Blockly.Python.statementToCode(block, 'NAME');

  var code = "";
  code += "\nio_num   = " + text_io_num;
  code += "\nio_level = " + dropdown_io_level;
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
if control_mode != CONTROLLER_CTRL_LIVE:\n\
  rospy.logerr(\"Run program failed! IO Blocks shoul be used in Live mode.\")\n\
  sys.exit()\n\
# 如果控制器数字IO输入电平为io_level，则执行\n\
if get_digital_input(io_num) == io_level:\n\
";
  if (statements_name.length == 0) {
    code += "  pass";
  } else {
    code += statements_name;
  }
  return code;
};

Blockly.Python['digital_level'] = function(block) {
  var text_level = block.getFieldValue('level');

  var code = "" + text_level;
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python['move_in_line'] = function(block) {
  var dropdown_xyz_direction = block.getFieldValue('xyz_direction');
  var dropdown_direction = block.getFieldValue('direction');
  var text_distance = block.getFieldValue('distance');
  var text_step_m = block.getFieldValue('step_m');
  var text_max_plan_times = block.getFieldValue('max_plan_times');
  var checkbox_traj_execute = block.getFieldValue('traj_execute') == 'TRUE';

  var distance = Number(text_distance);
  if (distance > 0) {
  } else {
    distance = 0;
    text_distance = "0";
  }
  var max_plan_times = Number(text_max_plan_times);
  if (max_plan_times > 0 && max_plan_times < 1000) {
  } else {
    text_max_plan_times = "20";
  }
  var step_m = Number(text_step_m);
  if (step_m > 0 && step_m < distance) {
  } else {
    step_m = 0.01;
    text_step_m = "0.01";
    if (step_m > distance) {
      text_step_m = text_distance;
    }
  }
  
  var code = '';
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 获取当前位姿数据最为机械臂运动的起始位姿\n\
start_pose = arm.get_current_pose(end_effector_link).pose\n\
\n\
# 初始化路点列表\n\
waypoints = []\n\
# 将初始位姿加入路点列表\n\
# waypoints.append(start_pose)\n\
# 设置路点数据，并加入路点列表\n\
wpose = deepcopy(start_pose)\n\
";
  var xyz_direction = Number(dropdown_xyz_direction);
  var direction = Number(dropdown_direction);
  if (xyz_direction == 0) { // x
    if (direction == 0) { // 正方向
      code += "wpose.position.x += " + text_distance + "\n";
    } else { // 负方向
      code += "wpose.position.x -= " + text_distance + "\n";
    }
  } else if (xyz_direction == 1) { // y
    if (direction == 0) { // 正方向
      code += "wpose.position.y += " + text_distance + "\n";
    } else { // 负方向
      code += "wpose.position.y -= " + text_distance + "\n";
    }
  } else { // z
    if (direction == 0) { // 正方向
      code += "wpose.position.z += " + text_distance + "\n";
    } else { // 负方向
      code += "wpose.position.z -= " + text_distance + "\n";
    }
  }
  code += "\n\
waypoints.append(deepcopy(wpose))\n\
\n\
fraction = 0.0   #路径规划覆盖率\n\
maxtries = " + text_max_plan_times + "   #最大尝试规划次数\n\
attempts = 0     #已经尝试规划次数\n\
\n\
# 设置机器臂当前的状态作为运动初始状态\n\
arm.set_start_state_to_current_state()\n\
\n\
# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点\n\
while fraction < 1.0 and attempts < maxtries:\n\
    (plan, fraction) = arm.compute_cartesian_path (\n\
                            waypoints,   # waypoint poses，路点列表\n\
                            " + text_step_m + ",        # eef_step，终端步进值\n\
                            0.0,         # jump_threshold，跳跃阈值\n\
                            True)        # avoid_collisions，避障规划\n\
    # 尝试次数累加\n\
    attempts += 1\n\
    # 打印运动规划进程\n\
    if attempts % 10 == 0:\n\
        rospy.loginfo(\"Still trying after \" + str(attempts) + \" attempts...\")\n\
\n\
# 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动\n\
if fraction == 1.0:\n\
    rospy.loginfo(\"Path computed successfully. Moving the arm.\")\n\
";
  if (checkbox_traj_execute) {
    code += "\
    arm.execute(plan)\n";
  }
  code += "\
# 如果路径规划失败，则打印失败信息\n\
else:\n\
    rospy.loginfo(\"Path planning failed with only \" + str(fraction) + \" success after \" + str(maxtries) + \" attempts.\")\n\
";

  return code;
};

Blockly.Python['move_in_circle'] = function(block) {
  var text_circle_r = block.getFieldValue('circle_r');
  var dropdown_xyz_direction = block.getFieldValue('xyz_direction');
  var angle_move = block.getFieldValue('angle_move');
  var text_step_m = block.getFieldValue('step_m');
  var text_max_plan_times = block.getFieldValue('max_plan_times');
  var checkbox_traj_execute = block.getFieldValue('traj_execute') == 'TRUE';

  var max_plan_times = Number(text_max_plan_times);
  if (max_plan_times > 0 && max_plan_times < 1000) {
  } else {
    text_max_plan_times = "20";
  }
  
  var code = '';
  code += "\n\
if probot_initialized != 1:\n\
  rospy.logerr(\"Run program failed! Robot is not initialized! Please place the block 机械臂设置--机械臂初始化 at the beginning.\")\n\
  sys.exit()\n\
# 获取当前位姿数据最为机械臂运动的起始位姿\n\
circle_point = arm.get_current_pose(end_effector_link)\n\
\n\
# 初始化路点列表\n\
waypoints = []\n\
# 将初始位姿加入路点列表\n\
waypoints.append(circle_point.pose)\n\
";
  var xyz_direction = Number(dropdown_xyz_direction);
  if (xyz_direction == 0) { // xy
    code += "\n\
centerA = circle_point.pose.position.y\n\
centerB = circle_point.pose.position.x\n\
";
  } else if (xyz_direction == 1) { // yz
    code += "\n\
centerA = circle_point.pose.position.z\n\
centerB = circle_point.pose.position.y\n\
";
  } else { // xz
    code += "\n\
centerA = circle_point.pose.position.z\n\
centerB = circle_point.pose.position.x\n\
";
  }
  var circle_r = Number(text_circle_r);
  var distance = 2 * 3.14 * circle_r * angle_move / 360 * 10;
  code += "\n\
radius = " + text_circle_r + "\n\
for th in numpy.arange(0, " + distance.toString() + ", " + text_step_m + "):\
";
  if (xyz_direction == 0) { // xy
    code += "\n\
    circle_point.pose.position.y = centerA + radius * math.cos(th)\n\
    circle_point.pose.position.x = centerB + radius * math.sin(th)\n\
";
  } else if (xyz_direction == 1) { // yz
    code += "\n\
    circle_point.pose.position.z = centerA + radius * math.cos(th)\n\
    circle_point.pose.position.y = centerB + radius * math.sin(th)\n\
";
  } else { // xz
    code += "\n\
    circle_point.pose.position.z = centerA + radius * math.cos(th)\n\
    circle_point.pose.position.x = centerB + radius * math.sin(th)\n\
";
  }
  code += "\
    wpose = deepcopy(circle_point.pose)\n\
    waypoints.append(deepcopy(wpose))\n\
\n\
fraction = 0.0   #路径规划覆盖率\n\
maxtries = " + text_max_plan_times + "   #最大尝试规划次数\n\
attempts = 0     #已经尝试规划次数\n\
\n\
# 设置机器臂当前的状态作为运动初始状态\n\
arm.set_start_state_to_current_state()\n\
\n\
# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点\n\
while fraction < 1.0 and attempts < maxtries:\n\
    (plan, fraction) = arm.compute_cartesian_path (\n\
                            waypoints,   # waypoint poses，路点列表\n\
                            0.01,        # eef_step，终端步进值\n\
                            0.0,         # jump_threshold，跳跃阈值\n\
                            True)        # avoid_collisions，避障规划\n\
    # 尝试次数累加\n\
    attempts += 1\n\
    # 打印运动规划进程\n\
    if attempts % 10 == 0:\n\
        rospy.loginfo(\"Still trying after \" + str(attempts) + \" attempts...\")\n\
\n\
# 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动\n\
if fraction == 1.0:\n\
    rospy.loginfo(\"Path computed successfully. Moving the arm.\")\n\
";
  if (checkbox_traj_execute) {
    code += "\
    arm.execute(plan)\n";
  }
  code += "\
# 如果路径规划失败，则打印失败信息\n\
else:\n\
    rospy.loginfo(\"Path planning failed with only \" + str(fraction) + \" success after \" + str(maxtries) + \" attempts.\")\n\
";

  return code;
};