'use strict';

goog.provide('Blockly.Python.probot');
goog.require('Blockly.Python');

Blockly.Blocks['arm_init'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("机械臂初始化：");
    this.appendDummyInput()
        .appendField("    设置")
        .appendField(new Blockly.FieldDropdown([["真机","0"], ["仿真","1"]]), "mode")
        .appendField("环境");
    this.appendDummyInput()
        .appendField("    机械臂控制使能")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "enable");
    this.setNextStatement(true, null);
    this.setColour(160);
 this.setTooltip("初始化机械臂及其运动环境");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['arm_deinit'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("机械臂运行环境卸载：");
    this.appendDummyInput()
        .appendField("    停止运动")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "stop")
        .appendField(" 并回到初始位姿")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "gohome");
    this.appendDummyInput()
        .appendField("    机械臂控制除能")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "disable");
    this.setPreviousStatement(true, null);
    this.setColour(160);
 this.setTooltip("初始化机械臂及其运动环境");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['jog_joint'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("关节运动")
        .appendField("joint1")
        .appendField(new Blockly.FieldTextInput("0"), "joint1")
        .appendField("°")
        .appendField("joint2")
        .appendField(new Blockly.FieldTextInput("0"), "joint2")
        .appendField("°")
        .appendField("joint3")
        .appendField(new Blockly.FieldTextInput("0"), "joint3")
        .appendField("°")
        .appendField("joint4")
        .appendField(new Blockly.FieldTextInput("0"), "joint4")
        .appendField("°")
        .appendField("joint5")
        .appendField(new Blockly.FieldTextInput("0"), "joint5")
        .appendField("°")
        .appendField("joint6")
        .appendField(new Blockly.FieldTextInput("0"), "joint6")
        .appendField("°");
    this.appendDummyInput()
        .appendField("执行")
        .appendField("轨迹运动")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "traj_execute");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(210);
 this.setTooltip("关节空间点动");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['go_home'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("机械臂运动回零位");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(210);
 this.setTooltip("机械臂所有关节回到零角度位姿");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_max_velocity_scaling_factor'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("设置运动速度系数")
        .appendField(new Blockly.FieldTextInput("50"), "velocity_scale")
        .appendField("%");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(260);
 this.setTooltip("设置运动速度系数");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_max_acceleration_scaling_factor'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("设置运动加速度系数")
        .appendField(new Blockly.FieldTextInput("50"), "acceleration_scale")
        .appendField("%");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(260);
 this.setTooltip("设置运动加速度系数");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['sleep_s'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("停留")
        .appendField(new Blockly.FieldTextInput("1"), "time_sleep_s")
        .appendField("s");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(210);
 this.setTooltip("停留不动(s)");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_goal_joint_tolerance'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("设置关节运动的允许误差")
        .appendField(new Blockly.FieldAngle(1), "goal_joint_tolerance");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(260);
 this.setTooltip("设置关节运动的允许误差");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['jog_pose'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("空间运动(四元数)")
        .appendField("X")
        .appendField(new Blockly.FieldTextInput("0"), "pos_x")
        .appendField("m")
        .appendField("Y")
        .appendField(new Blockly.FieldTextInput("0"), "pos_y")
        .appendField("m")
        .appendField("Z")
        .appendField(new Blockly.FieldTextInput("0"), "pos_z")
        .appendField("m")
        .appendField("W")
        .appendField(new Blockly.FieldTextInput("1"), "pos_w");
    this.appendDummyInput()
        .appendField("执行")
        .appendField("轨迹规划")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "traj_plan")
        .appendField("轨迹运动")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "traj_execute");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(210);
 this.setTooltip("空间运动");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['jog_pose_rpy'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("空间运动")
        .appendField("X")
        .appendField(new Blockly.FieldTextInput("0"), "pos_x")
        .appendField("m")
        .appendField("Y")
        .appendField(new Blockly.FieldTextInput("0"), "pos_y")
        .appendField("m")
        .appendField("Z")
        .appendField(new Blockly.FieldTextInput("0"), "pos_z")
        .appendField("m")
        .appendField("Roll")
        .appendField(new Blockly.FieldTextInput("0"), "angle_roll")
        .appendField("°")
        .appendField("Pitch")
        .appendField(new Blockly.FieldTextInput("0"), "angle_pitch")
        .appendField("°")
        .appendField("Yaw")
        .appendField(new Blockly.FieldTextInput("0"), "angle_yaw")
        .appendField("°");
    this.appendDummyInput()
        .appendField("执行")
        .appendField("轨迹规划")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "traj_plan")
        .appendField("轨迹运动")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "traj_execute");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(210);
 this.setTooltip("空间运动");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_allow_replanning'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("设置空间运动是否允许多次轨迹规划")
        .appendField("允许")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "allow_replanning");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(260);
 this.setTooltip("运动规划失败时，是否允许重新规划");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_goal_position_tolerance'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("设置空间运动的目标位置允许误差")
        .appendField(new Blockly.FieldTextInput("0.001"), "goal_position_tolerance")
        .appendField("m");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(260);
 this.setTooltip("设置空间运动的目标位置允许误差");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_goal_orientation_tolerance'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("设置空间运动的目标姿态允许误差")
        .appendField(new Blockly.FieldAngle(1), "goal_orientation_tolerance");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(260);
 this.setTooltip("设置空间运动的目标姿态允许误差");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_digital_output'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("设置控制器数字IO")
        .appendField("OUTPUT")
        .appendField(new Blockly.FieldTextInput("1"), "io_num")
        .appendField("为")
        .appendField(new Blockly.FieldDropdown([["高电平","1"], ["低电平","0"]]), "io_level");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(310);
 this.setTooltip("设置控制器数字IO电平输出");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['get_digital_input'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("读取控制器数字IO")
        .appendField("INPUT")
        .appendField(new Blockly.FieldTextInput("1"), "io_num");
    this.setOutput(true, 'Number');
    this.setColour(310);
 this.setTooltip("读取控制器数字IO");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['if_digital_input'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("如果")
        .appendField("读取控制器数字IO")
        .appendField("INPUT")
        .appendField(new Blockly.FieldTextInput("1"), "io_num")
        .appendField("为")
        .appendField(new Blockly.FieldDropdown([["高电平","1"], ["低电平","0"]]), "io_level");
    this.appendStatementInput("NAME")
        .setCheck(null)
        .appendField("则执行");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(310);
 this.setTooltip("读取控制器数字IO");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['digital_level'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldDropdown([["高电平","1"], ["低电平","0"]]), "level");
    this.setOutput(true, "Number");
    this.setColour(310);
 this.setTooltip("控制器数字IO电平值");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['move_in_line'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("笛卡尔直线运动")
        .appendField("沿")
        .appendField(new Blockly.FieldDropdown([["x","0"], ["y","1"], ["z","2"]]), "xyz_direction")
        .appendField("轴")
        .appendField(new Blockly.FieldDropdown([["正方向","0"], ["反方向","1"]]), "direction")
        .appendField("运动")
        .appendField(new Blockly.FieldTextInput("0.1"), "distance")
        .appendField("m");
    this.appendDummyInput()
        .appendField("    路径点间隔")
        .appendField(new Blockly.FieldTextInput("0.01"), "step_m")
        .appendField("m")
        .appendField("最大轨迹规划尝试次数")
        .appendField(new Blockly.FieldTextInput("20"), "max_plan_times")
        .appendField("次");
    this.appendDummyInput()
        .appendField("    执行")
        .appendField("轨迹运动")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "traj_execute");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(210);
 this.setTooltip("笛卡尔直线运动");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['move_in_circle'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("笛卡尔圆弧运动");
    this.appendDummyInput()
        .appendField("    以当前终端位姿为圆心 以")
        .appendField(new Blockly.FieldTextInput("0.1"), "circle_r")
        .appendField("m为半径")
        .appendField("沿")
        .appendField(new Blockly.FieldDropdown([["xy","0"], ["yz","1"], ["xz","2"]]), "xyz_direction")
        .appendField("平面")
        .appendField("转动")
        .appendField(new Blockly.FieldAngle(10), "angle_move");
    this.appendDummyInput()
        .appendField("    路径点间隔")
        .appendField(new Blockly.FieldTextInput("0.02"), "step_m")
        .appendField("m")
        .appendField("最大轨迹规划尝试次数")
        .appendField(new Blockly.FieldTextInput("100"), "max_plan_times")
        .appendField("次");
    this.appendDummyInput()
        .appendField("    执行")
        .appendField("轨迹运动")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "traj_execute");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(210);
 this.setTooltip("笛卡尔直线运动");
 this.setHelpUrl("");
  }
};