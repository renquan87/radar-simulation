# RM赛场地图仿真设置指南

## 概述

本指南将帮助您设置RoboMaster University Competition 2026 (RMUC2026)赛场地图的Gazebo仿真环境。

## 前提条件

- 已安装ROS2 Humble
- 已安装Gazebo Fortress
- 已完成RMOSS Gazebo项目的编译
- 具有sudo权限（用于安装软件包）

## 步骤1：安装STP文件转换工具

由于需要将STP格式文件转换为Gazebo可用的STL格式，我们需要安装FreeCAD：

```bash
# 更新软件包列表
sudo apt-get update

# 安装FreeCAD
sudo apt-get install -y freecad

# 验证安装
freecad --version
```

## 步骤2：转换赛场模型文件

### 2.1 转换STP到STL格式

使用FreeCAD命令行工具转换STP文件：

```bash
# 进入模型文件目录
cd /data/projects/radar/simulation/ros2_simu_ws/src/models

# 创建转换脚本
cat > convert_stp_to_stl.py << 'EOF'
import sys
import FreeCAD
import Part
import Mesh

def convert_stp_to_stl(input_file, output_file):
    try:
        # 打开STP文件
        doc = FreeCAD.openDocument(input_file)
        
        # 导出为STL
        Mesh.export([obj for obj in doc.Objects if hasattr(obj, 'Shape')], output_file)
        
        # 关闭文档
        FreeCAD.closeDocument(doc.Name)
        
        print(f"成功转换: {input_file} -> {output_file}")
        return True
    except Exception as e:
        print(f"转换失败: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("用法: python convert_stp_to_stl.py <输入文件.stp> <输出文件.stl>")
        sys.exit(1)
    
    convert_stp_to_stl(sys.argv[1], sys.argv[2])
EOF

# 转换RMUC2026.stp文件
freecad convert_stp_to_stl.py RMUC2026.stp RMUC2026.stl

# 清理临时脚本
rm convert_stp_to_stl.py
```

### 2.2 备选方案（如果FreeCAD转换失败）

如果FreeCAD转换遇到问题，可以尝试以下替代方法：

```bash
# 方法1: 使用Python OCC（如果可用）
# 需要先安装: sudo pip install pythonocc-core

# 方法2: 使用在线转换工具
# 将RMUC2026.stp上传到在线转换网站（如https://www.greentoken.de/onlineconv/）
# 下载转换后的STL文件

# 方法3: 使用其他CAD软件
# 如果您有访问其他CAD软件（如SolidWorks、Fusion 360等），可以使用它们进行转换
```

## 步骤3：创建赛场世界文件结构

### 3.1 创建目录结构

```bash
# 创建世界文件目录
mkdir -p /data/projects/radar/simulation/ros2_simu_ws/src/rmoss_gazebo/worlds

# 创建赛场模型资源目录
mkdir -p /data/projects/radar/simulation/ros2_simu_ws/src/rmoss_gz_resources/resource/models/rmuc2026_competition
```

### 3.2 移动转换后的模型文件

```bash
# 移动STL文件到资源目录
mv /data/projects/radar/simulation/ros2_simu_ws/src/models/RMUC2026.stl \
   /data/projects/radar/simulation/ros2_simu_ws/src/rmoss_gz_resources/resource/models/rmuc2026_competition/
```

## 步骤4：创建赛场模型配置文件

### 4.1 创建模型配置文件

```bash
# 创建模型配置文件
cat > /data/projects/radar/simulation/ros2_simu_ws/src/rmoss_gz_resources/resource/models/rmuc2026_competition/model.config << 'EOF'
<?xml version="1.0"?>
<model>
  <name>RMUC2026 Competition Field</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  
  <author>
    <name>RMOSS Team</name>
    <email>contact@rmoss.org</email>
  </author>
  
  <description>
    RoboMaster University Competition 2026 competition field model.
  </description>
</model>
EOF
```

### 4.2 创建模型SDF文件

```bash
# 创建模型SDF文件
cat > /data/projects/radar/simulation/ros2_simu_ws/src/rmoss_gz_resources/resource/models/rmuc2026_competition/model.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="rmuc2026_competition">
    <static>true</static>
    
    <!-- 赛场地板 -->
    <link name="field_floor">
      <pose>0 0 0 0 0 0</pose>
      
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://rmuc2026_competition/RMUC2026.stl</uri>
          </mesh>
        </geometry>
      </collision>
      
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://rmuc2026_competition/RMUC2026.stl</uri>
          </mesh>
        </geometry>
        
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
```

## 步骤5：创建比赛世界文件

### 5.1 创建世界SDF文件

```bash
# 创建比赛世界文件
cat > /data/projects/radar/simulation/ros2_simu_ws/src/rmoss_gazebo/worlds/rmuc2026_competition.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.7">
    <world name="rmuc2026_competition">
        <!-- Physics -->
        <plugin
            filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics"></plugin>
        
        <!-- Scene Broadcaster -->
        <plugin
            filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"></plugin>
        
        <!-- User Commands -->
        <plugin
            filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands"></plugin>
        
        <!-- Sensors System -->
        <plugin
            filename="libignition-gazebo-sensors-system.so"
            name="ignition::gazebo::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        
        <!-- IMU System -->
        <plugin filename="libignition-gazebo-imu-system.so"
                name="ignition::gazebo::systems::Imu"></plugin>
        
        <!-- Lighting -->
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 20 0 0 0</pose>
            <diffuse>0.9 0.9 0.9 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.3</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
        
        <!-- RMUC2026 Competition Field -->
        <include>
            <uri>model://rmuc2026_competition</uri>
            <pose>0 0 0 0 0 0</pose>
        </include>
        
        <!-- Blue Team Robot Starting Position 1 -->
        <include>
            <uri>model://rmua19_standard_robot</uri>
            <pose>-4 3 0.15 0 0 1.57</pose>
        </include>
        
        <!-- Blue Team Robot Starting Position 2 -->
        <include>
            <uri>model://rmua19_standard_robot</uri>
            <pose>-4 -3 0.15 0 0 1.57</pose>
        </include>
        
        <!-- Red Team Robot Starting Position 1 -->
        <include>
            <uri>model://rmua19_standard_robot</uri>
            <pose>4 3 0.15 0 0 -1.57</pose>
        </include>
        
        <!-- Red Team Robot Starting Position 2 -->
        <include>
            <uri>model://rmua19_standard_robot</uri>
            <pose>4 -3 0.15 0 0 -1.57</pose>
        </include>
    </world>
</sdf>
EOF
```

## 步骤6：更新资源配置

### 6.1 更新模型路径配置

```bash
# 确保模型路径在Gazebo资源路径中
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/data/projects/radar/simulation/ros2_simu_ws/src/rmoss_gz_resources/resource/models" >> ~/.bashrc

# 重新加载bash配置
source ~/.bashrc
```

## 步骤7：编译项目

```bash
# 返回工作空间根目录
cd /data/projects/radar/simulation/ros2_simu_ws

# 编译项目（包含新的模型文件）
colcon build --packages-select rmoss_gz_resources

# 如果编译成功，继续
if [ $? -eq 0 ]; then
    echo "编译成功！"
else
    echo "编译失败，请检查错误信息"
    exit 1
fi
```

## 步骤8：测试仿真

### 8.1 启动仿真环境

```bash
# 设置环境
source install/setup.bash

# 启动RMUC2026比赛仿真
ign gazebo src/rmoss_gazebo/worlds/rmuc2026_competition.sdf
```

### 8.2 验证仿真环境

仿真启动后，您应该能够看到：
1. RMUC2026比赛场地
2. 4个机器人（蓝队2个，红队2个）在各自的起始位置
3. 正常的物理仿真效果

## 步骤9：添加机器人控制（可选）

如果需要控制机器人，可以按照以下步骤：

### 9.1 启动机器人控制节点

在新的终端中：

```bash
# 终端1：机器人基础节点
cd /data/projects/radar/simulation/ros2_simu_ws
source install/setup.bash
ros2 run rmoss_gz_base rmua19_robot_base

# 终端2：底盘控制
ros2 run rmoss_gz_base test_chassis_cmd.py

# 终端3：云台控制
ros2 run rmoss_gz_base test_gimbal_cmd.py

# 终端4：射击控制
ros2 run rmoss_gz_base test_shoot_cmd.py
```

## 故障排除

### 问题1：STP转换失败

**解决方案：**
1. 确保FreeCAD正确安装
2. 尝试使用其他转换工具
3. 检查STP文件是否损坏

### 问题2：模型加载失败

**解决方案：**
1. 检查模型路径是否正确
2. 验证STL文件是否有效
3. 确保模型配置文件格式正确

### 问题3：物理仿真异常

**解决方案：**
1. 调整STL模型的缩放比例
2. 检查碰撞几何体设置
3. 调整物理引擎参数

## 下一步

1. 添加更多机器人传感器（相机、激光雷达等）
2. 配置裁判系统组件
3. 创建比赛任务脚本
4. 添加自动导航功能

## 参考资源

- [RMOSS Gazebo官方文档](https://robomaster-oss.github.io/rmoss_tutorials/#/developer_guides/rmoss_gazebo_modeling)
- [Gazebo仿真文档](https://gazebosim.org/docs/fortress)
- [FreeCAD用户手册](https://wiki.freecadweb.org/Manual)