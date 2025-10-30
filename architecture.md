## Pilote et structure

### URDF/XACRO

### publication des transformations

### pilote du materiel

lidar2D
depth camera
camera pince
IMU

### Contrôle
ros2_control avec Hardware interface

controller_manager
mecanum_drive_controller

### Odométrie

## SLAM

### Cartographie 2D
    slam_toolbox

### Cartographie 3D

## Navigation

## manipulation

```mermaid
graph TD
    subgraph "Niveau 1: Matériel & Pilotes"
        Lidar[LiDAR 2D] -->|/scan| SLAM_2D
        DepthCam[Depth Camera] -->|/depth/points, /color/image| SLAM_3D
        DepthCam -->|/depth/points| Nav_Costmap
        GripperCam[Gripper Camera] -->|/gripper_camera/image| Perception
        IMU[IMU] -->|/imu/data| EKF
        Chassis[Moteurs Châssis] <==> |/cmd_vel, /joint_states/| ROS2_Control
        Arm[Moteurs Bras] <==> |/joint_trajectory, /joint_states/| ROS2_Control
    end

    subgraph "Niveau 2: Fondamentaux ROS"
        ROS2_Control -->|/joint_states| RobotStatePub
        ROS2_Control -->|Odom Roues| EKF[robot_localization EKF]
        URDF[Fichier URDF] --> RobotStatePub[robot_state_publisher]
        RobotStatePub -->|/tf| Global
        EKF -->|/odometry/filtered /tf odom->base_link| Global(Tous les Nœuds)
    end

    subgraph "Niveau 3: Applications"
        
        subgraph "SLAM"
            SLAM_2D[slam_toolbox] -->|/map| Nav2
            SLAM_3D[rtabmap_ros] -->|/map, /rtabmap/cloud_map| Nav2
        end

        subgraph "Navigation (Nav2)"
            Nav2[Nav2 Stack]
            Nav_Costmap[Costmaps 2D] --> Nav2
            Lidar --> Nav_Costmap
            Nav2 -->|/cmd_vel| ROS2_Control
        end
        subgraph "Manipulation (MoveIt)"
            MoveIt[MoveIt 2] -->|/joint_trajectory| ROS2_Control
            Perception[Perception YOLO, AprilTag] -->|Pose Objet| GraspLogic
            GraspLogic[Logique de Saisie] --> MoveIt
        end
    end
```