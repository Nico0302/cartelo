# Cartelo

Teleoperate ROS 2 robot arms using cartesian coordinate inputs from 3D trackers (VR controllers, smartphones, etc.).

> [!WARNING]
> This project is still under construction 🚧
>
> My goal is to create a LeRobot fine tuning dataset using an UR12e arm and a HTC Vive controller. If you are interested in combining LeRobot with ROS 2 check out [LeROS2](https://github.com/Nico0302/leros2).

## Usage

```mermaid
flowchart LR
    i([Input])
    c([Cartelo])
    ik([IK])
    j([Joints])
    i --> c --> ik --> j
```

Cartelo can take any [tf2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html) frame and an optional `Joy` topic and output a desired target pose to an inverse kinematic node (like MoveIt2 Servo, Cartesian Controllers, CRISP Controllers, etc.).

To start teleportation, the operator can move the 3D tracking device close to the robot base and press the preconfigured calibration button (i.e. menu button). 
Now all movements will be transferred to the robot as long as the teleportation button (i.e. side grippers) is pressed down.

## HTC VIVE <> SO-101 Setup

```bash
sudo curl -fsSL https://raw.githubusercontent.com/cntools/libsurvive/master/useful_files/81-vive.rules \
    -o /etc/udev/rules.d/81-vive.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

```bash
lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=so101_follower
```

```bash
ros2 launch so101_description bringup.launch.py usb_port:=/dev/ttyACM1
```

```bash
ros2 launch cartesian_teleoperation vive.launch.py
```

```bash
ros2 launch cartesian_teleoperation teleop.launch.py
```

```bash
ros2 launch so101_moveit_config moveit.launch.py use_fake_hardware:=true
```

## X11Forwarding

```bash
xhost +localhost
```
