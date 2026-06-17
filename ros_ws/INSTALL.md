# Lab PC Setup (ROS 1 Noetic + Gazebo Classic)

## Isolation principle

The lab PC is shared with other users. This setup is designed to leave
their environments untouched:

- All project files live under `~/Desktop/anurag_ws/`. No files are
  written elsewhere except apt-installed packages (which are checked
  for first and only installed if missing).
- The catkin workspace for this project is
  `~/Desktop/anurag_ws/mobile-grasping/ros_ws/`. The system or
  per-user `~/catkin_ws/` is **never** touched.
- Python packages are installed with `pip3 install --user`, which
  writes to `~/.local/`. Per-user, isolated from other lab users and
  from the system Python.
- Nothing is added to `~/.bashrc`. Sourcing this workspace's
  `devel/setup.bash` is per-terminal, opt-in.

The only operation that touches global state is `sudo apt install` for
any missing TurtleBot 3 / OpenManipulator-X apt packages. Each step
below checks first and only installs what is missing. If even this is
unacceptable for the shared machine, every apt-only dependency can be
replaced by a source build inside our workspace; see
"Avoiding sudo entirely" at the end.

---

## 0. Pre-flight: confirm what's already on the machine

Run these once and note the output:

```bash
# ROS distro
echo "ROS_DISTRO=$ROS_DISTRO"
which roscore

# Gazebo Classic
which gazebo
gazebo --version 2>&1 | head -1

# Check which TurtleBot 3 / OpenManipulator-X apt packages are present
dpkg -l 2>/dev/null | grep -E "ros-noetic-(turtlebot3|dynamixel-sdk|robotis-manipulator|open-manipulator|gazebo-ros)" | awk '{print $2}'
```

If `ROS_DISTRO` is `noetic`, `roscore` is found, and `gazebo --version`
prints 11.x, the base stack is in place. The `dpkg -l` listing tells
us which extra packages are already installed so the next step only
adds what is missing.

---

## 1. Clone the repo into the user's workspace folder

```bash
mkdir -p ~/Desktop/anurag_ws
cd ~/Desktop/anurag_ws
git clone https://github.com/orionop/mobile-grasping.git
cd mobile-grasping
```

After this, the project lives at
`~/Desktop/anurag_ws/mobile-grasping/`, and the catkin workspace will
be `~/Desktop/anurag_ws/mobile-grasping/ros_ws/` (already part of
the repo).

---

## 2. Install missing apt packages (only the ones not already present)

The output of the `dpkg -l ... grep` from step 0 tells us what is
present. Install only the missing ones from the list below. Run
each `apt install` individually so the others remain untouched if
one is unwanted.

Required apt packages (aligned with the official ROBOTIS
TurtleBot3 + OpenManipulator-X documentation,
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/):

| Package | Purpose |
|---|---|
| `ros-noetic-turtlebot3` | TB3 meta package |
| `ros-noetic-turtlebot3-msgs` | TB3 custom messages |
| `ros-noetic-turtlebot3-simulations` | TB3 Gazebo worlds and models |
| `ros-noetic-turtlebot3-bringup` | TB3 launch files |
| `ros-noetic-dynamixel-sdk` | Dynamixel motor SDK (OMX-X) |
| `ros-noetic-robotis-manipulator` | ROBOTIS manipulator base classes |
| `ros-noetic-gazebo-ros-pkgs` | Gazebo Classic ROS integration |
| `ros-noetic-gazebo-ros-control` | Gazebo controller plugin |
| `ros-noetic-ros-control*` | ros_control framework (umbrella, per ROBOTIS docs) |
| `ros-noetic-control*` | Standard controllers (umbrella, per ROBOTIS docs) |
| `ros-noetic-moveit*` | MoveIt for the arm (used by `turtlebot3_manipulation_moveit_config`) |
| `ros-noetic-dwa-local-planner` | Local planner used by TB3 navigation |
| `ros-noetic-xacro` | URDF macro processor |

To install only the missing ones, for each package not in your
`dpkg -l` output:

```bash
sudo apt install -y <package-name>
```

Do **not** run `sudo apt upgrade` or `sudo apt dist-upgrade`. Those
would update unrelated packages on the shared machine.

---

## 3. Clone source dependencies into our workspace's src/

The combined `turtlebot3_manipulation_*` packages for ROS Noetic are
not in apt; they have to be built from ROBOTIS-GIT source. We clone
them into our project's `ros_ws/src/`, not into any other catkin
workspace.

```bash
cd ~/Desktop/anurag_ws/mobile-grasping/ros_ws/src/

git clone -b noetic \
  https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone -b noetic \
  https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
```

The branch name is `noetic` (per the official ROBOTIS docs), not
`noetic-devel`. Confirmed against
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/.

Now `ros_ws/src/` contains:

```
mobile_grasping_ros/                          (ours)
turtlebot3_manipulation/                      (from ROBOTIS-GIT)
turtlebot3_manipulation_simulations/          (from ROBOTIS-GIT)
open_manipulator_dependencies/                (from ROBOTIS-GIT)
```

Optionally resolve any remaining ROS dependencies these packages
declare, scoped to our workspace:

```bash
cd ~/Desktop/anurag_ws/mobile-grasping/ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

(Skip if `rosdep` reports everything is already satisfied.)

---

## 4. Install Python dependencies with --user

The QP solver inside the ROS package needs `qpsolvers` and `osqp`.
These are not in apt; install them with `pip3 --user`, which writes
to `~/.local/lib/python3.8/site-packages/` (per-user, not system,
not other lab users).

```bash
pip3 install --user qpsolvers osqp
```

**Do not** add `numpy<2` or any numpy version constraint here. The
system numpy that ships with Ubuntu 20.04 and ROS Noetic is 1.17.x,
which is compatible with all of our dependencies. Constraining numpy
at `--user` level could shadow the system numpy for other Python
projects in this user's home directory.

Verify:

```bash
python3 -c "import qpsolvers; print(qpsolvers.__version__)"
python3 -c "import osqp; print('OK')"
```

---

## 5. Build only this workspace

```bash
cd ~/Desktop/anurag_ws/mobile-grasping/ros_ws
catkin_make
```

This builds only the packages under our `src/`. It does not touch
`~/catkin_ws/` or any other workspace on the machine.

---

## 6. Source only this workspace per terminal

`~/.bashrc` is **not modified**. Each terminal that wants to work on
this project sources our `devel/setup.bash` manually:

```bash
source /opt/ros/noetic/setup.bash
source ~/Desktop/anurag_ws/mobile-grasping/ros_ws/devel/setup.bash
```

When the same terminal needs to work on a different project later
(for example, the IITB refueling work in a different catkin workspace),
open a fresh terminal and source that project's `setup.bash` instead.
Multiple workspaces never overlap because each terminal sources
exactly one.

Verify the workspace is active:

```bash
rospack find mobile_grasping_ros          # should print our path
rospack find turtlebot3_manipulation_gazebo  # should print our path
```

---

## 7a. Verification — run the official ROBOTIS tutorial first

Before launching our own composite `sim.launch`, run the official
ROBOTIS `turtlebot3_manipulation_gazebo.launch` standalone. This
isolates "does the platform work in Gazebo?" from "does my code
launch correctly?".

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```

Expected:

- Gazebo Classic opens and spawns the TB3 Waffle Pi with the
  OpenManipulator-X mounted on top.
- The robot stands still in an empty world.
- `rostopic list` (in another terminal, with the workspace sourced)
  shows topics including `/joint_states`, `/odom`, `/cmd_vel`,
  arm joint controllers, etc.

If this works, the platform install is correct. If it fails, the
problem is in the ROBOTIS install (step 2–5), not our code. Reference:
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/

## 7b. Launch our composite simulation

Once 7a is verified, run our launch file, which wraps the same
ROBOTIS Gazebo bring-up and adds our three nodes on top:

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch mobile_grasping_ros sim.launch
```

Expected:

- Same Gazebo Classic scene as in 7a.
- Terminal additionally shows three nodes starting:
  `mobile_grasping_controller`, `mobile_grasping_predictor`,
  `mobile_grasping_estimator`.

---

## 8. Sanity checks (open another terminal, source the workspace there too)

```bash
source /opt/ros/noetic/setup.bash
source ~/Desktop/anurag_ws/mobile-grasping/ros_ws/devel/setup.bash

# Topic flow
rostopic list
rostopic hz /grasp_pose         # should be ~30 Hz
rostopic hz /base_pose          # should be ~50 Hz
rostopic hz /arm_velocity_cmd   # should be ~200 Hz

# Inspect content
rostopic echo /grasp_pose -n 1
rostopic echo /arm_velocity_cmd -n 1
```

---

## Cleanup (if anything goes wrong and we want to undo)

- The repo can be wiped with `rm -rf ~/Desktop/anurag_ws/mobile-grasping/`.
  No state outside that folder is created by steps 1, 3, 5, 6, 7, 8.
- Python `--user` packages can be removed with:
  `pip3 uninstall --user qpsolvers osqp`.
- Apt packages installed in step 2 can be removed with
  `sudo apt remove <package>`. Other lab users may have started using
  them in the meantime; check with them before removing.

---

## Avoiding sudo entirely (optional, if even apt is not acceptable)

Every package in step 2 can be built from source inside our workspace
instead of installed via apt. This is more work but touches zero
system state. If needed:

```bash
cd ~/Desktop/anurag_ws/mobile-grasping/ros_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# Plus the relevant controllers; let me know and I'll fill in the
# remaining clone URLs.
```

This route is recommended only if the lab explicitly disallows
`sudo apt install`. For most lab PCs, the additive apt installs in
step 2 are fine.

---

## Velocity interface for the arm (required for reactive control / M2)

The Holistic QP outputs joint *velocities* (Fig. 1 q_dot-cmd). The stock
`turtlebot3_manipulation` transmission declares a `PositionJointInterface`
and loads a position `JointTrajectoryController`, which cannot track a
moving setpoint (measured ~29 cm error following a 0.05 m/s target). Switch
the four arm joints to a velocity interface:

```bash
# back up first
cp ~/Desktop/anurag_ws/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/open_manipulator_x.transmission.xacro{,.bak}

# joint1..joint4 PositionJointInterface -> VelocityJointInterface
# (gripper joints use EffortJointInterface and are left untouched)
sed -i 's#PositionJointInterface#VelocityJointInterface#g' \
  ~/Desktop/anurag_ws/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/open_manipulator_x.transmission.xacro

cd ~/Desktop/anurag_ws/catkin_ws && catkin_make
```

After this, the stock position `arm_controller` fails to load (expected -
it cannot claim velocity-interface joints). `pipeline.launch` spawns our
`arm_vel_controller` (velocity_controllers/JointGroupVelocityController,
config in `config/arm_velocity_controller.yaml`) and `controller_node`
publishes joint velocities to `/arm_vel_controller/command`.

To revert to the stock position interface:
```bash
mv ~/Desktop/anurag_ws/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/open_manipulator_x.transmission.xacro{.bak,}
catkin_make
# and launch with cmd_interface:=trajectory spawn_velocity_controller:=false
```
