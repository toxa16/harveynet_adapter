# harveynet-adapter

A ROS package that connects a ROS TurtleBot to the HarveyNet.

This adapter package (node) will authorize the TurtleBot at the HarveyNet as `machine1`, owned by user `alice@email.com` by default (the machine ID is *hard-coded*).

## Installation and running

1. Clone the package repository under `<CATKIN_WS_DIR>/src`.

```bash
$ cd <CATKIN_WS_DIR>/src
$ git clone <THIS_REPO_URL>
$ cd harveynet_adapter
```

2. Install dependencies:

```bash
$ npm install
```

3. Source the setup file:

```bash
$ . ~/catkin_ws/devel/setup.bash
```

4. Launch the TurtleBot Gazebo:

```bash
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```

5. In a new terminal, run the adapter node:

```
$ rosrun harveynet_adapter adapter_node.js
```

Now the TurtleBot machine is connected to the HarveyNet.

6. Open the HarveyNet Control Panel in your web browser: https://harveynet-control-panel.herokuapp.com

7. Login as `alice@email.com`

8. Now you can control the Turtlebot from the HarveyNet.

9. (Optionally) Kill the adapter node process to see the machine going offline at the Control Panel.
