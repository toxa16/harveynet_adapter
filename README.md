# harveynet-adapter

A ROS package that connects a ROS TurtleBot to the HarveyNet.

This adapter package (node) will authorize the TurtleBot at the HarveyNet as `machine1`, owned by user `alice@email.com` by default (the machine ID is *hard-coded*).

It is assumed that you have all the ROS infrastructure installed and configured on your computer, including the TurtleBot. Also the stable version of Node.js is required.

## Installation and running

1. Clone the package repository under `<CATKIN_WS_DIR>/src`.

```bash
$ cd <CATKIN_WS_DIR>/src
$ rosdep install --from-paths src -i -y
$ git clone <THIS_REPO_URL>
$ cd harveynet_adapter
```

2. Install dependencies:

```bash
$ npm install
```

3. It may be necessary to run `catkin_make` (from corresponding location).

4. (Assuming you are still in the directory from step 1) make the `adapter_node.js` executable:

```bash
$ chmod +x ./scripts/adapter_node.js
```

5. Source the setup file:

```bash
$ . ~/catkin_ws/devel/setup.bash
```

6. Launch the TurtleBot Gazebo:

turtlebot 3:
```bash
$ TURTLEBOT3_MODEL=burger roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

7. In a new terminal, run the adapter node:

```
$ rosrun harveynet_adapter adapter_node.js
```
Now the TurtleBot machine is connected to the HarveyNet.

8. Open the HarveyNet Control Panel in your web browser: https://harveynet-control-panel.herokuapp.com

9. Login as user `alice@email.com`, select machine `machine1`.

10. Now you can control the Turtlebot from the HarveyNet.

11. (Optionally) Kill the adapter node process to see the machine going offline at the Control Panel.
