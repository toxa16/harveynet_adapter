#!/usr/bin/env node

const rosnodejs = require('rosnodejs');
const Pusher = require('pusher-js');


const appKey = 'fa20e14745781ba145ef';
const cluster = 'eu';
const authEndpoint =
	'https://harveynet-ownership-server.herokuapp.com/pusher/auth/machine';

const machineId = 'machine1';		// authorize as "machine1"


// pusher message chunking (copy-pasted)
function triggerChunked(channel, event, data) {
  const chunkSize = 9999;
  const str = JSON.stringify(data);
  const msgId = Math.random() + '';
  for (var i = 0; i*chunkSize < str.length; i++) {
    // TODO: use pusher.triggerBatch for better performance
    channel.trigger(event + '-chunked', { 
      id: msgId, 
      index: i, 
      chunk: str.substr(i*chunkSize, chunkSize), 
      final: chunkSize*(i+1) >= str.length
    });
  }
}


// init pusher & presence channel
const pusher = new Pusher(appKey, {
	cluster,
	authEndpoint,
	auth: {
		params: { machineId },
	},
});
const channelName = `presence-${machineId}`;
const channel = pusher.subscribe(channelName);

const controlChannelName = `presence-control-${machineId}`;
const controlChannel = pusher.subscribe(controlChannelName);


// odometry & simulated GPS
function odomProcedure(nh) {
	const odomInterval = 500;
	let odomTriggerAllowed = true;
	setInterval(() => {
		odomTriggerAllowed = true;
	}, odomInterval);
	nh.subscribe('/odom', 'nav_msgs/Odometry', msg => {
		const { x, y } = msg.pose.pose.position;
		console.log(`x: ${x}, y: ${y}`);
		if (odomTriggerAllowed) {
			odomTriggerAllowed = false;

			// simulating GPS coords
			const baseLat = 50.422;
			const baseLng = 29.973;
			const mLat = 5 / 10;
			const mLng = 8 / 10;
			const latitude = baseLat + y * 0.001 * mLat;
			const longitude = baseLng + x * 0.001 * mLng;
			channel.trigger('client-set-navsat', { latitude, longitude });
			channel.trigger('client-set-coordinates', { x, y });
		}
	});
}


// camera
function cameraProcedure(nh) {
	const cameraTriggerInterval = 200;
	let cameraTriggerAllowed = true;
	setInterval(() => {
		cameraTriggerAllowed = true;
	}, cameraTriggerInterval);
	nh.subscribe(
		'/camera/rgb/image_raw/compressed',
		'sensor_msgs/CompressedImage',
		msg => {
			if (cameraTriggerAllowed) {
				const _image = Buffer.from(msg.data).toString('base64');
				const image = 'data:image/jpg;base64,' + _image;
				triggerChunked(channel, 'client-set-camera-image', { image });
				cameraTriggerAllowed = false;
			}
		},
	);
}


// movement control
function moveControlProcedure(nh) {
	const linearSpeed = 0.5;
	const angularSpeed = 1;
	const commandObj = {
		lx: 0,
		az: 0,
	}
	controlChannel.bind('client-move-command', command => {
		const { l, a } = command;
		commandObj.lx = l * linearSpeed;
		commandObj.az = a * angularSpeed;
	});
	let controlTopic = '/cmd_vel';		// TurtleBot3
	if (process.env.TURTLEBOT2) {
		controlTopic = '/cmd_vel_mux/input/teleop';	// TurtleBot2
	}
	const pub = nh.advertise(controlTopic, 'geometry_msgs/Twist');
	setInterval(() => {
		const { lx, az } = commandObj;		
		const msg = {
			linear: { x: lx, y: 0, z: 0 },
			angular: { x: 0, y: 0, z: az },
		};
		pub.publish(msg);
	}, 10);
}


// tool control
function openToolControlTopic(nh, msgFormat, topic) {
	const pub = nh.advertise(`/harvey/control/${topic}`, msgFormat);
	controlChannel.bind(`client-tool-${topic}`, command => {
		const msg = { data: command.value };
		pub.publish(msg);
	});
}
function toolControlProcedure(nh) {
	openToolControlTopic(nh, 'std_msgs/Bool', 'binary_0');
	openToolControlTopic(nh, 'std_msgs/Bool', 'binary_1');
}


// init adapter node
rosnodejs.initNode('/adapter')
	.then(() => {
		const nh = rosnodejs.nh;

		// you can disable a feature by commenting out correspondind line below
		odomProcedure(nh);
		cameraProcedure(nh);
		moveControlProcedure(nh);
		toolControlProcedure(nh);
	});
