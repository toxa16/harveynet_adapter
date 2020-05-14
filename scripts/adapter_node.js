#!/usr/bin/env node

const rosnodejs = require('rosnodejs');
const Pusher = require('pusher-js');


const appKey = 'fa20e14745781ba145ef';
const cluster = 'eu';
const authEndpoint =
	'https://us-central1-harveynet-dcfee.cloudfunctions.net/pusherAuthMachine';

const machineId = 'machine1';		// authorize as "machine1"


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


// init adapter node
rosnodejs.initNode('/adapter')
	.then(() => {		
		const logInterval = 1000;
		let logAllowed = true;
		setInterval(() => {
			logAllowed = true;
		}, logInterval);
		const nh = rosnodejs.nh;
		const sub = nh.subscribe('/odom', 'nav_msgs/Odometry', (msg) => {
			const { x, y } = msg.pose.pose.position; 
			console.log(`x: ${x}, y: ${y}`);
			if (logAllowed) {
				channel.trigger('client-set-coordinates', { x, y });
				logAllowed = false;
			}
		});

		const linearSpeed = 0.5;
		const angularSpeed = 1;
		const commandObj = {
			lx: 0,
			az: 0,
		}
		channel.bind('client-move-command', command => {
			const { l, a } = command;
			commandObj.lx = l * linearSpeed;
			commandObj.az = a * angularSpeed;
		});

		const pub = nh.advertise('/cmd_vel_mux/input/teleop', 'geometry_msgs/Twist');
		setInterval(() => {
			const { lx, az } = commandObj;		
			const msg = {
				linear: { x: lx, y: 0, z: 0 },
				angular: { x: 0, y: 0, z: az },
			};
			pub.publish(msg);
		}, 10);
	});
