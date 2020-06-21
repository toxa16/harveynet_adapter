#!/usr/bin/env node

const rosnodejs = require('rosnodejs');
const Pusher = require('pusher-js');
const { RTCPeerConnection, RTCIceCandidate, RTCSessionDescription, MediaStream, nonstandard } = require('wrtc');
const { RTCVideoSource, rgbaToI420 } = nonstandard;

// utility
function delay(ms) {
  return new Promise(resolve => {
    setTimeout(resolve, ms);
  });
}

//Pusher.logToConsole = true;
var pusher = new Pusher('fa20e14745781ba145ef', {
  cluster: 'eu',
  authEndpoint: 'https://harveynet-ownership-server.herokuapp.com/pusher/auth/machine',
});
const channelName = 'private-webrtc-sandbox';
const offerEventName = 'client-offer';
const answerEventName = 'client-answer';
const iceEventName = 'client-ice';
var channel = pusher.subscribe(channelName);


const iceServers = [
  {urls:'stun:stun.rixtelecom.se'},
  {urls:'stun:stunserver.org'},
  {urls:'stun:stun.softjoys.com'},
  {urls:'stun:stun.voxgratia.org'},
];


// init peer connection
let peerConnection = new RTCPeerConnection({
  iceServers,
});

peerConnection.onicecandidate = function (e) {
  if (e.candidate) {
    //console.log('onicecandidate')
    channel.trigger(iceEventName, {
      target: 'user',
      candidate: e.candidate,
    })
  }
}

channel.bind(iceEventName, (data) => {
  //console.log('ice received')
  var candidate = new RTCIceCandidate(data.candidate);
  peerConnection.addIceCandidate(candidate)
    .catch(err => {
      console.error(err);
    });
});

channel.bind(answerEventName, async (data) => {
  console.log('on answer')
  const { sdp } = data;
  var desc = new RTCSessionDescription(sdp);
  await peerConnection.setRemoteDescription(desc);
});

// utility
function toRgbaArrayBuffer(buf) {
  var ab = new ArrayBuffer(buf.length * 4 / 3);
  var view = new Uint8Array(ab);
  let c = 0;
  for (var i = 0; i < buf.length; i+=3) {
    view[c] = buf[i + 2];     // R channed
    view[c + 1] = buf[i + 1]; // G channel
    view[c + 2] = buf[i];     // B channel
    view[c + 3] = 255;        // A channel
    c += 4;
  }
  return ab;
}
// webrtc video (nonstandard)
const source = new RTCVideoSource();
const track = source.createTrack();
const stream = new MediaStream();
stream.addTrack(track);
// camera
function cameraProcedure(nh) {
	const cameraTriggerInterval = 300;
	let cameraTriggerAllowed = true;
	/*setInterval(() => {
		cameraTriggerAllowed = true;
	}, cameraTriggerInterval);*/
	nh.subscribe(
		'/camera/rgb/image_raw',
		'sensor_msgs/Image',
		msg => {
			//if (cameraTriggerAllowed) {
        const { width, height } = msg;
        const rgbaData = toRgbaArrayBuffer(msg.data);
        const i420Data = new Uint8ClampedArray(width * height * 1.5);
        const i420Frame = { width, height, data: i420Data };
        const rgbaFrame = { width, height, data: rgbaData };
        rgbaToI420(rgbaFrame, i420Frame)
        source.onFrame(i420Frame);
        cameraTriggerAllowed = false;
			//}
		},
	);
}

async function start() {
  await rosnodejs.initNode('/webrtc');
  const nh = rosnodejs.nh;
  cameraProcedure(nh);

  await delay(1000);  // waiting for pusher to initialize
  await peerConnection.addTrack(track, stream);
  const offer = await peerConnection.createOffer();
  await peerConnection.setLocalDescription(offer);
  channel.trigger(offerEventName, {
    name: 'machine',
    target: 'user',
    sdp: peerConnection.localDescription,
  });
}

start()
  .catch(err => {
    console.error(err);
  });
