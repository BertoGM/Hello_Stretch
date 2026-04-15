let ros;
let cmdVel;
let headTopic;
let statusListener;

// CONNECT TO ROS
function connectROS() {
  ros = new ROSLIB.Ros({
    url: 'ws://127.0.0.1:9090'
  });

  ros.on('connection', () => {
    document.getElementById("status").innerText = "Connected";
    document.getElementById("status").className = "connected";
    console.log("Connected to ROS");

    setupTopics();
  });

  ros.on('error', (error) => {
    console.log("Error:", error);
  });

  ros.on('close', () => {
    document.getElementById("status").innerText = "Disconnected";
    document.getElementById("status").className = "disconnected";
  });
}

// SETUP TOPICS
function setupTopics() {
  // BASE MOVEMENT
  cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  });

  // HEAD CONTROL (you may need to change this topic)
  headTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/head_pan_cmd',
    messageType: 'std_msgs/Float64'
  });

  // STATUS DISPLAY (change to real topic later)
  statusListener = new ROSLIB.Topic({
    ros: ros,
    name: '/my_topic',
    messageType: 'std_msgs/String'
  });

  statusListener.subscribe((msg) => {
    document.getElementById("robotStatus").innerText = msg.data;
  });
}

// HEAD CONTROL
function sendHead() {
  const val = parseFloat(document.getElementById("headSlider").value);

  document.getElementById("headValue").innerText = val.toFixed(2);

  const msg = new ROSLIB.Message({
    data: val
  });

  headTopic.publish(msg);
}

// BASE MOVEMENT
function moveForward() {
  publishTwist(0.3, 0);
}

function moveBackward() {
  publishTwist(-0.3, 0);
}

function turnLeft() {
  publishTwist(0, 0.5);
}

function turnRight() {
  publishTwist(0, -0.5);
}

function stop() {
  publishTwist(0, 0);
}

// TWIST HELPER
function publishTwist(linear, angular) {
  const twist = new ROSLIB.Message({
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular }
  });

  cmdVel.publish(twist);
}