const urlROSBRIDGE = 'ws://localhost:9090';

// Connect to ROSBridge
const ros = new ROSLIB.Ros({
    url: urlROSBRIDGE
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
    document.getElementById('connection-status').textContent = 'Connected to ROS2';
    document.getElementById('connection-status').style.color = 'green';
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    document.getElementById('connection-status').textContent = 'Connection Error';
    document.getElementById('connection-status').style.color = 'red';
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    document.getElementById('connection-status').textContent = 'Disconnected';
    document.getElementById('connection-status').style.color = 'orange';
});

// Initialize plots
const poseTrace = {
    x: [],
    y: [],
    mode: 'lines+markers',
    type: 'scatter',
    name: 'Vehicle Position'
};

const poseLayout = {
    title: 'Vehicle Position',
    xaxis: { title: 'X Position', range: [-5, 5] },
    yaxis: { title: 'Y Position', range: [-5, 5] }
};

Plotly.newPlot('pose-plot', [poseTrace], poseLayout);

const lidarTrace = {
    x: [],
    y: [],
    mode: 'markers',
    type: 'scatter',
    name: 'LIDAR Points',
    marker: { size: 2, color: 'blue' }
};

const lidarLayout = {
    title: 'LIDAR Scan',
    xaxis: { title: 'X (m)', range: [-10, 10] },
    yaxis: { title: 'Y (m)', range: [-10, 10] }
};

Plotly.newPlot('lidar-plot', [lidarTrace], lidarLayout);

// Subscribe to pose topic with timestamp checking
let lastPoseTime = 0;
const POSE_THROTTLE = 100; // Update every 100ms (10 FPS)

// Keep track of pose data in memory
let poseData = { x: [], y: [] };
const MAX_POSE_POINTS = 100;

const poseListener = new ROSLIB.Topic({
    ros: ros,
    name: '/ekf_pose',
    messageType: 'geometry_msgs/PoseStamped'
});

poseListener.subscribe(function(message) {
    // Get timestamp from message header
    const messageTime = message.header.stamp.sec * 1000 + message.header.stamp.nanosec / 1000000;
    const currentTime = Date.now();
    
    // Skip if message is too old
    const maxAge = 200; // milliseconds
    if (currentTime - messageTime > maxAge) {
        // console.log(`Skipping old pose message: ${currentTime - messageTime}ms old`);
        return;
    }
    
    // Throttle pose updates
    if (currentTime - lastPoseTime < POSE_THROTTLE) {
        return;
    }
    
    lastPoseTime = currentTime;
    // console.log('Received pose:', message.pose.position);
    
    // Add new point
    poseData.x.push(message.pose.position.x);
    poseData.y.push(message.pose.position.y);
    
    // Remove old points if exceeding limit
    if (poseData.x.length > MAX_POSE_POINTS) {
        poseData.x.shift(); // Remove first element
        poseData.y.shift(); // Remove first element
    }
    
    // Update plot with current data
    Plotly.restyle('pose-plot', {
        x: [poseData.x],
        y: [poseData.y]
    }, [0]);
});

let lastDisplayTime = 0;
const TARGET_FPS = 20; // Target 20 FPS for LIDAR display
const FRAME_INTERVAL = 1000 / TARGET_FPS; // 50ms between frames

const scanListener = new ROSLIB.Topic({
    ros: ros,
    name: '/scan',
    messageType: 'sensor_msgs/LaserScan'
});

scanListener.subscribe(function(message) {
    // Get timestamp from message header
    const messageTime = message.header.stamp.sec * 1000 + message.header.stamp.nanosec / 1000000;
    const currentTime = Date.now();
    
    // Skip if message is too old
    const maxAge = 100; // milliseconds
    if (currentTime - messageTime > maxAge) {
        // console.log(`Skipping old LIDAR message: ${currentTime - messageTime}ms old`);
        return;
    }
    
    // Throttle display updates to target FPS
    if (currentTime - lastDisplayTime < FRAME_INTERVAL) {
        return;
    }
    
    lastDisplayTime = currentTime;
    
    // Process all LIDAR points (no downsampling)
    const ranges = [...message.ranges].reverse();
    const validX = [];
    const validY = [];
    
    for (let i = 0; i < ranges.length; i++) {
        const range = ranges[i];
        if (range > 0.1 && range < 20 && isFinite(range)) {
            const angle = message.angle_min + (message.angle_max - message.angle_min) * i / (ranges.length - 1);
            const normalizedAngle = (angle + Math.PI) % (2 * Math.PI);
            
            validX.push(Math.sin(normalizedAngle) * range);
            validY.push(Math.cos(normalizedAngle) * range);
        }
    }
    
    if (validX.length > 0) {
        Plotly.restyle('lidar-plot', {
            x: [validX],
            y: [validY]
        }, [0]);
    }
});