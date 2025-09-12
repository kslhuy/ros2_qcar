// Plot management functions

function initializePosePlot() {
    if (!poseListener && ros && ros.isConnected) {
        // Get topic from settings
        const poseTopic = localStorage.getItem('pose-topic') || CONFIG.TOPICS.POSE;
        
        poseListener = new ROSLIB.Topic({
            ros: ros,
            name: poseTopic,
            messageType: 'geometry_msgs/PoseStamped'
        });

        // Get throttle setting
        const poseThrottle = parseInt(localStorage.getItem('pose-throttle')) || CONFIG.POSE_THROTTLE;

        poseListener.subscribe(function(message) {
            const currentTime = Date.now();
            
            if (currentTime - lastPoseTime < poseThrottle) {
                return;
            }
            
            lastPoseTime = currentTime;
            
            if (document.getElementById('pose-plot')) {
                Plotly.extendTraces('pose-plot', {
                    x: [[message.pose.position.x]],
                    y: [[message.pose.position.y]]
                }, [0]);
            }
        });
    }
}

function initializeLidarPlot() {
    if (!scanListener && ros && ros.isConnected) {
        // Get topic from settings
        const lidarTopic = localStorage.getItem('lidar-topic') || CONFIG.TOPICS.LIDAR;
        
        scanListener = new ROSLIB.Topic({
            ros: ros,
            name: lidarTopic,
            messageType: 'sensor_msgs/LaserScan'
        });

        // Get FPS setting
        const targetFps = parseInt(localStorage.getItem('lidar-fps')) || CONFIG.TARGET_FPS;
        const frameInterval = 1000 / targetFps;

        scanListener.subscribe(function(message) {
            const currentTime = Date.now();
            
            if (currentTime - lastDisplayTime < frameInterval) {
                return;
            }
            
            lastDisplayTime = currentTime;
            
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
            
            if (validX.length > 0 && document.getElementById('lidar-plot')) {
                Plotly.restyle('lidar-plot', {
                    x: [validX],
                    y: [validY]
                }, [0]);
            }
        });
    }
}

function initializeOccupancyPlot() {
    if (!occupancyListener && ros && ros.isConnected) {
        // Get topic from settings
        const occupancyTopic = localStorage.getItem('occupancy-topic') || CONFIG.TOPICS.OCCUPANCY_GRID;
        
        occupancyListener = new ROSLIB.Topic({
            ros: ros,
            name: occupancyTopic,
            messageType: 'nav_msgs/OccupancyGrid'
        });

        occupancyListener.subscribe(function(message) {
            const width = message.info.width;
            const height = message.info.height;
            const data = message.data;
            
            const grid = [];
            for (let i = 0; i < height; i++) {
                const row = [];
                for (let j = 0; j < width; j++) {
                    const value = data[i * width + j];
                    if (value === -1) {
                        row.push(0.5); // Unknown
                    } else {
                        row.push(value / 100.0); // 0=free, 1=occupied
                    }
                }
                grid.push(row);
            }
            
            if (document.getElementById('occupancy-plot')) {
                Plotly.restyle('occupancy-plot', {
                    z: [grid]
                }, [0]);
            }
        });
    }
}

function initializePlotlyGraphs() {
    // Get plot ranges from settings
    const poseRange = parseFloat(localStorage.getItem('pose-range')) || Math.abs(CONFIG.POSE_RANGE[0]);
    const lidarRange = parseFloat(localStorage.getItem('lidar-range')) || Math.abs(CONFIG.LIDAR_RANGE[0]);
    
    // Initialize pose plot
    const poseTrace = {
        x: [],
        y: [],
        mode: 'lines+markers',
        type: 'scatter',
        name: 'Vehicle Position'
    };

    const poseLayout = {
        title: 'Vehicle Position',
        xaxis: { title: 'X Position', range: [-poseRange, poseRange] },
        yaxis: { title: 'Y Position', range: [-poseRange, poseRange] }
    };

    if (document.getElementById('pose-plot')) {
        Plotly.newPlot('pose-plot', [poseTrace], poseLayout);
    }

    // Initialize LIDAR plot
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
        xaxis: { title: 'X (m)', range: [-lidarRange, lidarRange] },
        yaxis: { title: 'Y (m)', range: [-lidarRange, lidarRange] }
    };

    if (document.getElementById('lidar-plot')) {
        Plotly.newPlot('lidar-plot', [lidarTrace], lidarLayout);
    }

    // Initialize occupancy grid plot
    const occupancyTrace = {
        z: [],
        type: 'heatmap',
        colorscale: [
            [0, 'white'],
            [0.5, 'gray'],
            [1, 'black']
        ],
        showscale: false
    };

    const occupancyLayout = {
        title: 'Occupancy Grid',
        xaxis: { title: 'X (cells)' },
        yaxis: { title: 'Y (cells)' }
    };

    if (document.getElementById('occupancy-plot')) {
        Plotly.newPlot('occupancy-plot', [occupancyTrace], occupancyLayout);
    }
}