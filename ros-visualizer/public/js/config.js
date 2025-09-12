// Global configuration and constants
const CONFIG = {
    // Timing constants
    POSE_THROTTLE: 100,
    TARGET_FPS: 20,
    FRAME_INTERVAL: 1000 / 20,
    
    // Plot settings
    POSE_RANGE: [-5, 5],
    LIDAR_RANGE: [-10, 10],
    
    // Default connection settings
    DEFAULT_IP: 'localhost',
    DEFAULT_PORT: '9090',
    
    // Default topic settings
    TOPICS: {
        POSE: '/ekf_pose',
        LIDAR: '/scan',
        OCCUPANCY_GRID: '/occupancy_grid'
    },
    
    // Storage keys
    STORAGE_KEYS: {
        ROBOT_IP: 'robot-ip',
        ROSBRIDGE_PORT: 'rosbridge-port'
    }
};

// Global variables
let ros = null;
let poseListener = null;
let scanListener = null;
let occupancyListener = null;
let isConnected = false;

// Timing variables
let lastPoseTime = 0;
let lastDisplayTime = 0;