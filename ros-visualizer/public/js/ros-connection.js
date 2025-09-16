// ROS connection management

function loadSettings() {
    const savedIp = localStorage.getItem(CONFIG.STORAGE_KEYS.ROBOT_IP) || CONFIG.DEFAULT_IP;
    const savedPort = localStorage.getItem(CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT) || CONFIG.DEFAULT_PORT;
    
    // Only update DOM elements if they exist (for main page)
    const ipElement = document.getElementById('robot-ip');
    const portElement = document.getElementById('rosbridge-port');
    
    if (ipElement) ipElement.value = savedIp;
    if (portElement) portElement.value = savedPort;
    
    updateCurrentUrl();
}

function saveSettings() {
    // Get values from DOM if available, otherwise keep current settings
    const ipElement = document.getElementById('robot-ip');
    const portElement = document.getElementById('rosbridge-port');
    
    if (ipElement && portElement) {
        const ip = ipElement.value;
        const port = portElement.value;
        
        localStorage.setItem(CONFIG.STORAGE_KEYS.ROBOT_IP, ip);
        localStorage.setItem(CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT, port);
    }
    
    updateCurrentUrl();
}

function updateCurrentUrl() {
    const ip = getStoredOrDOMValue('robot-ip', CONFIG.STORAGE_KEYS.ROBOT_IP, CONFIG.DEFAULT_IP);
    const port = getStoredOrDOMValue('rosbridge-port', CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT, CONFIG.DEFAULT_PORT);
    const url = `ws://${ip}:${port}`;
    
    const urlElement = document.getElementById('current-url');
    if (urlElement) {
        urlElement.textContent = url;
    }
    
    return url;
}

function getStoredOrDOMValue(elementId, storageKey, defaultValue) {
    // First try to get from localStorage
    const storedValue = localStorage.getItem(storageKey);
    if (storedValue) {
        return storedValue;
    }
    
    // Then try DOM element
    const element = document.getElementById(elementId);
    if (element && element.value) {
        return element.value;
    }
    
    // Finally use default
    return defaultValue;
}

function connectToRobot() {
    console.log('Attempting to connect...');
    
    if (ros && ros.isConnected) {
        ros.close();
    }
    
    // Use stored settings
    const ip = localStorage.getItem(CONFIG.STORAGE_KEYS.ROBOT_IP) || CONFIG.DEFAULT_IP;
    const port = localStorage.getItem(CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT) || CONFIG.DEFAULT_PORT;
    const url = `ws://${ip}:${port}`;
    
    console.log('Connecting to:', url);
    
    updateConnectionStatus('Connecting...', 'orange');
    
    ros = new ROSLIB.Ros({
        url: url
    });
    
    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        updateConnectionStatus(`Connected to ${ip}:${port}`, 'green');
        isConnected = true;
        
        // Store last connected timestamp
        localStorage.setItem('last-connected', new Date().toISOString());
        
        // Debug all parameters after connection
        setTimeout(() => {
            if (typeof getAllParametersOrganized === 'function') {
                getAllParametersOrganized();
            }
        }, 2000);
        
        // Initialize plots if they are enabled
        initializeEnabledPlots();
        
        // Refresh nodes if function exists
        if (typeof refreshNodes === 'function') {
            refreshNodes();
        }
    });
    
    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        updateConnectionStatus(`Connection Error: ${error.message || 'Unknown error'}`, 'red');
        isConnected = false;
    });
    
    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        updateConnectionStatus('Disconnected', 'orange');
        isConnected = false;
    });
}

function disconnectFromRobot() {
    if (ros && ros.isConnected) {
        unsubscribeAllTopics();
        ros.close();
    }
}

function unsubscribeAllTopics() {
    if (poseListener) {
        poseListener.unsubscribe();
        poseListener = null;
    }
    if (scanListener) {
        scanListener.unsubscribe();
        scanListener = null;
    }
    if (occupancyListener) {
        occupancyListener.unsubscribe();
        occupancyListener = null;
    }
}

function updateConnectionStatus(text, color) {
    const statusElement = document.getElementById('connection-status');
    if (statusElement) {
        statusElement.textContent = text;
        statusElement.style.color = color;
        
        // Update connection badge classes if exists (for settings page)
        statusElement.className = statusElement.className.replace(/\b(connected|disconnected|connecting)\b/g, '');
        
        if (color === 'green') {
            statusElement.classList.add('connected');
        } else if (color === 'red') {
            statusElement.classList.add('disconnected');
        } else if (color === 'orange') {
            statusElement.classList.add('connecting');
        }
    }
    
    // Also update detailed status if it exists (settings page)
    const detailedStatusElement = document.getElementById('detailed-status');
    if (detailedStatusElement) {
        detailedStatusElement.textContent = text;
    }
}

function initializeEnabledPlots() {
    // Check if plot control elements exist before checking their state
    const showPose = document.getElementById('show-pose');
    const showLidar = document.getElementById('show-lidar');
    const showOccupancy = document.getElementById('show-occupancy');
    
    if (showPose && showPose.checked && typeof initializePosePlot === 'function') {
        initializePosePlot();
    }
    if (showLidar && showLidar.checked && typeof initializeLidarPlot === 'function') {
        initializeLidarPlot();
    }
    if (showOccupancy && showOccupancy.checked && typeof initializeOccupancyPlot === 'function') {
        initializeOccupancyPlot();
    }
}

// Function to get current connection settings (useful for other modules)
function getConnectionSettings() {
    return {
        ip: localStorage.getItem(CONFIG.STORAGE_KEYS.ROBOT_IP) || CONFIG.DEFAULT_IP,
        port: localStorage.getItem(CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT) || CONFIG.DEFAULT_PORT,
        url: `ws://${localStorage.getItem(CONFIG.STORAGE_KEYS.ROBOT_IP) || CONFIG.DEFAULT_IP}:${localStorage.getItem(CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT) || CONFIG.DEFAULT_PORT}`
    };
}

// Function to update settings from the settings page
function updateConnectionSettings(ip, port) {
    localStorage.setItem(CONFIG.STORAGE_KEYS.ROBOT_IP, ip);
    localStorage.setItem(CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT, port);
    
    // Update DOM elements if they exist
    const ipElement = document.getElementById('robot-ip');
    const portElement = document.getElementById('rosbridge-port');
    
    if (ipElement) ipElement.value = ip;
    if (portElement) portElement.value = port;
    
    updateCurrentUrl();
}