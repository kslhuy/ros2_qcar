// Event handlers and UI interactions

function setupEventListeners() { 
    // Plot control event listeners
    setupPlotControlListeners();
    
    // System info event listeners
    document.getElementById('refresh-nodes').addEventListener('click', refreshNodes);
    document.getElementById('node-select').addEventListener('change', function() {
        const selectedNode = this.value;
        getNodeParameters(selectedNode);
    });
}

function setupPlotControlListeners() {
    document.getElementById('show-pose').addEventListener('change', function() {
        const poseDiv = document.getElementById('pose-plot');
        if (this.checked) {
            poseDiv.classList.remove('hidden');
            if (isConnected) initializePosePlot();
        } else {
            poseDiv.classList.add('hidden');
            if (poseListener) {
                poseListener.unsubscribe();
                poseListener = null;
            }
        }
    });

    document.getElementById('show-lidar').addEventListener('change', function() {
        const lidarDiv = document.getElementById('lidar-plot');
        if (this.checked) {
            lidarDiv.classList.remove('hidden');
            if (isConnected) initializeLidarPlot();
        } else {
            lidarDiv.classList.add('hidden');
            if (scanListener) {
                scanListener.unsubscribe();
                scanListener = null;
            }
        }
    });

    document.getElementById('show-occupancy').addEventListener('change', function() {
        const occupancyDiv = document.getElementById('occupancy-plot');
        if (this.checked) {
            occupancyDiv.classList.remove('hidden');
            if (isConnected) initializeOccupancyPlot();
        } else {
            occupancyDiv.classList.add('hidden');
            if (occupancyListener) {
                occupancyListener.unsubscribe();
                occupancyListener = null;
            }
        }
    });
}