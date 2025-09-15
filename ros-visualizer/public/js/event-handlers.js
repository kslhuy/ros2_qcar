// Event handlers and UI interactions

function setupEventListeners() {
    // Connection buttons (if they exist on main page)
    const connectBtn = document.getElementById('connect-btn');
    const disconnectBtn = document.getElementById('disconnect-btn');
    
    if (connectBtn) connectBtn.addEventListener('click', connectToRobot);
    if (disconnectBtn) disconnectBtn.addEventListener('click', disconnectFromRobot);
    
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
        const plotCard = document.getElementById('pose-plot-card');
        if (this.checked) {
            plotCard.classList.remove('plot-card-hidden');
            if (isConnected) initializePosePlot();
        } else {
            plotCard.classList.add('plot-card-hidden');
            if (poseListener) {
                poseListener.unsubscribe();
                poseListener = null;
            }
        }
    });

    document.getElementById('show-lidar').addEventListener('change', function() {
        const plotCard = document.getElementById('lidar-plot-card');
        if (this.checked) {
            plotCard.classList.remove('plot-card-hidden');
            if (isConnected) initializeLidarPlot();
        } else {
            plotCard.classList.add('plot-card-hidden');
            if (scanListener) {
                scanListener.unsubscribe();
                scanListener = null;
            }
        }
    });

    document.getElementById('show-occupancy').addEventListener('change', function() {
        const plotCard = document.getElementById('occupancy-plot-card');
        if (this.checked) {
            plotCard.classList.remove('plot-card-hidden');
            if (isConnected) initializeOccupancyPlot();
        } else {
            plotCard.classList.add('plot-card-hidden');
            if (occupancyListener) {
                occupancyListener.unsubscribe();
                occupancyListener = null;
            }
        }
    });
}

function updateConnectionStatus(text, color) {
    const statusElement = document.getElementById('connection-status');
    if (statusElement) {
        statusElement.textContent = text;
        
        // Remove existing classes
        statusElement.classList.remove('connected', 'disconnected', 'connecting', 'bg-warning', 'bg-success', 'bg-danger');
        
        if (color === 'green') {
            statusElement.classList.add('connected', 'bg-success');
            statusElement.innerHTML = '<i class="bi bi-wifi"></i> ' + text;
        } else if (color === 'red') {
            statusElement.classList.add('disconnected', 'bg-danger');
            statusElement.innerHTML = '<i class="bi bi-wifi-off"></i> ' + text;
        } else if (color === 'orange') {
            statusElement.classList.add('connecting', 'bg-warning');
            statusElement.innerHTML = '<i class="bi bi-arrow-repeat"></i> ' + text;
        }
    }
    
    // Update footer status
    const footerStatus = document.getElementById('footer-status');
    if (footerStatus) {
        footerStatus.textContent = text;
    }
}

// Add Bootstrap toast notifications
function showNotification(message, type = 'info') {
    const alertDiv = document.createElement('div');
    alertDiv.className = `alert alert-${type} alert-dismissible fade show alert-floating`;
    alertDiv.innerHTML = `
        ${message}
        <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
    `;
    
    document.body.appendChild(alertDiv);
    
    // Auto-remove after 5 seconds
    setTimeout(() => {
        if (alertDiv.parentNode) {
            alertDiv.classList.remove('show');
            setTimeout(() => {
                if (alertDiv.parentNode) {
                    alertDiv.parentNode.removeChild(alertDiv);
                }
            }, 150);
        }
    }, 5000);
}