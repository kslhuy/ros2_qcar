// Main application initialization

document.addEventListener('DOMContentLoaded', function() {
    console.log('Initializing QCar Visualization...');
    
    // Initialize Plotly graphs
    initializePlotlyGraphs();
    
    // Load saved settings
    loadSettings();
    
    // Setup all event listeners
    setupEventListeners();
    
    // Auto-connect on page load
    connectToRobot();
    
    console.log('QCar Visualization initialized');
});