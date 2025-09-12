// Settings page functionality

// Load all settings from localStorage
function loadAllSettings() {
    // Connection settings
    document.getElementById('robot-ip').value = 
        localStorage.getItem(CONFIG.STORAGE_KEYS.ROBOT_IP) || CONFIG.DEFAULT_IP;
    document.getElementById('rosbridge-port').value = 
        localStorage.getItem(CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT) || CONFIG.DEFAULT_PORT;
    
    // Node information display settings
    document.getElementById('show-subscribed-topics').checked = 
        localStorage.getItem('show-subscribed-topics') !== 'false';
    document.getElementById('show-published-topics').checked = 
        localStorage.getItem('show-published-topics') !== 'false';
    document.getElementById('show-services').checked = 
        localStorage.getItem('show-services') !== 'false';
    document.getElementById('show-parameters').checked = 
        localStorage.getItem('show-parameters') !== 'false';
    
    // Parameter display settings
    document.getElementById('parameter-update-mode').value = 
        localStorage.getItem('parameter-update-mode') || 'manual';
    document.getElementById('show-parameter-types').checked = 
        localStorage.getItem('show-parameter-types') !== 'false';
    document.getElementById('confirm-parameter-changes').checked = 
        localStorage.getItem('confirm-parameter-changes') !== 'false';
    
    updateConnectionUrl();
}

function saveAllSettings() {
    // Connection settings
    localStorage.setItem(CONFIG.STORAGE_KEYS.ROBOT_IP, document.getElementById('robot-ip').value);
    localStorage.setItem(CONFIG.STORAGE_KEYS.ROSBRIDGE_PORT, document.getElementById('rosbridge-port').value);
    
    // Node information display settings
    localStorage.setItem('show-subscribed-topics', document.getElementById('show-subscribed-topics').checked);
    localStorage.setItem('show-published-topics', document.getElementById('show-published-topics').checked);
    localStorage.setItem('show-services', document.getElementById('show-services').checked);
    localStorage.setItem('show-parameters', document.getElementById('show-parameters').checked);
    
    // Parameter display settings
    localStorage.setItem('parameter-update-mode', document.getElementById('parameter-update-mode').value);
    localStorage.setItem('show-parameter-types', document.getElementById('show-parameter-types').checked);
    localStorage.setItem('confirm-parameter-changes', document.getElementById('confirm-parameter-changes').checked);
    
    // Update timestamp
    localStorage.setItem('settings-saved-at', new Date().toISOString());
    
    showNotification('Settings saved successfully!', 'success');
    updateConnectionUrl();
}

function resetToDefaults() {
    if (confirm('Are you sure you want to reset all settings to defaults? This cannot be undone.')) {
        // Clear all stored settings
        Object.values(CONFIG.STORAGE_KEYS).forEach(key => {
            localStorage.removeItem(key);
        });
        [
         'show-subscribed-topics', 'show-published-topics', 'show-services', 'show-parameters',
         'parameter-update-mode', 'show-parameter-types', 'confirm-parameter-changes'
        ].forEach(key => {
            localStorage.removeItem(key);
        });
        
        // Reload default values
        loadAllSettings();
        showNotification('Settings reset to defaults', 'info');
    }
}

function updateConnectionUrl() {
    const ip = document.getElementById('robot-ip').value;
    const port = document.getElementById('rosbridge-port').value;
    const url = `ws://${ip}:${port}`;
    document.getElementById('current-url').textContent = url;
}

function testConnection() {
    const testBtn = document.getElementById('test-connection-btn');
    const originalText = testBtn.textContent;
    
    testBtn.textContent = 'Testing...';
    testBtn.disabled = true;
    
    const ip = document.getElementById('robot-ip').value;
    const port = document.getElementById('rosbridge-port').value;
    const url = `ws://${ip}:${port}`;
    
    const testRos = new ROSLIB.Ros({ url: url });
    
    const timeout = setTimeout(() => {
        testRos.close();
        testBtn.textContent = originalText;
        testBtn.disabled = false;
        showNotification('Connection test timed out', 'error');
    }, 5000);
    
    testRos.on('connection', function() {
        clearTimeout(timeout);
        testRos.close();
        testBtn.textContent = originalText;
        testBtn.disabled = false;
        showNotification('Connection test successful!', 'success');
    });
    
    testRos.on('error', function(error) {
        clearTimeout(timeout);
        testBtn.textContent = originalText;
        testBtn.disabled = false;
        showNotification(`Connection test failed: ${error.message || 'Unknown error'}`, 'error');
    });
}

function showNotification(message, type = 'info') {
    // Create notification element
    const notification = document.createElement('div');
    notification.className = `notification notification-${type}`;
    notification.textContent = message;
    
    // Style the notification
    Object.assign(notification.style, {
        position: 'fixed',
        top: '20px',
        right: '20px',
        padding: '15px 20px',
        borderRadius: '6px',
        color: 'white',
        fontWeight: 'bold',
        zIndex: '1000',
        maxWidth: '300px',
        boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
    });
    
    // Set background color based on type
    const colors = {
        success: '#27ae60',
        error: '#e74c3c',
        warning: '#f39c12',
        info: '#3498db'
    };
    notification.style.backgroundColor = colors[type] || colors.info;
    
    document.body.appendChild(notification);
    
    // Remove after 3 seconds
    setTimeout(() => {
        if (notification.parentNode) {
            notification.parentNode.removeChild(notification);
        }
    }, 3000);
}

// Helper function to get display settings (for use by other modules)
function getDisplaySettings() {
    return {
        showSubscribedTopics: localStorage.getItem('show-subscribed-topics') !== 'false',
        showPublishedTopics: localStorage.getItem('show-published-topics') !== 'false',
        showServices: localStorage.getItem('show-services') !== 'false',
        showParameters: localStorage.getItem('show-parameters') !== 'false',
        parameterUpdateMode: localStorage.getItem('parameter-update-mode') || 'manual',
        showParameterTypes: localStorage.getItem('show-parameter-types') !== 'false',
        confirmParameterChanges: localStorage.getItem('confirm-parameter-changes') !== 'false',
        maxParametersDisplay: parseInt(localStorage.getItem('max-parameters-display')) || 50
    };
}

// Event listeners
document.addEventListener('DOMContentLoaded', function() {
    loadAllSettings();
    
    // Navigation
    document.getElementById('back-btn').addEventListener('click', function() {
        window.location.href = '/';
    });
    
    // Connection actions
    document.getElementById('test-connection-btn').addEventListener('click', testConnection);
    
    // Settings actions
    document.getElementById('save-settings-btn').addEventListener('click', saveAllSettings);
    document.getElementById('reset-settings-btn').addEventListener('click', resetToDefaults);
    
    // Auto-update URL when connection settings change
    document.getElementById('robot-ip').addEventListener('input', updateConnectionUrl);
    document.getElementById('rosbridge-port').addEventListener('input', updateConnectionUrl);
    
    // Auto-save on input change
    document.querySelectorAll('input, select').forEach(input => {
        input.addEventListener('change', function() {
            // Auto-save with a small delay to prevent too frequent saves
            clearTimeout(this.saveTimeout);
            this.saveTimeout = setTimeout(saveAllSettings, 1000);
        });
    });
});