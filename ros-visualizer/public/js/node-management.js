// Node and parameter management

function refreshNodes() {
    if (!ros || !ros.isConnected) {
        const nodesList = document.getElementById('nodes-list');
        if (nodesList) {
            nodesList.innerHTML = 'Not connected to ROS2';
        }
        return;
    }

    console.log('Getting nodes using rosapi...');
    
    const nodesClient = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/nodes',
        serviceType: 'rosapi/Nodes'
    });

    const request = new ROSLIB.ServiceRequest({});

    nodesClient.callService(request, function(result) {
        console.log('Rosapi nodes result:', result);
        populateNodesList(result.nodes);
    }, function(error) {
        console.error('Rosapi nodes failed, trying fallback:', error);
        tryFallbackGetNodes();
    });
}

function populateNodesList(nodes) {
    const nodesList = document.getElementById('nodes-list');
    const nodeSelect = document.getElementById('node-select');
    
    if (nodes && nodes.length > 0) {
        if (nodesList) {
            nodesList.innerHTML = nodes.map(node => 
                `<div class="node-item">${node}</div>`
            ).join('');
        }
        
        if (nodeSelect) {
            nodeSelect.innerHTML = '<option value="">Select a node...</option>' +
                nodes.map(node => 
                    `<option value="${node}">${node}</option>`
                ).join('');
        }
    } else {
        if (nodesList) {
            nodesList.innerHTML = 'No nodes found';
        }
    }
}

function tryFallbackGetNodes() {
    if (ros.getNodes) {
        ros.getNodes(function(nodes) {
            console.log('Fallback getNodes result:', nodes);
            populateNodesList(nodes);
        }, function(error) {
            console.error('Both methods failed:', error);
            const nodesList = document.getElementById('nodes-list');
            if (nodesList) {
                nodesList.innerHTML = 'Error fetching nodes: ' + error;
            }
        });
    } else {
        const nodesList = document.getElementById('nodes-list');
        if (nodesList) {
            nodesList.innerHTML = 'Error: No method available to get nodes';
        }
    }
}

function getNodeParameters(nodeName) {
    if (!nodeName || !ros || !ros.isConnected) {
        const paramsList = document.getElementById('parameters-list');
        if (paramsList) {
            paramsList.innerHTML = 'Select a node to view parameters';
        }
        return;
    }

    console.log('Getting details for node:', nodeName);
    const paramsList = document.getElementById('parameters-list');
    if (paramsList) {
        paramsList.innerHTML = 'Loading node information...';
    }

    const detailClient = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/node_details',
        serviceType: 'rosapi/NodeDetails'
    });

    const request = new ROSLIB.ServiceRequest({
        node: nodeName
    });

    detailClient.callService(request, function(result) {
        console.log('Node details result:', result);
        const html = buildNodeDetailsHtml(result);
        getActualParameters(nodeName, html);
        
        // Also display topic subscription options
        displayTopicSubscriptionOptions(result.publishing || [], result.subscribing || []);
    }, function(error) {
        console.error('Failed to get node details:', error);
        getActualParameters(nodeName, '');
    });
}

// Add this function to get display preferences
function getDisplaySettings() {
    return {
        showSubscribedTopics: localStorage.getItem('show-subscribed-topics') !== 'false',
        showPublishedTopics: localStorage.getItem('show-published-topics') !== 'false',
        showServices: localStorage.getItem('show-services') !== 'false',
        showParameters: localStorage.getItem('show-parameters') !== 'false',
        parameterUpdateMode: localStorage.getItem('parameter-update-mode') || 'manual',
        showParameterTypes: localStorage.getItem('show-parameter-types') !== 'false',
        confirmParameterChanges: localStorage.getItem('confirm-parameter-changes') !== 'false'    
    };
}

// Update buildNodeDetailsHtml function
function buildNodeDetailsHtml(result) {
    const settings = getDisplaySettings();
    let html = '';
    
    // Show subscribed topics
    if (settings.showSubscribedTopics && result.subscribing && result.subscribing.length > 0) {
        html += '<strong>📥 Subscribed Topics:</strong><br>';
        result.subscribing.forEach(topic => {
            html += `<div class="param-item">${topic}</div>`;
        });
        html += '<br>';
    }
    
    // Show published topics
    if (settings.showPublishedTopics && result.publishing && result.publishing.length > 0) {
        html += '<strong>📤 Published Topics:</strong><br>';
        result.publishing.forEach(topic => {
            html += `<div class="param-item">${topic}</div>`;
        });
        html += '<br>';
    }
    
    // Show services
    if (settings.showServices && result.services && result.services.length > 0) {
        html += '<strong>⚙️ Services:</strong><br>';
        result.services.forEach(service => {
            html += `<div class="param-item">${service}</div>`;
        });
        html += '<br>';
    }
    
    return html;
}

function getActualParameters(nodeName, existingHtml) {
    console.log('Getting parameters using ros.getParams()...');
    
    ros.getParams(function(paramNames) {
        console.log('All parameter names:', paramNames);
        
        if (!paramNames || paramNames.length === 0) {
            const paramsList = document.getElementById('parameters-list');
            if (paramsList) {
                paramsList.innerHTML = existingHtml + '<em>No parameters found in system</em>';
            }
            return;
        }
        
        const nodeParams = filterParametersForNode(nodeName, paramNames);
        
        if (nodeParams.length === 0) {
            const cleanNodeName = nodeName.startsWith('/') ? nodeName.substring(1) : nodeName;
            const debugHtml = `<em>No parameters found for this node</em><br>
                <small>Looking for parameters starting with: /${cleanNodeName}:</small>`;
            const paramsList = document.getElementById('parameters-list');
            if (paramsList) {
                paramsList.innerHTML = existingHtml + debugHtml;
            }
            return;
        }
        
        processParameterValues(nodeParams, existingHtml, nodeName);
        
    }, function(error) {
        console.error('Failed to get parameters:', error);
        const paramsList = document.getElementById('parameters-list');
        if (paramsList) {
            paramsList.innerHTML = existingHtml + '<em>Error fetching parameters</em>';
        }
    });
}

function filterParametersForNode(nodeName, paramNames) {
    const cleanNodeName = nodeName.startsWith('/') ? nodeName.substring(1) : nodeName;
    
    return paramNames.filter(paramName => {
        return paramName.startsWith(`/${cleanNodeName}:`) || 
               paramName.startsWith(`${cleanNodeName}:`);
    });
}

// Update processParameterValues function to respect settings
function processParameterValues(nodeParams, existingHtml, nodeName) {
    const settings = getDisplaySettings();
    
    if (!settings.showParameters) {
        const paramsList = document.getElementById('parameters-list');
        if (paramsList) {
            paramsList.innerHTML = existingHtml + '<em>Parameter display is disabled in settings</em>';
        }
        return;
    }
    
    // Limit number of parameters displayed
    const maxParams = parseInt(localStorage.getItem('max-parameters-display')) || 50;
    const limitedParams = nodeParams.slice(0, maxParams);
    
    if (nodeParams.length > maxParams) {
        existingHtml += `<div class="alert alert-warning">⚠️ Showing ${maxParams} of ${nodeParams.length} parameters (limit set in settings)</div><br>`;
    }
        
    existingHtml += '<strong>⚙️ Parameters:</strong><br>';
    existingHtml += '<div class="parameters-container">';
    
    let processedParams = 0;
    
    limitedParams.forEach(paramName => {
        const param = new ROSLIB.Param({
            ros: ros,
            name: paramName
        });
        
        param.get(function(value) {
            const shortParamName = paramName.split(':')[1] || paramName;
            const paramId = `param-${paramName.replace(/[^a-zA-Z0-9]/g, '_')}`;
            
            existingHtml += createParameterEditableHtml(paramName, shortParamName, value, paramId);
            
            processedParams++;
            
            if (processedParams === limitedParams.length) {
                existingHtml += '</div>';
                existingHtml += `<div class="param-actions">
                    <button onclick="refreshNodeParameters('${nodeName}')" class="btn btn-secondary">
                        <i class="bi bi-arrow-clockwise"></i> Refresh
                    </button>
                    <button onclick="saveAllNodeParameters('${nodeName}')" class="btn btn-success">
                        <i class="bi bi-save"></i> Save All
                    </button>
                </div>`;
                
                const paramsList = document.getElementById('parameters-list');
                if (paramsList) {
                    paramsList.innerHTML = existingHtml;
                    setupParameterEventListeners();
                }
            }
        });
    });
}

function createParameterEditableHtml(fullParamName, shortParamName, value, paramId) {
    const settings = getDisplaySettings();
    const valueType = typeof value;
    let inputHtml = '';
    
    // Determine input type based on value type
    if (valueType === 'boolean') {
        inputHtml = `
            <select id="${paramId}" 
                    data-param-name="${fullParamName}" 
                    data-original-value="${value}"
                    class="param-input form-select">
                <option value="true" ${value ? 'selected' : ''}>true</option>
                <option value="false" ${!value ? 'selected' : ''}>false</option>
            </select>`;
    } else if (valueType === 'number') {
        const step = Number.isInteger(value) ? '1' : '0.01';
        inputHtml = `
            <input type="number" 
                   id="${paramId}" 
                   data-param-name="${fullParamName}" 
                   data-original-value="${value}"
                   value="${value}" 
                   step="${step}"
                   class="param-input form-control">`;
    } else if (valueType === 'string') {
        inputHtml = `
            <input type="text" 
                   id="${paramId}" 
                   data-param-name="${fullParamName}" 
                   data-original-value="${value}"
                   value="${value}" 
                   class="param-input form-control">`;
    } else {
        // For arrays or complex objects, use textarea
        inputHtml = `
            <textarea id="${paramId}" 
                      data-param-name="${fullParamName}" 
                      data-original-value="${JSON.stringify(value)}"
                      rows="3" 
                      class="param-input form-control">${JSON.stringify(value)}</textarea>`;
    }
    
    // Build the parameter type display based on settings
    const typeDisplay = settings.showParameterTypes ? 
        `<span class="param-type">[${valueType}]</span>` : '';
    
    return `
        <div class="param-item-editable" data-param-name="${fullParamName}">
            <div class="param-header">
                <span class="param-name">${shortParamName}:</span>
                ${typeDisplay}
                <div class="param-buttons">
                    <button onclick="updateParameter('${fullParamName}', '${paramId}')" 
                            class="btn btn-sm btn-primary" title="Update this parameter">
                        <i class="bi bi-save"></i>
                    </button>
                    <button onclick="resetParameter('${paramId}')" 
                            class="btn btn-sm btn-secondary" title="Reset to original value">
                        <i class="bi bi-arrow-clockwise"></i>
                    </button>
                </div>
            </div>
            <div class="param-input-container">
                ${inputHtml}
                <div class="param-status" id="${paramId}-status"></div>
            </div>
        </div>`;
}

function setupParameterEventListeners() {
    // Add change detection to all parameter inputs and selects
    document.querySelectorAll('.param-input').forEach(element => {
        // Determine which event to listen for based on element type
        const eventType = element.tagName.toLowerCase() === 'select' ? 'change' : 'input';
        
        element.addEventListener(eventType, function() {
            const originalValue = this.getAttribute('data-original-value');
            const currentValue = this.value;
            const statusDiv = document.getElementById(this.id + '-status');
            
            if (currentValue !== originalValue) {
                this.classList.add('param-modified');
                if (statusDiv) {
                    statusDiv.textContent = 'Modified';
                    statusDiv.className = 'param-status modified';
                }
            } else {
                this.classList.remove('param-modified');
                if (statusDiv) {
                    statusDiv.textContent = '';
                    statusDiv.className = 'param-status';
                }
            }
        });
    });
}

function updateParameter(paramName, inputId) {
    const inputElement = document.getElementById(inputId);
    const statusDiv = document.getElementById(inputId + '-status');
    
    if (!inputElement) {
        console.error('Input element not found:', inputId);
        return;
    }
    
    let newValue = inputElement.value;
    
    // Convert value based on original type
    const originalValue = inputElement.getAttribute('data-original-value');
    if (originalValue === 'true' || originalValue === 'false') {
        newValue = newValue === 'true';
    } else if (!isNaN(originalValue) && originalValue !== '') {
        newValue = parseFloat(newValue);
        if (Number.isInteger(parseFloat(originalValue))) {
            newValue = parseInt(newValue);
        }
    } else if (originalValue.startsWith('[') || originalValue.startsWith('{')) {
        try {
            newValue = JSON.parse(newValue);
        } catch (e) {
            if (statusDiv) {
                statusDiv.textContent = 'Invalid JSON';
                statusDiv.className = 'param-status error';
            }
            return;
        }
    }
    
    console.log(`Updating parameter ${paramName} to:`, newValue);
    
    // Update status to show saving
    if (statusDiv) {
        statusDiv.textContent = 'Saving...';
        statusDiv.className = 'param-status saving';
    }
    
    const param = new ROSLIB.Param({
        ros: ros,
        name: paramName
    });
    
    param.set(newValue, function() {
        console.log(`Successfully updated ${paramName}`);
        
        // Update the original value attribute
        inputElement.setAttribute('data-original-value', inputElement.value);
        inputElement.classList.remove('param-modified');
        
        if (statusDiv) {
            statusDiv.textContent = 'Saved ✓';
            statusDiv.className = 'param-status saved';
            
            // Clear status after 2 seconds
            setTimeout(() => {
                statusDiv.textContent = '';
                statusDiv.className = 'param-status';
            }, 2000);
        }
        
        showParameterNotification(`Parameter ${paramName.split(':')[1]} updated successfully`, 'success');
        
    });
}

function resetParameter(inputId) {
    const inputElement = document.getElementById(inputId);
    const statusDiv = document.getElementById(inputId + '-status');
    
    if (!inputElement) return;
    
    const originalValue = inputElement.getAttribute('data-original-value');
    inputElement.value = originalValue;
    inputElement.classList.remove('param-modified');
    
    if (statusDiv) {
        statusDiv.textContent = '';
        statusDiv.className = 'param-status';
    }
}

function refreshNodeParameters(nodeName) {
    getNodeParameters(nodeName);
}

function saveAllNodeParameters(nodeName) {
    const modifiedInputs = document.querySelectorAll('.param-input.param-modified');
    
    if (modifiedInputs.length === 0) {
        showParameterNotification('No parameters have been modified', 'info');
        return;
    }
    
    let savedCount = 0;
    let totalCount = modifiedInputs.length;
    
    modifiedInputs.forEach(input => {
        const paramName = input.getAttribute('data-param-name');
        updateParameter(paramName, input.id);
        savedCount++;
    });
    
    showParameterNotification(`Saving ${totalCount} modified parameters...`, 'info');
}

function showParameterNotification(message, type = 'info') {
    // Create notification element
    const notification = document.createElement('div');
    notification.className = `param-notification param-notification-${type}`;
    notification.textContent = message;
    
    // Style the notification
    Object.assign(notification.style, {
        position: 'fixed',
        top: '20px',
        right: '20px',
        padding: '10px 15px',
        borderRadius: '4px',
        color: 'white',
        fontWeight: 'bold',
        zIndex: '1000',
        maxWidth: '300px',
        fontSize: '12px'
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

function getAllParametersOrganized() {
    if (!ros || !ros.isConnected) {
        console.log('Not connected');
        return;
    }
    
    console.log('Getting all parameters organized by node...');
    
    ros.getParams(function(paramNames) {
        console.log('=== ALL PARAMETERS ===');
        
        const paramsByNode = {};
        
        paramNames.forEach(paramName => {
            if (paramName.includes(':')) {
                const [nodePart] = paramName.split(':');
                
                if (!paramsByNode[nodePart]) {
                    paramsByNode[nodePart] = [];
                }
                paramsByNode[nodePart].push(paramName);
            } else {
                if (!paramsByNode['global']) {
                    paramsByNode['global'] = [];
                }
                paramsByNode['global'].push(paramName);
            }
        });
        
        console.log('Parameters organized by node:', paramsByNode);
        
        // Get values for each parameter
        Object.keys(paramsByNode).forEach(node => {
            console.log(`\n--- Parameters for ${node} ---`);
            paramsByNode[node].forEach(paramName => {
                const param = new ROSLIB.Param({
                    ros: ros,
                    name: paramName
                });
                
                param.get(function(value) {
                    const shortName = paramName.split(':')[1] || paramName;
                    console.log(`${shortName}: ${JSON.stringify(value)}`);
                });
            });
        });
    });
}

// Add these variables at the top of the file (after the existing code)
let activeSubscriptions = new Map(); // Store active topic subscriptions
let topicMessageCounts = new Map(); // Track message counts per topic


function updateTopicSubscriptionsList() {
    const activeSubsList = document.getElementById('active-subscriptions-list');
    if (activeSubsList) {
        if (activeSubscriptions.size === 0) {
            activeSubsList.innerHTML = '<em class="text-muted">No active subscriptions</em>';
        } else {
            const subscriptionItems = Array.from(activeSubscriptions.entries()).map(([topicName, info]) => {
                const count = topicMessageCounts.get(topicName) || 0;
                return `
                    <div class="subscription-item d-flex justify-content-between align-items-center p-2 border-bottom">
                        <div>
                            <strong>${topicName}</strong><br>
                            <small class="text-muted">${info.messageType}</small>
                        </div>
                        <div class="text-end">
                            <span class="badge bg-success">${count} msgs</span><br>
                            <button onclick="unsubscribeFromTopic('${topicName}')" 
                                    class="btn btn-sm btn-outline-danger mt-1">
                                <i class="bi bi-x"></i>
                            </button>
                        </div>
                    </div>
                `;
            }).join('');
            
            activeSubsList.innerHTML = subscriptionItems;
        }
    }
}

// Add this function after getAllParametersOrganized()
function subscribeToTopic(topicName, messageType) {
    // Check if already subscribed
    if (activeSubscriptions.has(topicName)) {
        showParameterNotification(`Already subscribed to ${topicName}`, 'warning');
        return;
    }
    
    if (!ros || !ros.isConnected) {
        showParameterNotification('Not connected to ROS2', 'error');
        return;
    }
    
    console.log(`Subscribing to topic: ${topicName} (${messageType})`);
    
    const listener = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType
    });
    
    // Initialize message count
    topicMessageCounts.set(topicName, 0);
    
    listener.subscribe(function(message) {
        // Increment message count
        const count = topicMessageCounts.get(topicName) + 1;
        topicMessageCounts.set(topicName, count);
        
        // Update the display
        updateTopicMessageDisplay(topicName, message, count);
    });
    
    // Store the subscription
    activeSubscriptions.set(topicName, {
        listener: listener,
        messageType: messageType,
        startTime: new Date()
    });
    
    // Update the UI
    updateTopicSubscriptionsList();
    showParameterNotification(`Subscribed to ${topicName}`, 'success');
}

function unsubscribeFromTopic(topicName) {
    const subscription = activeSubscriptions.get(topicName);
    if (subscription) {
        subscription.listener.unsubscribe();
        activeSubscriptions.delete(topicName);
        topicMessageCounts.delete(topicName);
        
        // Remove from display
        const messageDisplay = document.getElementById(`topic-messages-${topicName.replace(/[^a-zA-Z0-9]/g, '_')}`);
        if (messageDisplay) {
            messageDisplay.remove();
        }
        
        updateTopicSubscriptionsList();
        showParameterNotification(`Unsubscribed from ${topicName}`, 'info');
    }
}

function updateTopicMessageDisplay(topicName, message, count) {
    const displayId = `topic-messages-${topicName.replace(/[^a-zA-Z0-9]/g, '_')}`;
    let messageDisplay = document.getElementById(displayId);
    
    if (!messageDisplay) {
        // Create new message display container
        const topicSubscriptionsContainer = document.getElementById('topic-subscriptions-container');
        if (topicSubscriptionsContainer) {
            messageDisplay = document.createElement('div');
            messageDisplay.id = displayId;
            messageDisplay.className = 'topic-message-display card mb-3';
            topicSubscriptionsContainer.appendChild(messageDisplay);
        } else {
            return; // Container doesn't exist
        }
    }
    
    // Get message type for visualization
    const subscription = activeSubscriptions.get(topicName);
    const messageType = subscription ? subscription.messageType : 'unknown';
    
    // Create appropriate visualization based on message type
    const visualization = createMessageVisualization(topicName, message, messageType, count);
    const timestamp = new Date().toLocaleTimeString();
    
    messageDisplay.innerHTML = `
        <div class="card-header d-flex justify-content-between align-items-center">
            <h6 class="mb-0">
                <i class="bi bi-broadcast"></i> ${topicName}
                <span class="badge bg-secondary ms-2">${messageType}</span>
            </h6>
            <div>
                <span class="badge bg-primary">${count} msgs</span>
                <button onclick="unsubscribeFromTopic('${topicName}')" 
                        class="btn btn-sm btn-outline-danger ms-2">
                    <i class="bi bi-x-circle"></i> Unsubscribe
                </button>
            </div>
        </div>
        <div class="card-body">
            ${visualization}
            <div class="mt-2">
                <small class="text-muted">Last update: ${timestamp} | Messages: ${count}</small>
            </div>
        </div>
    `;
}

function createMessageVisualization(topicName, message, messageType, count) {
    const visualizationId = `viz-${topicName.replace(/[^a-zA-Z0-9]/g, '_')}`;
    
    switch (messageType) {
        case 'geometry_msgs/Twist':
            return createTwistVisualization(visualizationId, message);
        
        case 'geometry_msgs/PoseStamped':
        case 'geometry_msgs/Pose':
            return createPoseVisualization(visualizationId, message);
        
        case 'sensor_msgs/LaserScan':
            return createLaserScanVisualization(visualizationId, message);
        
        case 'nav_msgs/Odometry':
            return createOdometryVisualization(visualizationId, message);
        
        case 'std_msgs/String':
            return createStringVisualization(visualizationId, message);
        
        case 'std_msgs/Int32':
        case 'std_msgs/Float64':
            return createNumericVisualization(visualizationId, message, messageType, topicName);
        
        case 'std_msgs/Bool':
            return createBooleanVisualization(visualizationId, message);
        
        case 'sensor_msgs/Image':
            return createImageVisualization(visualizationId, message);
        
        case 'nav_msgs/OccupancyGrid':
            return createOccupancyGridVisualization(visualizationId, message);
        
        default:
            return createGenericVisualization(visualizationId, message);
    }
}

function createTwistVisualization(vizId, message) {
    const linear = message.linear || {};
    const angular = message.angular || {};
    
    // Schedule canvas drawing
    setTimeout(() => {
        drawTwistDirection(`${vizId}-direction`, linear.x || 0, angular.z || 0);
    }, 100);
    
    return `
        <div class="row">
            <div class="col-md-6">
                <h6><i class="bi bi-arrow-up-right"></i> Linear Velocity</h6>
                <div class="velocity-display">
                    <div class="velocity-bar">
                        <label>X: ${(linear.x || 0).toFixed(3)} m/s</label>
                        <div class="progress mb-1">
                            <div class="progress-bar bg-success" style="width: ${Math.abs(linear.x || 0) * 50}%"></div>
                        </div>
                    </div>
                    <div class="velocity-bar">
                        <label>Y: ${(linear.y || 0).toFixed(3)} m/s</label>
                        <div class="progress mb-1">
                            <div class="progress-bar bg-info" style="width: ${Math.abs(linear.y || 0) * 50}%"></div>
                        </div>
                    </div>
                    <div class="velocity-bar">
                        <label>Z: ${(linear.z || 0).toFixed(3)} m/s</label>
                        <div class="progress mb-1">
                            <div class="progress-bar bg-warning" style="width: ${Math.abs(linear.z || 0) * 50}%"></div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-6">
                <h6><i class="bi bi-arrow-clockwise"></i> Angular Velocity</h6>
                <div class="velocity-display">
                    <div class="velocity-bar">
                        <label>X: ${(angular.x || 0).toFixed(3)} rad/s</label>
                        <div class="progress mb-1">
                            <div class="progress-bar bg-danger" style="width: ${Math.abs(angular.x || 0) * 30}%"></div>
                        </div>
                    </div>
                    <div class="velocity-bar">
                        <label>Y: ${(angular.y || 0).toFixed(3)} rad/s</label>
                        <div class="progress mb-1">
                            <div class="progress-bar bg-primary" style="width: ${Math.abs(angular.y || 0) * 30}%"></div>
                        </div>
                    </div>
                    <div class="velocity-bar">
                        <label>Z: ${(angular.z || 0).toFixed(3)} rad/s</label>
                        <div class="progress mb-1">
                            <div class="progress-bar bg-secondary" style="width: ${Math.abs(angular.z || 0) * 30}%"></div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        <div class="mt-2">
            <canvas id="${vizId}-direction" width="200" height="200" class="border"></canvas>
        </div>
    `;
}

function createPoseVisualization(vizId, message) {
    const pose = message.pose || message;
    const position = pose.position || {};
    const orientation = pose.orientation || {};
    
    // Convert quaternion to Euler angles
    const euler = quaternionToEuler(orientation.x || 0, orientation.y || 0, orientation.z || 0, orientation.w || 1);
    
    // Schedule canvas drawing
    setTimeout(() => {
        drawPoseVisualization(`${vizId}-pose`, position.x || 0, position.y || 0, euler.yaw);
    }, 100);
    
    return `
        <div class="row">
            <div class="col-md-6">
                <h6><i class="bi bi-geo-alt"></i> Position</h6>
                <table class="table table-sm">
                    <tr><td>X:</td><td class="text-end">${(position.x || 0).toFixed(3)} m</td></tr>
                    <tr><td>Y:</td><td class="text-end">${(position.y || 0).toFixed(3)} m</td></tr>
                    <tr><td>Z:</td><td class="text-end">${(position.z || 0).toFixed(3)} m</td></tr>
                </table>
            </div>
            <div class="col-md-6">
                <h6><i class="bi bi-compass"></i> Orientation</h6>
                <table class="table table-sm">
                    <tr><td>Roll:</td><td class="text-end">${euler.roll.toFixed(3)} rad</td></tr>
                    <tr><td>Pitch:</td><td class="text-end">${euler.pitch.toFixed(3)} rad</td></tr>
                    <tr><td>Yaw:</td><td class="text-end">${euler.yaw.toFixed(3)} rad</td></tr>
                </table>
            </div>
        </div>
        <div class="mt-2">
            <canvas id="${vizId}-pose" width="300" height="200" class="border"></canvas>
        </div>
    `;
}

function createNumericVisualization(vizId, message, messageType, topicName) {
    const value = message.data;
    const isFloat = messageType.includes('Float');
    
    // Store historical data for plotting
    if (!window.topicHistory) window.topicHistory = {};
    if (!window.topicHistory[topicName]) window.topicHistory[topicName] = [];
    
    const history = window.topicHistory[topicName];
    history.push({
        value: value,
        timestamp: Date.now()
    });
    
    // Keep only last 50 points
    if (history.length > 50) {
        history.shift();
    }
    
    // Schedule plot drawing
    setTimeout(() => {
        plotNumericHistory(`${vizId}-chart`, topicName);
    }, 100);
    
    return `
        <div class="row">
            <div class="col-md-4">
                <div class="numeric-value-display text-center">
                    <h2 class="display-4 text-primary">${isFloat ? value.toFixed(3) : value}</h2>
                    <small class="text-muted">${messageType.split('/')[1]}</small>
                </div>
            </div>
            <div class="col-md-8">
                <div id="${vizId}-chart" style="height: 200px;"></div>
            </div>
        </div>
    `;
}

function createBooleanVisualization(vizId, message) {
    const value = message.data;
    
    return `
        <div class="text-center">
            <div class="boolean-indicator ${value ? 'active' : 'inactive'}">
                <i class="bi ${value ? 'bi-check-circle-fill' : 'bi-x-circle-fill'}"></i>
                <h3>${value ? 'TRUE' : 'FALSE'}</h3>
            </div>
        </div>
    `;
}

function createStringVisualization(vizId, message) {
    const text = message.data || '';
    
    return `
        <div class="string-message-display">
            <div class="alert alert-info">
                <i class="bi bi-chat-square-text"></i>
                <strong>Message:</strong> "${text}"
            </div>
            <small class="text-muted">Length: ${text.length} characters</small>
        </div>
    `;
}

function createOdometryVisualization(vizId, message) {
    const pose = message.pose?.pose || {};
    const twist = message.twist?.twist || {};
    
    return `
        <div class="row">
            <div class="col-md-4">
                <h6><i class="bi bi-geo-alt"></i> Position</h6>
                <small>X: ${(pose.position?.x || 0).toFixed(3)} m</small><br>
                <small>Y: ${(pose.position?.y || 0).toFixed(3)} m</small><br>
                <small>Z: ${(pose.position?.z || 0).toFixed(3)} m</small>
            </div>
            <div class="col-md-4">
                <h6><i class="bi bi-speedometer2"></i> Linear Vel</h6>
                <small>X: ${(twist.linear?.x || 0).toFixed(3)} m/s</small><br>
                <small>Y: ${(twist.linear?.y || 0).toFixed(3)} m/s</small><br>
                <small>Z: ${(twist.linear?.z || 0).toFixed(3)} m/s</small>
            </div>
            <div class="col-md-4">
                <h6><i class="bi bi-arrow-clockwise"></i> Angular Vel</h6>
                <small>X: ${(twist.angular?.x || 0).toFixed(3)} rad/s</small><br>
                <small>Y: ${(twist.angular?.y || 0).toFixed(3)} rad/s</small><br>
                <small>Z: ${(twist.angular?.z || 0).toFixed(3)} rad/s</small>
            </div>
        </div>
    `;
}

function createLaserScanVisualization(vizId, message) {
    const ranges = message.ranges || [];
    const validRanges = ranges.filter(r => r > 0.1 && r < 20 && isFinite(r));
    const minRange = Math.min(...validRanges);
    const maxRange = Math.max(...validRanges);
    const avgRange = validRanges.reduce((a, b) => a + b, 0) / validRanges.length;
    
    // Schedule canvas drawing
    setTimeout(() => {
        drawLidarScan(`${vizId}-lidar`, ranges, message.angle_min || 0, message.angle_max || 0);
    }, 100);
    
    return `
        <div class="row">
            <div class="col-md-6">
                <h6><i class="bi bi-radar"></i> Scan Statistics</h6>
                <table class="table table-sm">
                    <tr><td>Total Points:</td><td class="text-end">${ranges.length}</td></tr>
                    <tr><td>Valid Points:</td><td class="text-end">${validRanges.length}</td></tr>
                    <tr><td>Min Range:</td><td class="text-end">${minRange.toFixed(2)} m</td></tr>
                    <tr><td>Max Range:</td><td class="text-end">${maxRange.toFixed(2)} m</td></tr>
                    <tr><td>Avg Range:</td><td class="text-end">${avgRange.toFixed(2)} m</td></tr>
                </table>
            </div>
            <div class="col-md-6">
                <canvas id="${vizId}-lidar" width="250" height="250" class="border"></canvas>
            </div>
        </div>
    `;
}

function createGenericVisualization(vizId, message) {
    const formattedMessage = formatTopicMessage(message);
    
    return `
        <div class="generic-message-display">
            <h6><i class="bi bi-code-square"></i> Raw Message Data</h6>
            <pre class="topic-message-content">${formattedMessage}</pre>
        </div>
    `;
}

// Helper functions for visualizations
function quaternionToEuler(x, y, z, w) {
    // Convert quaternion to Euler angles
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    
    const sinp = 2 * (w * y - z * x);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
    
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);
    
    return { roll, pitch, yaw };
}

// Canvas drawing functions
function drawTwistDirection(canvasId, linearX, angularZ) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw robot (circle)
    ctx.beginPath();
    ctx.arc(centerX, centerY, 30, 0, 2 * Math.PI);
    ctx.strokeStyle = '#007bff';
    ctx.lineWidth = 3;
    ctx.stroke();
    
    // Draw linear velocity arrow
    if (Math.abs(linearX) > 0.01) {
        const arrowLength = Math.min(50, Math.abs(linearX) * 50);
        const arrowX = linearX > 0 ? centerX + arrowLength : centerX - arrowLength;
        
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(arrowX, centerY);
        ctx.strokeStyle = '#28a745';
        ctx.lineWidth = 3;
        ctx.stroke();
        
        // Arrow head
        const headSize = 10;
        const direction = linearX > 0 ? 1 : -1;
        ctx.beginPath();
        ctx.moveTo(arrowX, centerY);
        ctx.lineTo(arrowX - direction * headSize, centerY - headSize / 2);
        ctx.lineTo(arrowX - direction * headSize, centerY + headSize / 2);
        ctx.closePath();
        ctx.fillStyle = '#28a745';
        ctx.fill();
    }
    
    // Draw angular velocity indicator
    if (Math.abs(angularZ) > 0.01) {
        const radius = 45;
        const startAngle = -Math.PI / 4;
        const endAngle = angularZ > 0 ? startAngle + Math.PI / 2 : startAngle - Math.PI / 2;
        
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, startAngle, endAngle);
        ctx.strokeStyle = '#dc3545';
        ctx.lineWidth = 2;
        ctx.stroke();
    }
}

function drawPoseVisualization(canvasId, x, y, yaw) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const scale = 20;
    
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw grid
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 1;
    for (let i = -5; i <= 5; i++) {
        ctx.beginPath();
        ctx.moveTo(centerX + i * scale, 0);
        ctx.lineTo(centerX + i * scale, canvas.height);
        ctx.stroke();
        
        ctx.beginPath();
        ctx.moveTo(0, centerY + i * scale);
        ctx.lineTo(canvas.width, centerY + i * scale);
        ctx.stroke();
    }
    
    // Draw robot position
    const robotX = centerX + x * scale;
    const robotY = centerY - y * scale; // Flip Y axis
    
    ctx.beginPath();
    ctx.arc(robotX, robotY, 8, 0, 2 * Math.PI);
    ctx.fillStyle = '#007bff';
    ctx.fill();
    
    // Draw orientation arrow
    const arrowLength = 20;
    const arrowEndX = robotX + Math.cos(yaw) * arrowLength;
    const arrowEndY = robotY - Math.sin(yaw) * arrowLength;
    
    ctx.beginPath();
    ctx.moveTo(robotX, robotY);
    ctx.lineTo(arrowEndX, arrowEndY);
    ctx.strokeStyle = '#dc3545';
    ctx.lineWidth = 3;
    ctx.stroke();
}

function drawLidarScan(canvasId, ranges, angleMin, angleMax) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const scale = 10;
    
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw center point
    ctx.beginPath();
    ctx.arc(centerX, centerY, 3, 0, 2 * Math.PI);
    ctx.fillStyle = '#dc3545';
    ctx.fill();
    
    // Draw LIDAR points
    ctx.fillStyle = '#007bff';
    for (let i = 0; i < ranges.length; i++) {
        const range = ranges[i];
        if (range > 0.1 && range < 20 && isFinite(range)) {
            const angle = angleMin + (angleMax - angleMin) * i / (ranges.length - 1);
            const x = centerX + Math.cos(angle) * range * scale;
            const y = centerY - Math.sin(angle) * range * scale;
            
            ctx.beginPath();
            ctx.arc(x, y, 1, 0, 2 * Math.PI);
            ctx.fill();
        }
    }
}

function plotNumericHistory(chartId, topicName) {
    const history = window.topicHistory[topicName] || [];
    if (history.length === 0) return;
    
    const trace = {
        x: history.map(h => new Date(h.timestamp)),
        y: history.map(h => h.value),
        type: 'scatter',
        mode: 'lines+markers',
        line: { color: '#007bff' },
        marker: { size: 4 }
    };
    
    const layout = {
        margin: { l: 40, r: 20, t: 20, b: 40 },
        xaxis: { title: 'Time' },
        yaxis: { title: 'Value' },
        showlegend: false
    };
    
    Plotly.newPlot(chartId, [trace], layout, { responsive: true });
}

function getTopicsForNode(nodeName) {
    if (!nodeName || !ros || !ros.isConnected) {
        return;
    }
    
    // Get topics from the node details we already have
    const detailClient = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/node_details',
        serviceType: 'rosapi/NodeDetails'
    });

    const request = new ROSLIB.ServiceRequest({
        node: nodeName
    });

    detailClient.callService(request, function(result) {
        console.log('Node details for topic subscription:', result);
        displayTopicSubscriptionOptions(result.publishing || [], result.subscribing || []);
    }, function(error) {
        console.error('Failed to get node details for topics:', error);
    });
}

function displayTopicSubscriptionOptions(publishedTopics, subscribedTopics) {
    const topicOptionsContainer = document.getElementById('topic-subscription-options');
    if (!topicOptionsContainer) return;
    
    let optionsHtml = '';
    
    if (publishedTopics.length > 0) {
        optionsHtml += '<h6><i class="bi bi-upload"></i> Published Topics (available for subscription):</h6>';
        optionsHtml += '<div class="topic-options-grid">';
        
        publishedTopics.forEach(topic => {
            const isSubscribed = activeSubscriptions.has(topic);
            optionsHtml += `
                <div class="topic-option-item ${isSubscribed ? 'subscribed' : ''}">
                    <div class="topic-name">${topic}</div>
                    <div class="topic-actions">
                        ${isSubscribed ? 
                            `<button onclick="unsubscribeFromTopic('${topic}')" class="btn btn-sm btn-outline-danger">
                                <i class="bi bi-x"></i> Unsubscribe
                            </button>` :
                            `<button onclick="promptTopicSubscription('${topic}')" class="btn btn-sm btn-primary">
                                <i class="bi bi-plus"></i> Subscribe
                            </button>`
                        }
                    </div>
                </div>
            `;
        });
        
        optionsHtml += '</div>';
    }
    
    topicOptionsContainer.innerHTML = optionsHtml;
}

function promptTopicSubscription(topicName) {
    // Common ROS2 message types
    const commonMessageTypes = [
        'std_msgs/String',
        'std_msgs/Int32',
        'std_msgs/Float64',
        'std_msgs/Bool',
        'geometry_msgs/Twist',
        'geometry_msgs/PoseStamped',
        'sensor_msgs/LaserScan',
        'sensor_msgs/Image',
        'nav_msgs/OccupancyGrid',
        'nav_msgs/Odometry'
    ];
    
    // Create a modal-like prompt
    const messageTypeSelection = commonMessageTypes.map(msgType => 
        `<option value="${msgType}">${msgType}</option>`
    ).join('');
    
    const promptHtml = `
        <div class="message-type-prompt">
            <h6>Subscribe to: ${topicName}</h6>
            <div class="mb-3">
                <label for="message-type-select" class="form-label">Message Type:</label>
                <select id="message-type-select" class="form-select">
                    <option value="">Select message type...</option>
                    ${messageTypeSelection}
                </select>
            </div>
            <div class="mb-3">
                <label for="custom-message-type" class="form-label">Or enter custom type:</label>
                <input type="text" id="custom-message-type" class="form-control" 
                       placeholder="e.g., my_package/CustomMessage">
            </div>
            <div class="d-flex gap-2">
                <button onclick="confirmTopicSubscription('${topicName}')" class="btn btn-primary">
                    Subscribe
                </button>
                <button onclick="cancelTopicSubscription()" class="btn btn-secondary">
                    Cancel
                </button>
            </div>
        </div>
    `;
    
    // Show the prompt in a container
    const promptContainer = document.getElementById('topic-subscription-prompt');
    if (promptContainer) {
        promptContainer.innerHTML = promptHtml;
        promptContainer.style.display = 'block';
    }
}

function confirmTopicSubscription(topicName) {
    const messageTypeSelect = document.getElementById('message-type-select');
    const customMessageType = document.getElementById('custom-message-type');
    
    const messageType = customMessageType.value.trim() || messageTypeSelect.value;
    
    if (!messageType) {
        showParameterNotification('Please select or enter a message type', 'warning');
        return;
    }
    
    subscribeToTopic(topicName, messageType);
    cancelTopicSubscription();
}

function cancelTopicSubscription() {
    const promptContainer = document.getElementById('topic-subscription-prompt');
    if (promptContainer) {
        promptContainer.style.display = 'none';
        promptContainer.innerHTML = '';
    }
}

// Add these functions after the existing visualization functions in node-management.js

function createImageVisualization(vizId, message) {
    const width = message.width || 640;
    const height = message.height || 480;
    const encoding = message.encoding || 'unknown';
    const step = message.step || width;
    const dataLength = message.data ? message.data.length : 0;
    
    // Schedule the canvas drawing after the DOM is updated, passing the actual image data
    setTimeout(() => {
        drawImageVisualization(`${vizId}-image`, {
            width: width,
            height: height,
            encoding: encoding,
            dataLength: dataLength
        }, message.data); // Pass the actual image data here
    }, 100);
    
    return `
        <div class="row">
            <div class="col-md-6">
                <h6><i class="bi bi-camera"></i> Image Information</h6>
                <table class="table table-sm">
                    <tr><td>Width:</td><td class="text-end">${width} px</td></tr>
                    <tr><td>Height:</td><td class="text-end">${height} px</td></tr>
                    <tr><td>Encoding:</td><td class="text-end">${encoding}</td></tr>
                    <tr><td>Step:</td><td class="text-end">${step} bytes</td></tr>
                    <tr><td>Data Size:</td><td class="text-end">${dataLength} bytes</td></tr>
                    <tr><td>Expected Size:</td><td class="text-end">${getExpectedImageSize(width, height, encoding)} bytes</td></tr>
                </table>
            </div>
            <div class="col-md-6">
                <div class="image-display-container">
                    <canvas id="${vizId}-image" width="400" height="300" class="border"></canvas>
                    <div class="mt-2">
                        <small class="text-muted">
                            ${getImageEncodingDescription(encoding)}
                        </small>
                    </div>
                </div>
            </div>
        </div>
        <div class="mt-2">
            <div class="alert ${dataLength > 0 ? 'alert-success' : 'alert-info'}">
                <i class="bi bi-info-circle"></i>
                <strong>${dataLength > 0 ? 'Real Image Data' : 'No Image Data'}:</strong> 
                ${dataLength > 0 ? 
                    'Displaying actual image from ROS2 topic.' : 
                    'No image data received yet, showing placeholder.'}
            </div>
        </div>
    `;
}

function getExpectedImageSize(width, height, encoding) {
    const pixels = width * height;
    switch (encoding) {
        case 'mono8': return pixels;
        case 'rgb8':
        case 'bgr8': return pixels * 3;
        case 'rgba8':
        case 'bgra8': return pixels * 4;
        case 'mono16': return pixels * 2;
        case 'rgb16':
        case 'bgr16': return pixels * 6;
        default: return 'Unknown';
    }
}

function getImageEncodingDescription(encoding) {
    const descriptions = {
        'mono8': 'Grayscale 8-bit image',
        'rgb8': 'Color RGB 8-bit image',
        'bgr8': 'Color BGR 8-bit image',
        'rgba8': 'Color RGBA 8-bit image with transparency',
        'bgra8': 'Color BGRA 8-bit image with transparency',
        'mono16': 'Grayscale 16-bit image',
        'rgb16': 'Color RGB 16-bit image',
        'bgr16': 'Color BGR 16-bit image'
    };
    return descriptions[encoding] || `Unknown encoding: ${encoding}`;
}

// Add error handling for compressed images
function handleCompressedImage(message) {
    // For compressed images (like 'compressed' encoding), we'd need to decode them
    // This would require additional libraries like jpeg-js or similar
    console.warn('Compressed image formats require additional decoding libraries');
    return null;
}

// Also add the missing formatTopicMessage function if it doesn't exist
function formatTopicMessage(message) {
    try {
        return JSON.stringify(message, null, 2);
    } catch (e) {
        return String(message);
    }
}

// Update the drawImageVisualization function to handle real image data
function drawImageVisualization(canvasId, imageInfo, imageData = null) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) {
        console.error('Canvas not found:', canvasId);
        return;
    }
    
    console.log('Drawing image visualization for:', canvasId, imageInfo);
    
    const ctx = canvas.getContext('2d');
    if (!ctx) {
        console.error('Could not get 2D context for canvas:', canvasId);
        return;
    }
    
    const canvasWidth = canvas.width;
    const canvasHeight = canvas.height;
    
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    
    // If we have actual image data, try to render it
    if (imageData && imageInfo.width && imageInfo.height) {
        try {
            drawActualImageData(ctx, imageInfo, imageData, canvasWidth, canvasHeight);
            return;
        } catch (error) {
            console.warn('Failed to draw actual image data, falling back to placeholder:', error);
        }
    }
    
    // Fallback to placeholder pattern
    drawImagePlaceholder(ctx, imageInfo, canvasWidth, canvasHeight);
}

function drawActualImageData(ctx, imageInfo, imageData, canvasWidth, canvasHeight) {
    const { width, height, encoding } = imageInfo;
    
    // Create ImageData object for the canvas
    const imageDataObj = ctx.createImageData(width, height);
    const data = imageDataObj.data;
    
    // Convert ROS image data based on encoding
    switch (encoding) {
        case 'rgb8':
            convertRGB8ToImageData(imageData, data, width, height);
            break;
        case 'bgr8':
            convertBGR8ToImageData(imageData, data, width, height);
            break;
        case 'mono8':
            convertMono8ToImageData(imageData, data, width, height);
            break;
        case 'rgba8':
            convertRGBA8ToImageData(imageData, data, width, height);
            break;
        case 'bgra8':
            convertBGRA8ToImageData(imageData, data, width, height);
            break;
        default:
            console.warn(`Unsupported image encoding: ${encoding}`);
            throw new Error(`Unsupported encoding: ${encoding}`);
    }
    
    // Create a temporary canvas to draw the full-size image
    const tempCanvas = document.createElement('canvas');
    tempCanvas.width = width;
    tempCanvas.height = height;
    const tempCtx = tempCanvas.getContext('2d');
    
    // Put the image data on the temporary canvas
    tempCtx.putImageData(imageDataObj, 0, 0);
    
    // Scale and draw on the main canvas
    const scale = Math.min(canvasWidth / width, canvasHeight / height);
    const scaledWidth = width * scale;
    const scaledHeight = height * scale;
    const offsetX = (canvasWidth - scaledWidth) / 2;
    const offsetY = (canvasHeight - scaledHeight) / 2;
    
    ctx.drawImage(tempCanvas, offsetX, offsetY, scaledWidth, scaledHeight);
    
    // Add image info overlay
    drawImageInfoOverlay(ctx, imageInfo, canvasWidth, canvasHeight);
    
    console.log('Successfully drew actual image data');
}

function convertRGB8ToImageData(rosImageData, canvasData, width, height) {
    for (let i = 0; i < width * height; i++) {
        const srcIdx = i * 3;
        const dstIdx = i * 4;
        
        canvasData[dstIdx] = rosImageData[srcIdx];     // R
        canvasData[dstIdx + 1] = rosImageData[srcIdx + 1]; // G
        canvasData[dstIdx + 2] = rosImageData[srcIdx + 2]; // B
        canvasData[dstIdx + 3] = 255; // Alpha
    }
}

function convertBGR8ToImageData(rosImageData, canvasData, width, height) {
    for (let i = 0; i < width * height; i++) {
        const srcIdx = i * 3;
        const dstIdx = i * 4;
        
        canvasData[dstIdx] = rosImageData[srcIdx + 2];     // R (from B)
        canvasData[dstIdx + 1] = rosImageData[srcIdx + 1]; // G
        canvasData[dstIdx + 2] = rosImageData[srcIdx];     // B (from R)
        canvasData[dstIdx + 3] = 255; // Alpha
    }
}

function convertMono8ToImageData(rosImageData, canvasData, width, height) {
    for (let i = 0; i < width * height; i++) {
        const gray = rosImageData[i];
        const dstIdx = i * 4;
        
        canvasData[dstIdx] = gray;     // R
        canvasData[dstIdx + 1] = gray; // G
        canvasData[dstIdx + 2] = gray; // B
        canvasData[dstIdx + 3] = 255;  // Alpha
    }
}

function convertRGBA8ToImageData(rosImageData, canvasData, width, height) {
    for (let i = 0; i < width * height * 4; i++) {
        canvasData[i] = rosImageData[i];
    }
}

function convertBGRA8ToImageData(rosImageData, canvasData, width, height) {
    for (let i = 0; i < width * height; i++) {
        const srcIdx = i * 4;
        const dstIdx = i * 4;
        
        canvasData[dstIdx] = rosImageData[srcIdx + 2];     // R (from B)
        canvasData[dstIdx + 1] = rosImageData[srcIdx + 1]; // G
        canvasData[dstIdx + 2] = rosImageData[srcIdx];     // B (from R)
        canvasData[dstIdx + 3] = rosImageData[srcIdx + 3]; // Alpha
    }
}

function drawImagePlaceholder(ctx, imageInfo, canvasWidth, canvasHeight) {
    // Draw a checkerboard pattern as placeholder
    const cellSize = 20;
    const cols = Math.floor(canvasWidth / cellSize);
    const rows = Math.floor(canvasHeight / cellSize);
    
    for (let row = 0; row < rows; row++) {
        for (let col = 0; col < cols; col++) {
            const x = col * cellSize;
            const y = row * cellSize;
            
            const isEven = (row + col) % 2 === 0;
            ctx.fillStyle = isEven ? '#f0f0f0' : '#e0e0e0';
            ctx.fillRect(x, y, cellSize, cellSize);
        }
    }
    
    // Add animated pattern overlay
    const time = Date.now() * 0.002;
    for (let row = 0; row < rows; row++) {
        for (let col = 0; col < cols; col++) {
            const x = col * cellSize;
            const y = row * cellSize;
            
            const intensity = Math.sin((col + row) * 0.5 + time) * 0.3 + 0.7;
            
            if (imageInfo.encoding === 'rgb8' || imageInfo.encoding === 'bgr8') {
                const r = Math.floor(intensity * 255);
                const g = Math.floor(Math.sin(col * 0.3 + time) * 127 + 128);
                const b = Math.floor(Math.sin(row * 0.3 + time) * 127 + 128);
                ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.6)`;
            } else {
                const gray = Math.floor(intensity * 255);
                ctx.fillStyle = `rgba(${gray}, ${gray}, ${gray}, 0.6)`;
            }
            
            ctx.fillRect(x + 2, y + 2, cellSize - 4, cellSize - 4);
        }
    }
    
    drawImageInfoOverlay(ctx, imageInfo, canvasWidth, canvasHeight);
}

function drawImageInfoOverlay(ctx, imageInfo, canvasWidth, canvasHeight) {
    // Draw image info text overlay
    ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
    ctx.fillRect(5, 5, 220, 80);
    
    ctx.fillStyle = 'white';
    ctx.font = 'bold 14px monospace';
    ctx.fillText(`📸 IMAGE DATA`, 10, 25);
    ctx.font = '12px monospace';
    ctx.fillText(`Size: ${imageInfo.width}x${imageInfo.height}`, 10, 45);
    ctx.fillText(`Format: ${imageInfo.encoding}`, 10, 60);
    ctx.fillText(`Data: ${imageInfo.dataLength} bytes`, 10, 75);
    
    // Add camera icon representation in corner
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 3;
    ctx.strokeRect(canvasWidth - 50, 10, 40, 25);
    ctx.fillStyle = 'white';
    ctx.fillRect(canvasWidth - 60, 15, 8, 15);
    
    // Add center crosshair
    ctx.strokeStyle = 'red';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(canvasWidth/2 - 10, canvasHeight/2);
    ctx.lineTo(canvasWidth/2 + 10, canvasHeight/2);
    ctx.moveTo(canvasWidth/2, canvasHeight/2 - 10);
    ctx.lineTo(canvasWidth/2, canvasHeight/2 + 10);
    ctx.stroke();
}

// Update the drawImageVisualization function to add more debugging
function drawImageVisualization(canvasId, imageInfo) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) {
        console.error('Canvas not found:', canvasId);
        return;
    }
    
    console.log('Drawing image visualization for:', canvasId, imageInfo);
    
    const ctx = canvas.getContext('2d');
    if (!ctx) {
        console.error('Could not get 2D context for canvas:', canvasId);
        return;
    }
    
    const width = canvas.width;
    const height = canvas.height;
    
    ctx.clearRect(0, 0, width, height);
    
    // Draw a more visible placeholder pattern
    const cellSize = 20;
    const cols = Math.floor(width / cellSize);
    const rows = Math.floor(height / cellSize);
    
    // Create a checkerboard pattern as a base
    for (let row = 0; row < rows; row++) {
        for (let col = 0; col < cols; col++) {
            const x = col * cellSize;
            const y = row * cellSize;
            
            // Checkerboard pattern
            const isEven = (row + col) % 2 === 0;
            ctx.fillStyle = isEven ? '#f0f0f0' : '#e0e0e0';
            ctx.fillRect(x, y, cellSize, cellSize);
        }
    }
    
    // Add animated pattern overlay
    const time = Date.now() * 0.002;
    for (let row = 0; row < rows; row++) {
        for (let col = 0; col < cols; col++) {
            const x = col * cellSize;
            const y = row * cellSize;
            
            // Create a wave pattern
            const intensity = Math.sin((col + row) * 0.5 + time) * 0.3 + 0.7;
            
            if (imageInfo.encoding === 'rgb8' || imageInfo.encoding === 'bgr8') {
                // Color pattern for color images
                const r = Math.floor(intensity * 255);
                const g = Math.floor(Math.sin(col * 0.3 + time) * 127 + 128);
                const b = Math.floor(Math.sin(row * 0.3 + time) * 127 + 128);
                ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.6)`;
            } else {
                // Grayscale pattern
                const gray = Math.floor(intensity * 255);
                ctx.fillStyle = `rgba(${gray}, ${gray}, ${gray}, 0.6)`;
            }
            
            ctx.fillRect(x + 2, y + 2, cellSize - 4, cellSize - 4);
        }
    }
    
    // Draw image info text overlay with better visibility
    ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
    ctx.fillRect(5, 5, 220, 80);
    
    ctx.fillStyle = 'white';
    ctx.font = 'bold 14px monospace';
    ctx.fillText(`📸 IMAGE DATA`, 10, 25);
    ctx.font = '12px monospace';
    ctx.fillText(`Size: ${imageInfo.width}x${imageInfo.height}`, 10, 45);
    ctx.fillText(`Format: ${imageInfo.encoding}`, 10, 60);
    ctx.fillText(`Data: ${imageInfo.dataLength} bytes`, 10, 75);
    
    // Add camera icon representation in corner
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 3;
    ctx.strokeRect(width - 50, 10, 40, 25);
    ctx.fillStyle = 'white';
    ctx.fillRect(width - 60, 15, 8, 15);
    
    // Add center crosshair
    ctx.strokeStyle = 'red';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(width/2 - 10, height/2);
    ctx.lineTo(width/2 + 10, height/2);
    ctx.moveTo(width/2, height/2 - 10);
    ctx.lineTo(width/2, height/2 + 10);
    ctx.stroke();
    
    console.log('Image visualization drawn successfully');
}

function drawOccupancyGrid(canvasId, gridData) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const canvasWidth = canvas.width;
    const canvasHeight = canvas.height;
    
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    
    const gridWidth = gridData.width;
    const gridHeight = gridData.height;
    const data = gridData.data;
    
    if (!gridWidth || !gridHeight || !data.length) {
        // Draw placeholder if no valid data
        ctx.fillStyle = '#f0f0f0';
        ctx.fillRect(0, 0, canvasWidth, canvasHeight);
        ctx.fillStyle = '#666';
        ctx.font = '14px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('No grid data', canvasWidth/2, canvasHeight/2);
        return;
    }
    
    // Calculate cell size to fit canvas
    const cellWidth = canvasWidth / gridWidth;
    const cellHeight = canvasHeight / gridHeight;
    
    // Draw grid cells
    for (let y = 0; y < gridHeight; y++) {
        for (let x = 0; x < gridWidth; x++) {
            const index = y * gridWidth + x;
            if (index >= data.length) continue;
            
            const value = data[index];
            let color;
            
            if (value < 0) {
                // Unknown
                color = '#808080';
            } else if (value <= 50) {
                // Free space (white to light gray)
                const intensity = 255 - (value * 2);
                color = `rgb(${intensity}, ${intensity}, ${intensity})`;
            } else {
                // Occupied (dark gray to black)
                const intensity = 255 - value;
                color = `rgb(${intensity}, ${intensity}, ${intensity})`;
            }
            
            ctx.fillStyle = color;
            ctx.fillRect(
                x * cellWidth,
                y * cellHeight,
                Math.ceil(cellWidth),
                Math.ceil(cellHeight)
            );
        }
    }
    
    // Draw grid border
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 1;
    ctx.strokeRect(0, 0, canvasWidth, canvasHeight);
    
    // Add coordinate indicators
    ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.fillRect(5, 5, 80, 40);
    ctx.fillStyle = '#333';
    ctx.font = '10px monospace';
    ctx.fillText(`${gridWidth}x${gridHeight}`, 8, 18);
    ctx.fillText(`${gridData.resolution.toFixed(3)}m/cell`, 8, 32);
}