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