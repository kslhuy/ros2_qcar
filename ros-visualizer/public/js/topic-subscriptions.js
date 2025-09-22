// Topic subscription management - Handle ROS2 topic subscriptions

// Global variables for topic management
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

function subscribeToTopic(topicName, messageType) {
    // Check if already subscribed
    if (activeSubscriptions.has(topicName)) {
        if (typeof showParameterNotification === 'function') {
            showParameterNotification(`Already subscribed to ${topicName}`, 'warning');
        }
        return;
    }
    
    if (!ros || !ros.isConnected) {
        if (typeof showParameterNotification === 'function') {
            showParameterNotification('Not connected to ROS2', 'error');
        }
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
        
        // Update the display (if function exists)
        if (typeof updateTopicMessageDisplay === 'function') {
            updateTopicMessageDisplay(topicName, message, count);
        }
    });
    
    // Store the subscription
    activeSubscriptions.set(topicName, {
        listener: listener,
        messageType: messageType,
        startTime: new Date()
    });
    
    // Update the UI
    updateTopicSubscriptionsList();
    if (typeof showParameterNotification === 'function') {
        showParameterNotification(`Subscribed to ${topicName}`, 'success');
    }
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
        
        // Clean up visualization
        if (typeof cleanupVisualization === 'function') {
            cleanupVisualization(topicName);
        }
        
        updateTopicSubscriptionsList();
        if (typeof showParameterNotification === 'function') {
            showParameterNotification(`Unsubscribed from ${topicName}`, 'info');
        }
    }
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
        if (typeof showParameterNotification === 'function') {
            showParameterNotification('Please select or enter a message type', 'warning');
        }
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