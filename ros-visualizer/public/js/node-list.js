// Node list management - Discovery and selection of ROS2 nodes

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
    
    if (nodes && nodes.length > 0) {
        if (nodesList) {
            nodesList.innerHTML = nodes.map(node => 
                `<div class="node-item" data-node-name="${node}" onclick="selectNodeFromList('${node}')">
                    <i class="bi bi-node-plus me-2"></i>${node}
                    <i class="bi bi-chevron-right float-end"></i>
                </div>`
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

function selectNodeFromList(nodeName) {
    // Update visual selection in the nodes list
    const nodeItems = document.querySelectorAll('.node-item');
    nodeItems.forEach(item => {
        item.classList.remove('active');
        if (item.getAttribute('data-node-name') === nodeName) {
            item.classList.add('active');
        }
    });
    
    // Also update the dropdown selection to keep them in sync
    const nodeSelect = document.getElementById('node-select');
    if (nodeSelect) {
        nodeSelect.value = nodeName;
    }
    
    // Load the node information (if function exists)
    if (typeof getNodeParameters === 'function') {
        getNodeParameters(nodeName);
    }
    
    // Load topic subscription options for the node (if function exists)
    if (typeof getTopicsForNode === 'function') {
        getTopicsForNode(nodeName);
    }
}