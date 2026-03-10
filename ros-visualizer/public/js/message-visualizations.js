// Message visualization and display - Handle ROS2 message visualization using class-based approach

// Base class for all message visualizations
class MessageVisualization {
    constructor(vizId, topicName, messageType) {
        this.vizId = vizId;
        this.topicName = topicName;
        this.messageType = messageType;
        this.initialized = false;
    }

    // Abstract method to be implemented by subclasses
    generateInitialHTML() {
        throw new Error('generateInitialHTML must be implemented by subclass');
    }

    // Abstract method to be implemented by subclasses
    updateWithMessage(message) {
        throw new Error('updateWithMessage must be implemented by subclass');
    }

    // Common method to check if DOM elements exist
    getElement(elementId) {
        return document.getElementById(elementId);
    }

    // Common method to update element content safely
    updateElement(elementId, content) {
        const element = this.getElement(elementId);
        if (element) {
            element.innerHTML = content;
        }
    }

    // Common method to update element text safely
    updateElementText(elementId, text) {
        const element = this.getElement(elementId);
        if (element) {
            element.textContent = text;
        }
    }
}

// Twist message visualization class
class TwistVisualization extends MessageVisualization {
    constructor(vizId, topicName) {
        super(vizId, topicName, 'geometry_msgs/Twist');
        this.canvasId = `${vizId}-direction`;
    }

    generateInitialHTML() {
        return `
            <div class="row">
                <div class="col-md-6">
                    <h6><i class="bi bi-arrow-up-right"></i> Linear Velocity</h6>
                    <div class="velocity-display">
                        <div class="velocity-bar">
                            <label id="${this.vizId}-linear-x-label">X: 0.000 m/s</label>
                            <div class="progress mb-1">
                                <div id="${this.vizId}-linear-x-bar" class="progress-bar bg-success" style="width: 0%"></div>
                            </div>
                        </div>
                        <div class="velocity-bar">
                            <label id="${this.vizId}-linear-y-label">Y: 0.000 m/s</label>
                            <div class="progress mb-1">
                                <div id="${this.vizId}-linear-y-bar" class="progress-bar bg-info" style="width: 0%"></div>
                            </div>
                        </div>
                        <div class="velocity-bar">
                            <label id="${this.vizId}-linear-z-label">Z: 0.000 m/s</label>
                            <div class="progress mb-1">
                                <div id="${this.vizId}-linear-z-bar" class="progress-bar bg-warning" style="width: 0%"></div>
                            </div>
                        </div>
                    </div>
                </div>
                <div class="col-md-6">
                    <h6><i class="bi bi-arrow-clockwise"></i> Angular Velocity</h6>
                    <div class="velocity-display">
                        <div class="velocity-bar">
                            <label id="${this.vizId}-angular-x-label">X: 0.000 rad/s</label>
                            <div class="progress mb-1">
                                <div id="${this.vizId}-angular-x-bar" class="progress-bar bg-danger" style="width: 0%"></div>
                            </div>
                        </div>
                        <div class="velocity-bar">
                            <label id="${this.vizId}-angular-y-label">Y: 0.000 rad/s</label>
                            <div class="progress mb-1">
                                <div id="${this.vizId}-angular-y-bar" class="progress-bar bg-primary" style="width: 0%"></div>
                            </div>
                        </div>
                        <div class="velocity-bar">
                            <label id="${this.vizId}-angular-z-label">Z: 0.000 rad/s</label>
                            <div class="progress mb-1">
                                <div id="${this.vizId}-angular-z-bar" class="progress-bar bg-secondary" style="width: 0%"></div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="mt-2">
                <canvas id="${this.canvasId}" width="200" height="200" class="border"></canvas>
            </div>
        `;
    }

    updateWithMessage(message) {
        const linear = message.linear || {};
        const angular = message.angular || {};

        // Update linear velocity displays
        this.updateElementText(`${this.vizId}-linear-x-label`, `X: ${(linear.x || 0).toFixed(3)} m/s`);
        this.updateElementText(`${this.vizId}-linear-y-label`, `Y: ${(linear.y || 0).toFixed(3)} m/s`);
        this.updateElementText(`${this.vizId}-linear-z-label`, `Z: ${(linear.z || 0).toFixed(3)} m/s`);

        // Update linear velocity bars
        const linearXElement = this.getElement(`${this.vizId}-linear-x-bar`);
        const linearYElement = this.getElement(`${this.vizId}-linear-y-bar`);
        const linearZElement = this.getElement(`${this.vizId}-linear-z-bar`);

        if (linearXElement) linearXElement.style.width = `${Math.abs(linear.x || 0) * 50}%`;
        if (linearYElement) linearYElement.style.width = `${Math.abs(linear.y || 0) * 50}%`;
        if (linearZElement) linearZElement.style.width = `${Math.abs(linear.z || 0) * 50}%`;

        // Update angular velocity displays
        this.updateElementText(`${this.vizId}-angular-x-label`, `X: ${(angular.x || 0).toFixed(3)} rad/s`);
        this.updateElementText(`${this.vizId}-angular-y-label`, `Y: ${(angular.y || 0).toFixed(3)} rad/s`);
        this.updateElementText(`${this.vizId}-angular-z-label`, `Z: ${(angular.z || 0).toFixed(3)} rad/s`);

        // Update angular velocity bars
        const angularXElement = this.getElement(`${this.vizId}-angular-x-bar`);
        const angularYElement = this.getElement(`${this.vizId}-angular-y-bar`);
        const angularZElement = this.getElement(`${this.vizId}-angular-z-bar`);

        if (angularXElement) angularXElement.style.width = `${Math.abs(angular.x || 0) * 30}%`;
        if (angularYElement) angularYElement.style.width = `${Math.abs(angular.y || 0) * 30}%`;
        if (angularZElement) angularZElement.style.width = `${Math.abs(angular.z || 0) * 30}%`;

        // Update canvas visualization
        setTimeout(() => {
            this.drawTwistDirection(linear.x || 0, angular.z || 0);
        }, 10);
    }

    drawTwistDirection(linearX, angularZ) {
        const canvas = this.getElement(this.canvasId);
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
}

// LaserScan visualization class
class LaserScanVisualization extends MessageVisualization {
    constructor(vizId, topicName) {
        super(vizId, topicName, 'sensor_msgs/LaserScan');
        this.canvasId = `${vizId}-lidar`;
    }

    generateInitialHTML() {
        return `
            <div class="row">
                <div class="col-md-6">
                    <h6><i class="bi bi-broadcast"></i> Scan Properties</h6>
                    <div class="small">
                        <div>Range: <span id="${this.vizId}-range-min">0</span> - <span id="${this.vizId}-range-max">0</span> m</div>
                        <div>Angle: <span id="${this.vizId}-angle-min">0</span> - <span id="${this.vizId}-angle-max">0</span> rad</div>
                        <div>Increment: <span id="${this.vizId}-angle-increment">0</span> rad</div>
                        <div>Points: <span id="${this.vizId}-scan-points">0</span></div>
                    </div>
                </div>
                <div class="col-md-6">
                    <h6><i class="bi bi-speedometer2"></i> Range Stats</h6>
                    <div class="small">
                        <div>Min: <span id="${this.vizId}-min-range">0</span> m</div>
                        <div>Max: <span id="${this.vizId}-max-range">0</span> m</div>
                        <div>Avg: <span id="${this.vizId}-avg-range">0</span> m</div>
                        <div>Valid: <span id="${this.vizId}-valid-ranges">0</span>%</div>
                    </div>
                </div>
            </div>
            <div class="mt-2">
                <canvas id="${this.canvasId}" width="400" height="300" class="border"></canvas>
            </div>
        `;
    }

    updateWithMessage(message) {
        const ranges = message.ranges || [];
        const rangeMin = message.range_min || 0;
        const rangeMax = message.range_max || 0;
        const angleMin = message.angle_min || 0;
        const angleMax = message.angle_max || 0;
        const angleIncrement = message.angle_increment || 0;

        // Update scan properties
        this.updateElementText(`${this.vizId}-range-min`, rangeMin.toFixed(2));
        this.updateElementText(`${this.vizId}-range-max`, rangeMax.toFixed(2));
        this.updateElementText(`${this.vizId}-angle-min`, angleMin.toFixed(3));
        this.updateElementText(`${this.vizId}-angle-max`, angleMax.toFixed(3));
        this.updateElementText(`${this.vizId}-angle-increment`, angleIncrement.toFixed(4));
        this.updateElementText(`${this.vizId}-scan-points`, ranges.length.toString());

        // Calculate statistics
        if (ranges.length > 0) {
            const validRanges = ranges.filter(r => !isNaN(r) && isFinite(r) && r >= rangeMin && r <= rangeMax);
            const minRange = validRanges.length > 0 ? Math.min(...validRanges) : 0;
            const maxRange = validRanges.length > 0 ? Math.max(...validRanges) : 0;
            const avgRange = validRanges.length > 0 ? validRanges.reduce((a, b) => a + b, 0) / validRanges.length : 0;
            const validPercent = (validRanges.length / ranges.length) * 100;

            this.updateElementText(`${this.vizId}-min-range`, minRange.toFixed(2));
            this.updateElementText(`${this.vizId}-max-range`, maxRange.toFixed(2));
            this.updateElementText(`${this.vizId}-avg-range`, avgRange.toFixed(2));
            this.updateElementText(`${this.vizId}-valid-ranges`, validPercent.toFixed(1));
        }

        // Update canvas visualization
        setTimeout(() => {
            this.drawLaserScan(ranges, rangeMin, rangeMax, angleMin, angleIncrement);
        }, 10);
    }

    drawLaserScan(ranges, rangeMin, rangeMax, angleMin, angleIncrement) {
        const canvas = this.getElement(this.canvasId);
        if (!canvas) return;
        
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height * 0.9;
        const scale = Math.min(canvas.width, canvas.height) / (rangeMax * 2.5);
        
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw range circles
        ctx.strokeStyle = '#e0e0e0';
        ctx.lineWidth = 1;
        for (let r = 1; r <= Math.floor(rangeMax); r++) {
            ctx.beginPath();
            ctx.arc(centerX, centerY, r * scale, 0, 2 * Math.PI);
            ctx.stroke();
        }
        
        // Draw robot
        ctx.beginPath();
        ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
        ctx.fillStyle = '#007bff';
        ctx.fill();
        
        // Draw scan points
        ctx.fillStyle = '#dc3545';
        for (let i = 0; i < ranges.length; i++) {
            const range = ranges[i];
            if (!isNaN(range) && isFinite(range) && range >= rangeMin && range <= rangeMax) {
                const angle = angleMin + i * angleIncrement;
                const x = centerX + range * scale * Math.cos(angle);
                const y = centerY - range * scale * Math.sin(angle);
                
                ctx.beginPath();
                ctx.arc(x, y, 2, 0, 2 * Math.PI);
                ctx.fill();
            }
        }
    }
}

// Image visualization class
class ImageVisualization extends MessageVisualization {
    constructor(vizId, topicName) {
        super(vizId, topicName, 'sensor_msgs/Image');
        this.canvasId = `${vizId}-image`;
        this.lastImageData = null;
    }

    generateInitialHTML() {
        return `
            <div class="row">
                <div class="col-md-8">
                    <h6><i class="bi bi-camera"></i> Image Data</h6>
                    <div class="small">
                        <div>Size: <span id="${this.vizId}-width">0</span> x <span id="${this.vizId}-height">0</span></div>
                        <div>Encoding: <span id="${this.vizId}-encoding">-</span></div>
                        <div>Step: <span id="${this.vizId}-step">0</span> bytes</div>
                    </div>
                </div>
                <div class="col-md-4">
                    <h6><i class="bi bi-info-circle"></i> Status</h6>
                    <div class="small">
                        <div>Is Big Endian: <span id="${this.vizId}-big-endian">-</span></div>
                        <div>Data Size: <span id="${this.vizId}-data-size">0</span> bytes</div>
                    </div>
                </div>
            </div>
            <div class="mt-2">
                <canvas id="${this.canvasId}" width="400" height="300" class="border"></canvas>
            </div>
        `;
    }

    updateWithMessage(message) {
        const width = message.width || 0;
        const height = message.height || 0;
        const encoding = message.encoding || '';
        const step = message.step || 0;
        const isBigEndian = message.is_bigendian || false;
        const data = message.data || '';

        // Update image properties
        this.updateElementText(`${this.vizId}-width`, width.toString());
        this.updateElementText(`${this.vizId}-height`, height.toString());
        this.updateElementText(`${this.vizId}-encoding`, encoding);
        this.updateElementText(`${this.vizId}-step`, step.toString());
        this.updateElementText(`${this.vizId}-big-endian`, isBigEndian ? 'Yes' : 'No');
        this.updateElementText(`${this.vizId}-data-size`, data.length.toString());

        // Update canvas with image data
        if (width > 0 && height > 0 && data) {
            setTimeout(() => {
                this.drawImage(width, height, encoding, data);
            }, 10);
        }
    }

    drawImage(width, height, encoding, base64Data) {
        const canvas = this.getElement(this.canvasId);
        if (!canvas) return;

        // Check if image data changed to avoid unnecessary redraws
        if (this.lastImageData === base64Data) {
            return;
        }
        this.lastImageData = base64Data;

        try {
            if (encoding === 'rgb8' || encoding === 'bgr8') {
                this.drawRGBImage(canvas, width, height, encoding, base64Data);
            } else if (encoding === 'mono8') {
                this.drawMonoImage(canvas, width, height, base64Data);
            } else {
                // Try to decode as JPEG or PNG
                this.drawCompressedImage(canvas, base64Data);
            }
        } catch (error) {
            console.error('Error drawing image:', error);
            const ctx = canvas.getContext('2d');
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = '#6c757d';
            ctx.font = '14px Arial';
            ctx.textAlign = 'center';
            ctx.fillText('Error displaying image', canvas.width / 2, canvas.height / 2);
        }
    }

    drawRGBImage(canvas, width, height, encoding, base64Data) {
        const ctx = canvas.getContext('2d');
        const binaryString = atob(base64Data);
        const bytes = new Uint8ClampedArray(binaryString.length);
        
        for (let i = 0; i < binaryString.length; i++) {
            bytes[i] = binaryString.charCodeAt(i);
        }

        const imageData = ctx.createImageData(width, height);
        const isRGB = encoding === 'rgb8';
        
        for (let i = 0; i < width * height; i++) {
            const srcIndex = i * 3;
            const dstIndex = i * 4;
            
            if (isRGB) {
                imageData.data[dstIndex] = bytes[srcIndex];     // R
                imageData.data[dstIndex + 1] = bytes[srcIndex + 1]; // G
                imageData.data[dstIndex + 2] = bytes[srcIndex + 2]; // B
            } else {
                imageData.data[dstIndex] = bytes[srcIndex + 2];     // B
                imageData.data[dstIndex + 1] = bytes[srcIndex + 1]; // G
                imageData.data[dstIndex + 2] = bytes[srcIndex];     // R
            }
            imageData.data[dstIndex + 3] = 255; // A
        }

        canvas.width = width;
        canvas.height = height;
        ctx.putImageData(imageData, 0, 0);
    }

    drawMonoImage(canvas, width, height, base64Data) {
        const ctx = canvas.getContext('2d');
        const binaryString = atob(base64Data);
        const bytes = new Uint8ClampedArray(binaryString.length);
        
        for (let i = 0; i < binaryString.length; i++) {
            bytes[i] = binaryString.charCodeAt(i);
        }

        const imageData = ctx.createImageData(width, height);
        
        for (let i = 0; i < width * height; i++) {
            const gray = bytes[i];
            const dstIndex = i * 4;
            
            imageData.data[dstIndex] = gray;     // R
            imageData.data[dstIndex + 1] = gray; // G
            imageData.data[dstIndex + 2] = gray; // B
            imageData.data[dstIndex + 3] = 255;  // A
        }

        canvas.width = width;
        canvas.height = height;
        ctx.putImageData(imageData, 0, 0);
    }

    drawCompressedImage(canvas, base64Data) {
        const img = new Image();
        img.onload = () => {
            canvas.width = img.width;
            canvas.height = img.height;
            const ctx = canvas.getContext('2d');
            ctx.drawImage(img, 0, 0);
        };
        img.onerror = () => {
            console.error('Failed to load compressed image');
        };
        img.src = 'data:image/jpeg;base64,' + base64Data;
    }
}

// Default/String visualization class
class DefaultVisualization extends MessageVisualization {
    constructor(vizId, topicName, messageType) {
        super(vizId, topicName, messageType);
    }

    generateInitialHTML() {
        return `
            <div class="row">
                <div class="col-12">
                    <h6><i class="bi bi-code"></i> Raw Message Data</h6>
                    <pre id="${this.vizId}-raw-data" class="small bg-light p-2" style="max-height: 200px; overflow-y: auto; white-space: pre-wrap;"></pre>
                </div>
            </div>
        `;
    }

    updateWithMessage(message) {
        const messageStr = typeof message === 'string' ? message : JSON.stringify(message, null, 2);
        this.updateElementText(`${this.vizId}-raw-data`, messageStr);
    }
}

// Factory function to create appropriate visualization class
function createMessageVisualization(vizId, topicName, messageType) {
    switch (messageType) {
        case 'geometry_msgs/msg/Twist':
        case 'geometry_msgs/Twist':
            return new TwistVisualization(vizId, topicName);
        
        case 'sensor_msgs/msg/LaserScan':
        case 'sensor_msgs/LaserScan':
            return new LaserScanVisualization(vizId, topicName);
        
        case 'sensor_msgs/msg/Image':
        case 'sensor_msgs/Image':
            return new ImageVisualization(vizId, topicName);
        
        default:
            return new DefaultVisualization(vizId, topicName, messageType);
    }
}

// Global map to store active visualization instances
const activeVisualizations = new Map();

function updateTopicMessageDisplay(topicName, message, count) {
    const activeSubscription = activeSubscriptions.get(topicName);
    if (!activeSubscription) return;
    
    const messageType = activeSubscription.messageType;
    const visualizationId = `viz-${topicName.replace(/[^a-zA-Z0-9]/g, '_')}`;
    const displayId = `topic-messages-${topicName.replace(/[^a-zA-Z0-9]/g, '_')}`;
    
    // Get or create topic display container
    let messageDisplay = document.getElementById(displayId);
    if (!messageDisplay) {
        // Create new message display container
        const topicSubscriptionsContainer = document.getElementById('topic-subscriptions-container');
        if (topicSubscriptionsContainer) {
            messageDisplay = document.createElement('div');
            messageDisplay.id = displayId;
            messageDisplay.className = 'topic-message-display card mb-3';
            
            // Create the complete topic display structure
            messageDisplay.innerHTML = `
                <div class="card-header bg-secondary text-white d-flex justify-content-between align-items-center">
                    <h6 class="mb-0">
                        <i class="bi bi-broadcast"></i> ${topicName}
                        <small class="ms-2 opacity-75">${messageType}</small>
                    </h6>
                    <div>
                        <span class="badge bg-light text-dark me-2">Messages: <span id="${displayId}-count">0</span></span>
                        <button onclick="unsubscribeFromTopic('${topicName}')" class="btn btn-sm btn-outline-light">
                            <i class="bi bi-x"></i> Unsubscribe
                        </button>
                    </div>
                </div>
                <div class="card-body">
                    <div id="${visualizationId}-container">
                        <!-- Visualization content will be inserted here -->
                    </div>
                </div>
            `;
            
            topicSubscriptionsContainer.appendChild(messageDisplay);
        } else {
            return; // Container doesn't exist
        }
    }
    
    // Update message count
    const countElement = document.getElementById(`${displayId}-count`);
    if (countElement) {
        countElement.textContent = count.toString();
    }
    
    // Get or create visualization instance
    if (!activeVisualizations.has(topicName)) {
        const visualization = createMessageVisualization(visualizationId, topicName, messageType);
        activeVisualizations.set(topicName, visualization);
        
        // Initialize HTML for new visualization
        const vizContainer = document.getElementById(`${visualizationId}-container`);
        if (vizContainer) {
            vizContainer.innerHTML = visualization.generateInitialHTML();
        }
    }
    
    // Update the visualization with new message
    const visualization = activeVisualizations.get(topicName);
    if (visualization) {
        try {
            visualization.updateWithMessage(message);
        } catch (error) {
            console.error(`Error updating visualization for ${topicName}:`, error);
        }
    }
}

// Clean up visualization when topic is unsubscribed
function cleanupVisualization(topicName) {
    activeVisualizations.delete(topicName);
}