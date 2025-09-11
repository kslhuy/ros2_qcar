"""
Configuration file for Semantic Occupancy Grid Mapping System
Modify these parameters to customize the system behavior
"""

# ============================================================================
# OCCUPANCY GRID PARAMETERS
# ============================================================================

# Grid dimensions (meters)
GRID_X_LENGTH = 30.0        # Map width in meters
GRID_Y_LENGTH = 30.0        # Map height in meters
GRID_CELL_WIDTH = 0.2       # Resolution: meters per grid cell

# Probability parameters
PRIOR_PROBABILITY = 0.5     # Prior occupancy probability for unknown cells
SATURATION_LIMIT = 0.001    # Probability saturation limits

# ============================================================================
# YOLO DETECTION PARAMETERS
# ============================================================================

# Image dimensions
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Detection classes (COCO dataset indices)
DETECTION_CLASSES = [
    0,   # person
    1,   # bicycle
    2,   # car
    3,   # motorcycle
    5,   # bus
    7,   # truck
    9,   # traffic light
    11,  # stop sign
    13   # bench
]

# Detection thresholds
YOLO_CONFIDENCE_THRESHOLD = 0.3     # Minimum confidence for detections
YOLO_HALF_PRECISION = True          # Use FP16 for faster inference

# ============================================================================
# SENSOR FUSION PARAMETERS
# ============================================================================

# Camera parameters
CAMERA_FOV_HORIZONTAL = 70.0        # Horizontal field of view (degrees)
CAMERA_FOV_VERTICAL = 45.0          # Vertical field of view (degrees)

# LiDAR parameters
LIDAR_SAMPLE_POINTS = 360           # Number of LiDAR sample points
LIDAR_MIN_DISTANCE = 0.5            # Minimum valid distance (meters)
LIDAR_MAX_DISTANCE = 20.0           # Maximum valid distance (meters)

# Fusion tolerances
ANGULAR_FUSION_TOLERANCE = 15.0     # Degrees tolerance for LiDAR-vision fusion
SEMANTIC_CONFIDENCE_THRESHOLD = 0.3 # Minimum confidence for semantic updates

# ============================================================================
# SEMANTIC MAPPING PARAMETERS
# ============================================================================

# Class names (COCO dataset)
CLASS_NAMES = {
    0: 'person',
    1: 'bicycle', 
    2: 'car',
    3: 'motorcycle',
    4: 'airplane',
    5: 'bus',
    6: 'train',
    7: 'truck',
    8: 'boat',
    9: 'traffic_light',
    10: 'fire_hydrant',
    11: 'stop_sign',
    12: 'parking_meter',
    13: 'bench'
}

# Visualization colors (RGB values 0-255)
CLASS_COLORS = {
    0: [255, 0, 0],      # person - red
    1: [255, 165, 0],    # bicycle - orange
    2: [0, 255, 0],      # car - green
    3: [255, 192, 203],  # motorcycle - pink
    5: [0, 0, 255],      # bus - blue
    7: [255, 255, 0],    # truck - yellow
    9: [255, 0, 255],    # traffic light - magenta
    11: [0, 255, 255],   # stop sign - cyan
    13: [165, 42, 42],   # bench - brown
    -1: [128, 128, 128]  # unknown - gray
}

# Object influence radius (meters)
OBJECT_INFLUENCE_RADIUS = 1.0       # Radius around detected objects to mark as semantic

# ============================================================================
# VISUALIZATION PARAMETERS
# ============================================================================

# Display update rates
MAP_VISUALIZATION_UPDATE_RATE = 10  # Update semantic map every N frames
STATISTICS_PRINT_RATE = 20          # Print statistics every N frames

# Window sizes and positions
CAMERA_WINDOW_SIZE = (800, 600)
LIDAR_PLOT_RANGE = 15.0             # LiDAR plot range (±meters)

# Visualization options
SHOW_DETECTION_BOXES = True         # Show bounding boxes on camera feed
SHOW_DISTANCE_TEXT = True           # Show distance text on detections
SHOW_CONFIDENCE_VALUES = True       # Show confidence values
SAVE_INTERMEDIATE_MAPS = False      # Save maps during processing

# ============================================================================
# PERFORMANCE PARAMETERS
# ============================================================================

# Processing limits
MAX_FRAMES_TO_PROCESS = 300         # Auto-stop after this many frames
FRAME_PROCESSING_DELAY = 0.05       # Seconds between frames

# Memory management
ENABLE_GPU_ACCELERATION = True      # Use GPU if available
OPTIMIZE_MEMORY_USAGE = True        # Enable memory optimizations

# ============================================================================
# FILE PATHS AND NAMING
# ============================================================================

# Output directories
OUTPUT_DIRECTORY = "semantic_ogm_results"
LOG_DIRECTORY = "logs"

# File naming patterns
SEMANTIC_MAP_PREFIX = "semantic_ogm"
FINAL_MAP_PREFIX = "final_semantic_ogm"
LOG_FILE_PREFIX = "semantic_mapping"

# File formats
IMAGE_SAVE_FORMAT = "png"
IMAGE_SAVE_DPI = 300

# ============================================================================
# QCAR ENVIRONMENT PARAMETERS
# ============================================================================

# QCar spawn positions (x, y, z, rotation_z)
OBSERVER_QCAR_POSITION = [-8.700, 14.643, 0.005, 1.5708]  # pi/2 radians
TARGET_QCAR_POSITIONS = [
    [-11.048, 14.643, 0.005, 1.5708],  # Target 1
    [-4.49, 33.189, 0.005, 3.1416],    # Target 2 (pi radians)
]

# Camera setup
OVERVIEW_CAMERA_POSITION = [-15.075, 26.703, 6.074]
OVERVIEW_CAMERA_ROTATION = [0, 0.564, -1.586]

# LED colors for target identification (RGB 0-1)
TARGET_LED_COLORS = [
    [0, 0, 1],  # Blue
    [0, 1, 0],  # Green
]

# ============================================================================
# DEBUG AND LOGGING PARAMETERS
# ============================================================================

# Console output verbosity
VERBOSE_DETECTION_INFO = True       # Print detailed detection information
VERBOSE_FUSION_INFO = True          # Print sensor fusion details
VERBOSE_MAPPING_INFO = True         # Print mapping update information

# Debug visualizations
SHOW_LIDAR_POINTS = True           # Show individual LiDAR points
HIGHLIGHT_FUSED_OBJECTS = True     # Highlight objects in LiDAR view
SHOW_GRID_COORDINATES = False      # Show grid coordinate overlays

# Error handling
CONTINUE_ON_SENSOR_ERROR = True    # Continue if sensors fail
MAX_CONSECUTIVE_ERRORS = 10        # Stop after this many consecutive errors

# ============================================================================
# ADVANCED PARAMETERS
# ============================================================================

# Temporal consistency
ENABLE_TEMPORAL_FILTERING = False   # Smooth detections over time
TEMPORAL_WINDOW_SIZE = 5            # Frames for temporal averaging

# Multi-hypothesis tracking
ENABLE_OBJECT_TRACKING = False      # Track objects between frames
MAX_TRACKING_DISTANCE = 2.0         # Max distance for object association

# Probabilistic updates
USE_BAYESIAN_UPDATES = True         # Use Bayesian probability updates
ENABLE_UNCERTAINTY_ESTIMATION = False  # Estimate mapping uncertainty

# Export options
EXPORT_RAW_DATA = False             # Export raw sensor data
EXPORT_DETECTION_RESULTS = True     # Export detection results
EXPORT_GRID_DATA = False            # Export grid data as numpy arrays
