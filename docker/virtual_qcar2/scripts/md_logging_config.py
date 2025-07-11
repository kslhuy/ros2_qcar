import os
import logging
from logging.handlers import RotatingFileHandler

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Create formatter with vehicle_id, defaulting to 'N/A' if not provided
formatter = logging.Formatter('%(asctime)s [%(levelname)s] V%(vehicle_id)s: %(message)s', defaults={'vehicle_id': 'N/A'})

# Console handler (added conditionally based on show_console)
show_console = False  # Default to no console output
if show_console:
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

# Delete or truncate existing platoon.log before each run
log_file = 'platoon.log'
if os.path.exists(log_file):
    try:
        with open(log_file, 'w') as f:
            f.truncate(0)  # Clear the file content
        logger.info(f"Existing {log_file} truncated for new run")
    except Exception as e:
        logger.error(f"Failed to truncate {log_file}: {e}")
        raise

# File handler with rotation (max 5MB per file, keep 3 backups)
file_handler = RotatingFileHandler(log_file, maxBytes=5*1024*1024, backupCount=3)
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)