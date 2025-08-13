import os
import logging
from logging.handlers import RotatingFileHandler
from typing import Dict, Optional

class FleetLoggingConfig:
    """
    Advanced logging configuration for fleet vehicle simulation.
    
    Creates separate log files for:
    - Communication data (send/receive messages)
    - GPS synchronization events
    - Individual vehicle operations
    - General fleet operations
    """
    
    def __init__(self, log_dir: str = "logs", max_bytes: int = 5*1024*1024, backup_count: int = 3):
        """
        Initialize fleet logging configuration.
        
        Args:
            log_dir: Directory to store log files
            max_bytes: Maximum size per log file before rotation (default: 5MB)
            backup_count: Number of backup files to keep (default: 3)
        """
        self.log_dir = log_dir
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        self.show_console = False  # Console output control
        
        # Create logs directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Initialize different loggers
        self.loggers = {}
        self._setup_loggers()
        
    def _setup_loggers(self):
        """Setup all specialized loggers with their respective handlers."""
        
        # Define logger configurations
        logger_configs = {
            'fleet': {
                'filename': 'fleet_operations.log',
                'level': logging.INFO,
                'description': 'General fleet operations and coordination'
            },
            'communication': {
                'filename': 'communication_data.log', 
                'level': logging.INFO,
                'description': 'Data send/receive messages and ACK operations'
            },
            'gps': {
                'filename': 'gps_sync.log',
                'level': logging.INFO, 
                'description': 'GPS synchronization events and time coordination'
            },
            'control': {
                'filename': 'vehicle_control.log',
                'level': logging.INFO,
                'description': 'Vehicle control logic and command execution'
            }
        }
        
        # Create formatters
        self.formatters = {
            'standard': logging.Formatter(
                '%(asctime)s [%(levelname)s] %(name)s: %(message)s'
            ),
            'vehicle': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s: %(message)s', 
                defaults={'vehicle_id': 'N/A'}
            ),
            'communication': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s [COMM]: %(message)s',
                defaults={'vehicle_id': 'N/A'}
            ),
            'gps': logging.Formatter(
                '%(asctime)s [%(levelname)s] V%(vehicle_id)s [GPS]: %(message)s',
                defaults={'vehicle_id': 'N/A'}
            )
        }
        
        # Setup each logger
        for logger_name, config in logger_configs.items():
            self._create_logger(logger_name, config)
            
    def _create_logger(self, logger_name: str, config: dict):
        """Create and configure a specific logger."""
        
        # Create logger
        logger = logging.getLogger(f"fleet.{logger_name}")
        logger.setLevel(config['level'])
        
        # Clear existing handlers to avoid duplicates
        logger.handlers.clear()
        logger.propagate = False
        
        # Create log file path
        log_file_path = os.path.join(self.log_dir, config['filename'])
        
        # Truncate existing log file for fresh start
        if os.path.exists(log_file_path):
            try:
                with open(log_file_path, 'w') as f:
                    f.truncate(0)
                print(f"Truncated existing {config['filename']} for new run")
            except Exception as e:
                print(f"Failed to truncate {config['filename']}: {e}")
        
        # Create rotating file handler
        file_handler = RotatingFileHandler(
            log_file_path, 
            maxBytes=self.max_bytes, 
            backupCount=self.backup_count
        )
        
        # Choose appropriate formatter
        if logger_name == 'communication':
            file_handler.setFormatter(self.formatters['communication'])
        elif logger_name == 'gps':
            file_handler.setFormatter(self.formatters['gps'])
        else:
            file_handler.setFormatter(self.formatters['vehicle'])
        
        logger.addHandler(file_handler)
        
        # Add console handler if enabled
        if self.show_console:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(file_handler.formatter)
            logger.addHandler(console_handler)
        
        # Store logger reference
        self.loggers[logger_name] = logger
        
        # Log initialization
        logger.info(f"Logger '{logger_name}' initialized - {config['description']}")
    
    def get_vehicle_logger(self, vehicle_id: int, subsystem: str = 'fleet') -> logging.LoggerAdapter:
        """
        Get a logger adapter for a specific vehicle and subsystem.
        
        Args:
            vehicle_id: Unique vehicle identifier
            subsystem: Subsystem type ('fleet', 'communication', 'gps', 'control')
            
        Returns:
            LoggerAdapter with vehicle_id context
        """
        if subsystem not in self.loggers:
            subsystem = 'fleet'  # Fallback to fleet logger
            
        base_logger = self.loggers[subsystem]
        return logging.LoggerAdapter(base_logger, {'vehicle_id': vehicle_id})
    
    def get_individual_vehicle_logger(self, vehicle_id: int) -> logging.LoggerAdapter:
        """
        Create a dedicated logger for an individual vehicle with its own log file.
        
        Args:
            vehicle_id: Unique vehicle identifier
            
        Returns:
            LoggerAdapter for the specific vehicle
        """
        logger_name = f"vehicle_{vehicle_id}"
        
        # Create individual vehicle logger if it doesn't exist
        if logger_name not in self.loggers:
            vehicle_logger = logging.getLogger(f"fleet.{logger_name}")
            vehicle_logger.setLevel(logging.INFO)
            vehicle_logger.handlers.clear()
            vehicle_logger.propagate = False
            
            # Create vehicle-specific log file
            log_file_path = os.path.join(self.log_dir, f"vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            file_handler.setFormatter(self.formatters['vehicle'])
            vehicle_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(self.formatters['vehicle'])
                vehicle_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = vehicle_logger
            vehicle_logger.info(f"Individual vehicle logger created for Vehicle {vehicle_id}")
        
        return logging.LoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id})
    
    def enable_console_output(self, enabled: bool = True):
        """Enable or disable console output for all loggers."""
        self.show_console = enabled
        
        for logger in self.loggers.values():
            # Remove existing console handlers
            console_handlers = [h for h in logger.handlers if isinstance(h, logging.StreamHandler) and not isinstance(h, RotatingFileHandler)]
            for handler in console_handlers:
                logger.removeHandler(handler)
            
            # Add console handler if enabled
            if enabled:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(logger.handlers[0].formatter if logger.handlers else self.formatters['standard'])
                logger.addHandler(console_handler)
    
    def get_communication_logger(self, vehicle_id: int) -> logging.LoggerAdapter:
        """
        Get logger specifically for communication events with individual file per vehicle.
        Creates separate communication log files: communication_vehicle_N.log
        """
        logger_name = f"communication_vehicle_{vehicle_id}"
        
        # Create individual communication logger if it doesn't exist
        if logger_name not in self.loggers:
            comm_logger = logging.getLogger(f"fleet.{logger_name}")
            comm_logger.setLevel(logging.INFO)
            comm_logger.handlers.clear()
            comm_logger.propagate = False
            
            # Create vehicle-specific communication log file
            log_file_path = os.path.join(self.log_dir, f"communication_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate communication_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            file_handler.setFormatter(self.formatters['communication'])
            comm_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(self.formatters['communication'])
                comm_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = comm_logger
            comm_logger.info(f"Individual communication logger created for Vehicle {vehicle_id}")
        
        return logging.LoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id})
    
    def get_gps_logger(self, vehicle_id: int) -> logging.LoggerAdapter:
        """
        Get logger specifically for GPS synchronization events with individual file per vehicle.
        Creates separate GPS log files: gps_vehicle_N.log
        """
        logger_name = f"gps_vehicle_{vehicle_id}"
        
        # Create individual GPS logger if it doesn't exist
        if logger_name not in self.loggers:
            gps_logger = logging.getLogger(f"fleet.{logger_name}")
            gps_logger.setLevel(logging.INFO)
            gps_logger.handlers.clear()
            gps_logger.propagate = False
            
            # Create vehicle-specific GPS log file
            log_file_path = os.path.join(self.log_dir, f"gps_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate gps_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            file_handler.setFormatter(self.formatters['gps'])
            gps_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(self.formatters['gps'])
                gps_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = gps_logger
            gps_logger.info(f"Individual GPS logger created for Vehicle {vehicle_id}")
        
        return logging.LoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id})
    
    def get_control_logger(self, vehicle_id: int) -> logging.LoggerAdapter:
        """
        Get logger specifically for control operations with individual file per vehicle.
        Creates separate control log files: control_vehicle_N.log
        """
        logger_name = f"control_vehicle_{vehicle_id}"
        
        # Create individual control logger if it doesn't exist
        if logger_name not in self.loggers:
            control_logger = logging.getLogger(f"fleet.{logger_name}")
            control_logger.setLevel(logging.INFO)
            control_logger.handlers.clear()
            control_logger.propagate = False
            
            # Create vehicle-specific control log file
            log_file_path = os.path.join(self.log_dir, f"control_vehicle_{vehicle_id}.log")
            
            # Truncate existing log file
            if os.path.exists(log_file_path):
                try:
                    with open(log_file_path, 'w') as f:
                        f.truncate(0)
                except Exception as e:
                    print(f"Failed to truncate control_vehicle_{vehicle_id}.log: {e}")
            
            # Setup file handler
            file_handler = RotatingFileHandler(
                log_file_path,
                maxBytes=self.max_bytes,
                backupCount=self.backup_count
            )
            file_handler.setFormatter(self.formatters['vehicle'])  # Use vehicle formatter for control
            control_logger.addHandler(file_handler)
            
            # Add console handler if enabled
            if self.show_console:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(self.formatters['vehicle'])
                control_logger.addHandler(console_handler)
            
            # Store logger
            self.loggers[logger_name] = control_logger
            control_logger.info(f"Individual control logger created for Vehicle {vehicle_id}")
        
        return logging.LoggerAdapter(self.loggers[logger_name], {'vehicle_id': vehicle_id})
    
    def get_fleet_logger(self, vehicle_id: int) -> logging.LoggerAdapter:
        """Get logger for general fleet operations."""
        return self.get_vehicle_logger(vehicle_id, 'fleet')

# Global fleet logging configuration instance
fleet_logging = FleetLoggingConfig()

# Convenience functions for backward compatibility
def get_logger(vehicle_id: int) -> logging.LoggerAdapter:
    """Get general fleet logger for a vehicle (backward compatibility)."""
    return fleet_logging.get_fleet_logger(vehicle_id)

def get_communication_logger(vehicle_id: int) -> logging.LoggerAdapter:
    """Get communication-specific logger for a vehicle."""
    return fleet_logging.get_communication_logger(vehicle_id)

def get_gps_logger(vehicle_id: int) -> logging.LoggerAdapter:
    """Get GPS-specific logger for a vehicle."""
    return fleet_logging.get_gps_logger(vehicle_id)

def get_individual_vehicle_logger(vehicle_id: int) -> logging.LoggerAdapter:
    """Get individual vehicle logger with dedicated log file."""
    return fleet_logging.get_individual_vehicle_logger(vehicle_id)

def get_control_logger(vehicle_id: int) -> logging.LoggerAdapter:
    """Get control-specific logger for a vehicle."""
    return fleet_logging.get_control_logger(vehicle_id)

def enable_console_logging(enabled: bool = True):
    """Enable or disable console output for all loggers."""
    fleet_logging.enable_console_output(enabled)

# Backward compatibility - default logger
logger = fleet_logging.loggers.get('fleet', logging.getLogger(__name__))