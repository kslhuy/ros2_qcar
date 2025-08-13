#!/usr/bin/env python3
"""
Quick script to enable console logging for debugging.
Run this before starting your vehicle simulation to see logs in terminal.
"""

from md_logging_config import enable_console_logging

# Enable console output for all loggers
enable_console_logging(True)

print("Console logging enabled!")
print("Now run your vehicle simulation and you'll see logs in the terminal.")
print("Log files are still being created in the 'logs/' directory.")
