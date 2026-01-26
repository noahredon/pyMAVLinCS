# pyMAVLinCS/setup_logger.py
# Copyright (C) 2025 Noah Redon
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

import logging
import time
from datetime import datetime
import os

# Global start time for elapsed_ms
_program_start_time = time.monotonic()


# ========= CONSOLE COLORS ========= #
try:
    from colorama import Fore, Style, init as colorama_init
    colorama_init()
    _ENABLE_COLORS = True
except ImportError:
    print("Warning: colorama isn't installed. No logging color available")
    _ENABLE_COLORS = False

_COLORS = {
    logging.DEBUG: Fore.CYAN,
    logging.INFO: Fore.WHITE,
    logging.WARNING: Fore.YELLOW,
    logging.ERROR: Fore.RED,
    logging.CRITICAL: Fore.RED + Style.BRIGHT,
} if _ENABLE_COLORS else {}
_RESET = Style.RESET_ALL if _ENABLE_COLORS else ""


# ========= CONSOLE FORMATTER ========= #
class _ConsoleFormatterNoDebug(logging.Formatter):
    def __init__(self, add_timestamp: bool):
        self.add_timestamp = add_timestamp
        super().__init__("%(message)s")  # Message will be built manually

    def format(self, record):
        timestamp = ""
        if self.add_timestamp:
            timestamp = datetime.now().strftime("%H:%M:%S") + " | "

        if _ENABLE_COLORS or record.levelno == logging.DEBUG or record.levelno == logging.INFO:
            level = ""  # no [DEBUG] or [INFO]
        else:
            level = f"[{record.levelname}]"
        color = _COLORS.get(record.levelno, "") if _ENABLE_COLORS else ""

        msg = f"{timestamp}{color}{level}{_RESET} {record.getMessage()}"
        if record.exc_info:
            # Append formatted traceback
            msg += "\n" + self.formatException(record.exc_info)
        return msg

class _ConsoleFormatterDebug(logging.Formatter):
    def __init__(self, add_timestamp: bool):
        self.add_timestamp = add_timestamp
        super().__init__("%(message)s")  # Message will be built manually

    def format(self, record):
        timestamp = ""
        if self.add_timestamp:
            timestamp = datetime.now().strftime("%H:%M:%S") + " | "

        level = f"[{record.levelname}]"
        color = _COLORS.get(record.levelno, "") if _ENABLE_COLORS else ""

        msg = f"{timestamp}{color}{level}{_RESET} {record.getMessage()}"
        if record.exc_info:
            # Append formatted traceback
            msg += "\n" + self.formatException(record.exc_info)
        return msg


# ========= FILE FORMATTER ========= #
class _FileFormatter(logging.Formatter):
    def __init__(self, add_timestamp: bool):
        self.add_timestamp = add_timestamp
        # Minimal template, everything built manually
        super().__init__("%(message)s")

    def format(self, record):
        # Elapsed time since program start
        elapsed_ms = int((time.monotonic() - _program_start_time) * 1000)

        # Optional readable timestamp
        timestamp = ""
        if self.add_timestamp:
            timestamp = " | " + datetime.now().strftime("%H:%M:%S")

        module_line = f"{record.module}:{record.lineno}"

        msg = (
            f"{elapsed_ms} ms | {module_line}"
            f"{timestamp} | [{record.levelname}] {record.getMessage()}"
        )

        if record.exc_info:
            msg += "\n" + self.formatException(record.exc_info)
        return msg


# ========= LOGGER SETUP ========= #
def setup_logger(
        logger: logging.Logger,
        log_file: str|None = None,
        relative_path_to_cwd: bool = True,
        clear_existing_logs: bool = True,
        debug_mode: bool = False,
        add_timestamp_to_console: bool = False,
        add_timestamp_to_file: bool = False
    ) -> None:
    """Set up the logger.

    Args:
        logger (logging.Logger): Logger to set up.
        log_file (str|None): Log file. None if no file logging is needed.
        relative_path_to_cwd (bool): Path relative to current working directory or absolute.
        clear_existing_logs (bool): Whether to clear existing logs or not.
        debug_mode (bool): Indicates if debug mode is enabled.
        add_timestamp_to_console (bool): Add time to console logs.
        add_timestamp_to_file (bool): Add time to log file entries.
    """
    if logger.hasHandlers():  # logger already configured
        return  # Skip configuration
        
    if debug_mode:
        level = logging.DEBUG
    else:
        level = logging.INFO
    logger.setLevel(level)

    # --- Prevent duplicates: skip if already configured
    existing_handler_names = {h.name for h in logger.handlers}

    # ========= CONSOLE ========= #
    if "console" not in existing_handler_names:
        console_handler = logging.StreamHandler()
        console_handler.name = "console"
        if debug_mode:
            console_handler.setFormatter(_ConsoleFormatterDebug(add_timestamp_to_console))
        else:
            console_handler.setFormatter(_ConsoleFormatterNoDebug(add_timestamp_to_console))
        logger.addHandler(console_handler)

    # ========= FILE ========= #
    if log_file is not None and "file" not in existing_handler_names:
        if relative_path_to_cwd:
            cwd = os.getcwd()
            abs_path = os.path.join(cwd, log_file)
        else:
            abs_path = log_file
        mode = "w" if clear_existing_logs else "a"
        file_handler = logging.FileHandler(abs_path, mode=mode, encoding="utf-8")
        file_handler.name = "file"
        file_handler.setFormatter(_FileFormatter(add_timestamp_to_file))
        logger.addHandler(file_handler)


default_logger = logging.getLogger(__file__)
setup_logger(logger=default_logger)
