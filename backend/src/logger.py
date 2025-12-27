"""Structured logging configuration with JSON formatting.

This module provides a structured logger that outputs JSON-formatted logs
with required fields: timestamp, level, message, request_id, endpoint, duration_ms.
"""

import logging
import json
import sys
from datetime import datetime
from typing import Any, Dict, Optional


class StructuredFormatter(logging.Formatter):
    """Custom formatter that outputs structured JSON logs.

    All logs include timestamp, level, and message. Additional fields
    can be passed via the extra parameter in logging calls.
    """

    def format(self, record: logging.LogRecord) -> str:
        """Format a log record as JSON.

        Args:
            record: The log record to format

        Returns:
            JSON-formatted log string
        """
        # Base log structure with required fields
        log_data: Dict[str, Any] = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": record.levelname,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }

        # Add optional fields from extra parameter
        # Filter out built-in logging attributes
        builtin_attrs = {
            'name', 'msg', 'args', 'created', 'filename', 'funcName',
            'levelname', 'levelno', 'lineno', 'module', 'msecs',
            'pathname', 'process', 'processName', 'relativeCreated',
            'thread', 'threadName', 'exc_info', 'exc_text', 'stack_info'
        }

        for key, value in record.__dict__.items():
            if key not in builtin_attrs and not key.startswith('_'):
                log_data[key] = value

        # Include exception info if present
        if record.exc_info:
            log_data["exception"] = self.formatException(record.exc_info)

        return json.dumps(log_data, default=str)


class StructuredLogger:
    """Structured logger with JSON output and contextual fields.

    Provides methods for logging with automatic inclusion of contextual
    information like request_id, endpoint, and duration_ms.
    """

    def __init__(self, name: str = "ragchatbot"):
        """Initialize the structured logger.

        Args:
            name: Logger name (default: "ragchatbot")
        """
        self.logger = logging.getLogger(name)

        # Only add handler if not already present
        if not self.logger.handlers:
            handler = logging.StreamHandler(sys.stdout)
            handler.setFormatter(StructuredFormatter())
            self.logger.addHandler(handler)
            self.logger.setLevel(logging.INFO)

        # Prevent log propagation to avoid duplicate logs
        self.logger.propagate = False

    def info(
        self,
        message: str,
        request_id: Optional[str] = None,
        endpoint: Optional[str] = None,
        duration_ms: Optional[float] = None,
        **kwargs: Any
    ) -> None:
        """Log an info message with optional contextual fields.

        Args:
            message: The log message
            request_id: Optional request ID for tracing
            endpoint: Optional API endpoint
            duration_ms: Optional duration in milliseconds
            **kwargs: Additional fields to include in the log
        """
        extra = self._build_extra(request_id, endpoint, duration_ms, **kwargs)
        self.logger.info(message, extra=extra)

    def warning(
        self,
        message: str,
        request_id: Optional[str] = None,
        endpoint: Optional[str] = None,
        duration_ms: Optional[float] = None,
        **kwargs: Any
    ) -> None:
        """Log a warning message with optional contextual fields.

        Args:
            message: The log message
            request_id: Optional request ID for tracing
            endpoint: Optional API endpoint
            duration_ms: Optional duration in milliseconds
            **kwargs: Additional fields to include in the log
        """
        extra = self._build_extra(request_id, endpoint, duration_ms, **kwargs)
        self.logger.warning(message, extra=extra)

    def error(
        self,
        message: str,
        request_id: Optional[str] = None,
        endpoint: Optional[str] = None,
        duration_ms: Optional[float] = None,
        **kwargs: Any
    ) -> None:
        """Log an error message with optional contextual fields.

        Args:
            message: The log message
            request_id: Optional request ID for tracing
            endpoint: Optional API endpoint
            duration_ms: Optional duration in milliseconds
            **kwargs: Additional fields to include in the log
        """
        extra = self._build_extra(request_id, endpoint, duration_ms, **kwargs)
        self.logger.error(message, extra=extra)

    def debug(
        self,
        message: str,
        request_id: Optional[str] = None,
        endpoint: Optional[str] = None,
        duration_ms: Optional[float] = None,
        **kwargs: Any
    ) -> None:
        """Log a debug message with optional contextual fields.

        Args:
            message: The log message
            request_id: Optional request ID for tracing
            endpoint: Optional API endpoint
            duration_ms: Optional duration in milliseconds
            **kwargs: Additional fields to include in the log
        """
        extra = self._build_extra(request_id, endpoint, duration_ms, **kwargs)
        self.logger.debug(message, extra=extra)

    def _build_extra(
        self,
        request_id: Optional[str],
        endpoint: Optional[str],
        duration_ms: Optional[float],
        **kwargs: Any
    ) -> Dict[str, Any]:
        """Build the extra fields dictionary for logging.

        Args:
            request_id: Optional request ID
            endpoint: Optional endpoint
            duration_ms: Optional duration
            **kwargs: Additional fields

        Returns:
            Dictionary of extra fields
        """
        extra: Dict[str, Any] = {}

        if request_id is not None:
            extra["request_id"] = request_id
        if endpoint is not None:
            extra["endpoint"] = endpoint
        if duration_ms is not None:
            extra["duration_ms"] = duration_ms

        # Add any additional fields
        extra.update(kwargs)

        return extra


# Global logger instance
logger = StructuredLogger("ragchatbot")
