"""Input validation helpers for RAG Chatbot API.

This module provides validation functions for user inputs,
ensuring data quality and security.
"""

from typing import Optional
from src.utils.exceptions import ValidationError


def validate_query(query: str, max_length: int = 500) -> str:
    """Validate and sanitize a user query.

    Checks that the query is non-empty, not just whitespace,
    and within the maximum allowed length.

    Args:
        query: The user's question/query to validate
        max_length: Maximum allowed length in characters (default: 500)

    Returns:
        The validated and trimmed query string

    Raises:
        ValidationError: If query is empty, whitespace-only, or too long

    Examples:
        >>> validate_query("What is AI?")
        'What is AI?'

        >>> validate_query("  ")
        Traceback (most recent call last):
        ...
        ValidationError: Question cannot be empty

        >>> validate_query("x" * 501)
        Traceback (most recent call last):
        ...
        ValidationError: Question exceeds maximum length of 500 characters
    """
    if not query:
        raise ValidationError(
            "Question cannot be empty",
            details={"field": "query", "reason": "empty_string"}
        )

    # Strip whitespace
    query = query.strip()

    if not query:
        raise ValidationError(
            "Question cannot be empty or contain only whitespace",
            details={"field": "query", "reason": "whitespace_only"}
        )

    if len(query) > max_length:
        raise ValidationError(
            f"Question exceeds maximum length of {max_length} characters",
            details={
                "field": "query",
                "reason": "too_long",
                "current_length": len(query),
                "max_length": max_length
            }
        )

    return query


def validate_url(url: Optional[str], required: bool = False) -> Optional[str]:
    """Validate a URL string.

    Performs basic URL validation to ensure it's well-formed.
    Currently checks for basic structure; can be extended with
    more sophisticated validation as needed.

    Args:
        url: The URL to validate
        required: Whether the URL is required (default: False)

    Returns:
        The validated URL or None if url is None and not required

    Raises:
        ValidationError: If URL is required but missing or if URL is malformed

    Examples:
        >>> validate_url("https://example.com")
        'https://example.com'

        >>> validate_url(None, required=False)
        None

        >>> validate_url(None, required=True)
        Traceback (most recent call last):
        ...
        ValidationError: URL is required
    """
    if url is None:
        if required:
            raise ValidationError(
                "URL is required",
                details={"field": "url", "reason": "missing"}
            )
        return None

    # Strip whitespace
    url = url.strip()

    if not url:
        if required:
            raise ValidationError(
                "URL cannot be empty",
                details={"field": "url", "reason": "empty_string"}
            )
        return None

    # Basic URL validation
    if not (url.startswith("http://") or url.startswith("https://")):
        raise ValidationError(
            "URL must start with http:// or https://",
            details={
                "field": "url",
                "reason": "invalid_scheme",
                "provided_url": url
            }
        )

    # Check for basic malformed URLs
    if " " in url:
        raise ValidationError(
            "URL cannot contain spaces",
            details={"field": "url", "reason": "contains_spaces"}
        )

    if len(url) < 10:  # Minimum realistic URL length
        raise ValidationError(
            "URL is too short to be valid",
            details={"field": "url", "reason": "too_short"}
        )

    return url


def validate_positive_int(
    value: int,
    field_name: str,
    min_value: int = 1,
    max_value: Optional[int] = None
) -> int:
    """Validate that an integer is positive and within bounds.

    Args:
        value: The integer value to validate
        field_name: Name of the field being validated (for error messages)
        min_value: Minimum allowed value (default: 1)
        max_value: Maximum allowed value (default: None for no limit)

    Returns:
        The validated integer value

    Raises:
        ValidationError: If value is not within the specified bounds

    Examples:
        >>> validate_positive_int(5, "count")
        5

        >>> validate_positive_int(0, "count")
        Traceback (most recent call last):
        ...
        ValidationError: count must be at least 1

        >>> validate_positive_int(150, "count", max_value=100)
        Traceback (most recent call last):
        ...
        ValidationError: count cannot exceed 100
    """
    if value < min_value:
        raise ValidationError(
            f"{field_name} must be at least {min_value}",
            details={
                "field": field_name,
                "reason": "below_minimum",
                "value": value,
                "min_value": min_value
            }
        )

    if max_value is not None and value > max_value:
        raise ValidationError(
            f"{field_name} cannot exceed {max_value}",
            details={
                "field": field_name,
                "reason": "above_maximum",
                "value": value,
                "max_value": max_value
            }
        )

    return value
