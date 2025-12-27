"""
Unit tests for validation helper functions.

Tests input validation for user queries and other inputs to ensure
data quality and security.
"""

import pytest

from src.utils.validation import validate_query, validate_url, validate_positive_int
from src.utils.exceptions import ValidationError


class TestValidateQuery:
    """Test cases for query validation."""

    def test_validate_query_valid_case(self):
        """Test validation passes for valid query."""
        query = "What is physical AI?"
        result = validate_query(query)

        assert result == "What is physical AI?"

    def test_validate_query_valid_with_whitespace(self):
        """Test validation trims whitespace from valid query."""
        query = "  What is machine learning?  "
        result = validate_query(query)

        assert result == "What is machine learning?"

    def test_validate_query_valid_at_max_length(self):
        """Test validation passes for query at max length (500 chars)."""
        query = "a" * 500
        result = validate_query(query)

        assert result == query
        assert len(result) == 500

    def test_validate_query_valid_unicode(self):
        """Test validation passes for query with unicode characters."""
        query = "What is AI? 人工知能とは？"
        result = validate_query(query)

        assert result == query

    def test_validate_query_empty_string(self):
        """Test validation fails for empty string."""
        with pytest.raises(ValidationError) as exc_info:
            validate_query("")

        assert "cannot be empty" in str(exc_info.value).lower()
        assert exc_info.value.details["field"] == "query"
        assert exc_info.value.details["reason"] == "empty_string"

    def test_validate_query_whitespace_only(self):
        """Test validation fails for whitespace-only string."""
        with pytest.raises(ValidationError) as exc_info:
            validate_query("   ")

        assert "whitespace" in str(exc_info.value).lower()
        assert exc_info.value.details["field"] == "query"
        assert exc_info.value.details["reason"] == "whitespace_only"

    def test_validate_query_tabs_and_newlines_only(self):
        """Test validation fails for query with only tabs and newlines."""
        with pytest.raises(ValidationError) as exc_info:
            validate_query("\t\n\r")

        assert "whitespace" in str(exc_info.value).lower()
        assert exc_info.value.details["reason"] == "whitespace_only"

    def test_validate_query_too_long(self):
        """Test validation fails for query exceeding max length."""
        query = "a" * 501  # 501 characters

        with pytest.raises(ValidationError) as exc_info:
            validate_query(query)

        assert "exceeds maximum length" in str(exc_info.value)
        assert "500" in str(exc_info.value)
        assert exc_info.value.details["field"] == "query"
        assert exc_info.value.details["reason"] == "too_long"
        assert exc_info.value.details["current_length"] == 501
        assert exc_info.value.details["max_length"] == 500

    def test_validate_query_custom_max_length(self):
        """Test validation with custom max length."""
        query = "a" * 100

        # Should pass with max_length=100
        result = validate_query(query, max_length=100)
        assert result == query

        # Should fail with max_length=50
        with pytest.raises(ValidationError) as exc_info:
            validate_query(query, max_length=50)

        assert "exceeds maximum length" in str(exc_info.value)
        assert "50" in str(exc_info.value)

    def test_validate_query_single_character(self):
        """Test validation passes for single character query."""
        query = "?"
        result = validate_query(query)

        assert result == "?"

    def test_validate_query_with_special_characters(self):
        """Test validation passes for query with special characters."""
        query = "What is AI? #PhysicalAI @robotics 50% accuracy!"
        result = validate_query(query)

        assert result == query

    def test_validate_query_multiline(self):
        """Test validation handles multiline queries correctly."""
        query = "What is AI?\nHow does it work?"
        result = validate_query(query)

        assert result == query

    def test_validate_query_leading_trailing_whitespace(self):
        """Test that leading and trailing whitespace is trimmed."""
        query = "  \t  What is AI?  \n  "
        result = validate_query(query)

        assert result == "What is AI?"


class TestValidateUrl:
    """Test cases for URL validation."""

    def test_validate_url_valid_https(self):
        """Test validation passes for valid HTTPS URL."""
        url = "https://example.com/page"
        result = validate_url(url)

        assert result == url

    def test_validate_url_valid_http(self):
        """Test validation passes for valid HTTP URL."""
        url = "http://example.com/page"
        result = validate_url(url)

        assert result == url

    def test_validate_url_none_not_required(self):
        """Test validation passes for None when not required."""
        result = validate_url(None, required=False)

        assert result is None

    def test_validate_url_none_required(self):
        """Test validation fails for None when required."""
        with pytest.raises(ValidationError) as exc_info:
            validate_url(None, required=True)

        assert "required" in str(exc_info.value).lower()

    def test_validate_url_empty_not_required(self):
        """Test validation returns None for empty string when not required."""
        result = validate_url("", required=False)

        assert result is None

    def test_validate_url_empty_required(self):
        """Test validation fails for empty string when required."""
        with pytest.raises(ValidationError) as exc_info:
            validate_url("", required=True)

        assert "empty" in str(exc_info.value).lower()

    def test_validate_url_invalid_scheme(self):
        """Test validation fails for URL with invalid scheme."""
        with pytest.raises(ValidationError) as exc_info:
            validate_url("ftp://example.com")

        assert "http://" in str(exc_info.value).lower() or "https://" in str(exc_info.value).lower()

    def test_validate_url_no_scheme(self):
        """Test validation fails for URL without scheme."""
        with pytest.raises(ValidationError) as exc_info:
            validate_url("example.com/page")

        assert "http://" in str(exc_info.value).lower() or "https://" in str(exc_info.value).lower()

    def test_validate_url_with_spaces(self):
        """Test validation fails for URL containing spaces."""
        with pytest.raises(ValidationError) as exc_info:
            validate_url("https://example.com/page with spaces")

        assert "space" in str(exc_info.value).lower()

    def test_validate_url_too_short(self):
        """Test validation fails for unrealistically short URL."""
        with pytest.raises(ValidationError) as exc_info:
            validate_url("http://a")

        assert "short" in str(exc_info.value).lower()

    def test_validate_url_with_query_params(self):
        """Test validation passes for URL with query parameters."""
        url = "https://example.com/page?param=value&other=123"
        result = validate_url(url)

        assert result == url

    def test_validate_url_with_fragment(self):
        """Test validation passes for URL with fragment."""
        url = "https://example.com/page#section"
        result = validate_url(url)

        assert result == url


class TestValidatePositiveInt:
    """Test cases for positive integer validation."""

    def test_validate_positive_int_valid(self):
        """Test validation passes for valid positive integer."""
        result = validate_positive_int(5, "count")

        assert result == 5

    def test_validate_positive_int_at_minimum(self):
        """Test validation passes for value at minimum."""
        result = validate_positive_int(1, "count", min_value=1)

        assert result == 1

    def test_validate_positive_int_below_minimum(self):
        """Test validation fails for value below minimum."""
        with pytest.raises(ValidationError) as exc_info:
            validate_positive_int(0, "count", min_value=1)

        assert "at least 1" in str(exc_info.value)
        assert exc_info.value.details["field"] == "count"
        assert exc_info.value.details["reason"] == "below_minimum"

    def test_validate_positive_int_at_maximum(self):
        """Test validation passes for value at maximum."""
        result = validate_positive_int(100, "count", max_value=100)

        assert result == 100

    def test_validate_positive_int_above_maximum(self):
        """Test validation fails for value above maximum."""
        with pytest.raises(ValidationError) as exc_info:
            validate_positive_int(150, "count", max_value=100)

        assert "cannot exceed 100" in str(exc_info.value)
        assert exc_info.value.details["field"] == "count"
        assert exc_info.value.details["reason"] == "above_maximum"

    def test_validate_positive_int_no_maximum(self):
        """Test validation passes with no maximum constraint."""
        result = validate_positive_int(999999, "count", min_value=1, max_value=None)

        assert result == 999999

    def test_validate_positive_int_custom_min_value(self):
        """Test validation with custom minimum value."""
        result = validate_positive_int(10, "score", min_value=10)

        assert result == 10

        with pytest.raises(ValidationError) as exc_info:
            validate_positive_int(9, "score", min_value=10)

        assert "at least 10" in str(exc_info.value)

    def test_validate_positive_int_negative(self):
        """Test validation fails for negative integer."""
        with pytest.raises(ValidationError) as exc_info:
            validate_positive_int(-5, "count")

        assert "at least" in str(exc_info.value)

    def test_validate_positive_int_zero_with_zero_min(self):
        """Test validation passes for zero when min_value is 0."""
        result = validate_positive_int(0, "count", min_value=0)

        assert result == 0
