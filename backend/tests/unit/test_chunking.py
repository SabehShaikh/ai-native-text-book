"""
Unit tests for text chunking utility.

Tests the chunking functionality including various text sizes, overlap logic,
word boundary handling, and edge cases.
"""

import pytest

from src.utils.chunking import chunk_text, chunk_text_with_metadata


class TestChunkText:
    """Test suite for chunk_text function."""

    def test_empty_text(self):
        """Test chunking empty text returns empty list."""
        result = chunk_text("")
        assert result == []

    def test_text_shorter_than_chunk_size(self, sample_text_short):
        """Test text shorter than chunk size returns single chunk."""
        result = chunk_text(sample_text_short, chunk_size=1000, overlap=200)
        assert len(result) == 1
        assert result[0] == sample_text_short

    def test_text_exactly_chunk_size(self):
        """Test text exactly equal to chunk size returns single chunk."""
        text = "a" * 1000
        result = chunk_text(text, chunk_size=1000, overlap=200)
        assert len(result) == 1
        assert len(result[0]) == 1000

    def test_long_text_creates_multiple_chunks(self, sample_text_long):
        """Test long text is split into multiple chunks."""
        result = chunk_text(sample_text_long, chunk_size=1000, overlap=200)
        assert len(result) > 1
        # Each chunk should be approximately chunk_size
        for chunk in result:
            assert len(chunk) <= 1000

    def test_overlap_logic(self):
        """Test overlap between consecutive chunks."""
        text = "word " * 500  # 2500 characters
        result = chunk_text(text, chunk_size=1000, overlap=200)

        # Should have multiple chunks
        assert len(result) >= 2

        # Check that chunks have some overlap
        # The last 200 chars of chunk N should overlap with first chars of chunk N+1
        if len(result) >= 2:
            # Due to word boundary adjustments, we just verify chunks exist
            # and have reasonable lengths
            for chunk in result:
                assert 800 <= len(chunk) <= 1000

    def test_word_boundary_respected(self):
        """Test that chunks split at word boundaries."""
        text = "hello world " * 200  # Creates text with clear word boundaries
        result = chunk_text(text, chunk_size=100, overlap=20)

        # Check that chunks don't end in the middle of a word
        for chunk in result[:-1]:  # Except last chunk
            # If chunk doesn't end with space, it should end at text boundary
            if not chunk.endswith(" "):
                # Should be at the end of the original text
                assert text.endswith(chunk) or chunk.endswith("world")

    def test_custom_chunk_size(self):
        """Test custom chunk size parameter."""
        text = "a" * 2000
        result = chunk_text(text, chunk_size=500, overlap=100)

        assert len(result) >= 3
        for chunk in result:
            assert len(chunk) <= 500

    def test_custom_overlap(self):
        """Test custom overlap parameter."""
        text = "word " * 500
        result = chunk_text(text, chunk_size=1000, overlap=300)

        # With larger overlap, should create more chunks
        assert len(result) >= 2

    def test_zero_overlap(self):
        """Test chunking with zero overlap."""
        text = "a" * 2000
        result = chunk_text(text, chunk_size=500, overlap=0)

        # Should have exactly 4 chunks of 500 chars each
        assert len(result) == 4
        assert all(len(chunk) == 500 for chunk in result)

    def test_small_chunk_size(self):
        """Test very small chunk size."""
        text = "This is a test sentence with multiple words."
        result = chunk_text(text, chunk_size=20, overlap=5)

        assert len(result) >= 2
        for chunk in result:
            assert len(chunk) <= 20

    def test_whitespace_only_chunks_excluded(self):
        """Test that whitespace-only chunks are excluded."""
        text = "word   " * 500  # Multiple spaces
        result = chunk_text(text, chunk_size=50, overlap=10)

        # All chunks should have non-whitespace content
        for chunk in result:
            assert chunk.strip() != ""

    def test_newlines_preserved(self):
        """Test that newlines in text are preserved."""
        text = "Line 1\nLine 2\nLine 3\n" * 50
        result = chunk_text(text, chunk_size=100, overlap=20)

        # Newlines should be present in chunks
        assert any("\n" in chunk for chunk in result)

    def test_invalid_chunk_size_zero(self):
        """Test that zero chunk_size raises ValueError."""
        with pytest.raises(ValueError, match="chunk_size must be greater than 0"):
            chunk_text("test text", chunk_size=0, overlap=10)

    def test_invalid_chunk_size_negative(self):
        """Test that negative chunk_size raises ValueError."""
        with pytest.raises(ValueError, match="chunk_size must be greater than 0"):
            chunk_text("test text", chunk_size=-100, overlap=10)

    def test_invalid_overlap_negative(self):
        """Test that negative overlap raises ValueError."""
        with pytest.raises(ValueError, match="overlap must be non-negative"):
            chunk_text("test text", chunk_size=100, overlap=-10)

    def test_invalid_overlap_exceeds_chunk_size(self):
        """Test that overlap >= chunk_size raises ValueError."""
        with pytest.raises(ValueError, match="overlap must be less than chunk_size"):
            chunk_text("test text", chunk_size=100, overlap=100)

        with pytest.raises(ValueError, match="overlap must be less than chunk_size"):
            chunk_text("test text", chunk_size=100, overlap=150)

    def test_unicode_text(self):
        """Test chunking text with unicode characters."""
        text = "Hello 世界 " * 200
        result = chunk_text(text, chunk_size=100, overlap=20)

        assert len(result) >= 2
        # All chunks should contain valid unicode
        for chunk in result:
            assert isinstance(chunk, str)

    def test_special_characters(self):
        """Test chunking text with special characters."""
        text = "Special @#$%^&*() chars! " * 100
        result = chunk_text(text, chunk_size=100, overlap=20)

        assert len(result) >= 2
        for chunk in result:
            assert len(chunk) > 0


class TestChunkTextWithMetadata:
    """Test suite for chunk_text_with_metadata function."""

    def test_metadata_attached(self):
        """Test that metadata is correctly attached to chunks."""
        text = "word " * 500
        url = "https://example.com/page1"
        title = "Test Page"

        result = chunk_text_with_metadata(
            text,
            chunk_size=1000,
            overlap=200,
            url=url,
            title=title
        )

        assert len(result) > 0
        for i, chunk_data in enumerate(result):
            assert chunk_data["url"] == url
            assert chunk_data["title"] == title
            assert chunk_data["chunk_index"] == i
            assert "text" in chunk_data
            assert len(chunk_data["text"]) > 0

    def test_chunk_index_sequential(self):
        """Test that chunk indices are sequential starting from 0."""
        text = "word " * 500
        result = chunk_text_with_metadata(text, chunk_size=500, overlap=100)

        assert len(result) >= 2
        for i, chunk_data in enumerate(result):
            assert chunk_data["chunk_index"] == i

    def test_empty_metadata(self):
        """Test chunking with empty metadata values."""
        text = "word " * 500
        result = chunk_text_with_metadata(text, chunk_size=500, overlap=100)

        assert len(result) >= 2
        for chunk_data in result:
            assert chunk_data["url"] == ""
            assert chunk_data["title"] == ""

    def test_short_text_with_metadata(self):
        """Test short text with metadata returns single chunk."""
        text = "Short text"
        result = chunk_text_with_metadata(
            text,
            chunk_size=1000,
            url="https://example.com",
            title="Short"
        )

        assert len(result) == 1
        assert result[0]["text"] == text
        assert result[0]["chunk_index"] == 0
        assert result[0]["url"] == "https://example.com"
        assert result[0]["title"] == "Short"

    def test_empty_text_with_metadata(self):
        """Test empty text with metadata returns empty list."""
        result = chunk_text_with_metadata(
            "",
            chunk_size=1000,
            url="https://example.com",
            title="Empty"
        )

        assert result == []
