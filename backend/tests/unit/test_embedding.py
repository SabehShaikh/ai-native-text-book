"""
Unit tests for EmbeddingService.

Tests embedding generation including single text, batch processing,
Cohere API mocking, and retry logic on rate limits.
"""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
import cohere

from src.services.embedding import EmbeddingService
from src.utils.exceptions import CohereAPIError


class TestEmbeddingService:
    """Test suite for EmbeddingService."""

    @pytest.fixture
    def embedding_service(self, mock_cohere_client):
        """Create EmbeddingService with mocked client."""
        service = EmbeddingService()
        service.client = mock_cohere_client
        return service

    @pytest.mark.asyncio
    async def test_embed_text_success(self, embedding_service):
        """Test successful single text embedding."""
        text = "This is a test sentence for embedding."

        result = await embedding_service.embed_text(text)

        assert isinstance(result, list)
        assert len(result) == 1024  # Cohere embed-english-v3.0 dimension
        assert all(isinstance(x, float) for x in result)

    @pytest.mark.asyncio
    async def test_embed_text_empty_string(self, embedding_service):
        """Test embedding empty string."""
        result = await embedding_service.embed_text("")

        assert isinstance(result, list)
        assert len(result) == 1024

    @pytest.mark.asyncio
    async def test_embed_text_long_text(self, embedding_service):
        """Test embedding long text."""
        text = "Long text. " * 1000  # Very long text

        result = await embedding_service.embed_text(text)

        assert isinstance(result, list)
        assert len(result) == 1024

    @pytest.mark.asyncio
    async def test_embed_text_special_characters(self, embedding_service):
        """Test embedding text with special characters."""
        text = "Special chars: @#$%^&*() 世界 emoji 😀"

        result = await embedding_service.embed_text(text)

        assert isinstance(result, list)
        assert len(result) == 1024

    @pytest.mark.asyncio
    async def test_embed_text_with_rate_limit_retry(self):
        """Test retry logic when Cohere API rate limit is hit."""
        service = EmbeddingService()

        # Mock client that fails once then succeeds
        mock_client = AsyncMock()
        call_count = 0

        async def mock_embed(*args, **kwargs):
            nonlocal call_count
            call_count += 1

            if call_count == 1:
                raise cohere.errors.TooManyRequestsError("Rate limit exceeded")
            else:
                response = MagicMock()
                response.embeddings = [[0.1] * 1024]
                return response

        mock_client.embed = mock_embed
        service.client = mock_client

        # Should succeed after retry
        result = await service.embed_text("test text")

        assert len(result) == 1024
        assert call_count == 2  # Failed once, succeeded on retry

    @pytest.mark.asyncio
    async def test_embed_text_rate_limit_max_retries_exceeded(self):
        """Test that max retries are respected for rate limits."""
        service = EmbeddingService()

        # Mock client that always fails
        mock_client = AsyncMock()

        async def mock_embed_always_fails(*args, **kwargs):
            raise cohere.errors.TooManyRequestsError("Rate limit exceeded")

        mock_client.embed = mock_embed_always_fails
        service.client = mock_client

        # Should raise CohereAPIError after max retries
        with pytest.raises(CohereAPIError, match="failed after 3 attempts"):
            await service.embed_text("test text")

    @pytest.mark.asyncio
    async def test_embed_text_api_error(self):
        """Test handling of Cohere API errors."""
        service = EmbeddingService()

        # Mock client that raises generic error
        mock_client = AsyncMock()

        async def mock_embed_with_error(*args, **kwargs):
            raise Exception("API connection error")

        mock_client.embed = mock_embed_with_error
        service.client = mock_client

        # Should raise CohereAPIError
        with pytest.raises(CohereAPIError, match="Embedding generation failed"):
            await service.embed_text("test text")

    @pytest.mark.asyncio
    async def test_embed_text_invalid_dimension(self):
        """Test error handling for invalid embedding dimension."""
        service = EmbeddingService()

        # Mock client returning wrong dimension
        mock_client = AsyncMock()

        async def mock_embed_wrong_dimension(*args, **kwargs):
            response = MagicMock()
            response.embeddings = [[0.1] * 512]  # Wrong dimension
            return response

        mock_client.embed = mock_embed_wrong_dimension
        service.client = mock_client

        # Should raise CohereAPIError
        with pytest.raises(CohereAPIError, match="Invalid embedding dimension"):
            await service.embed_text("test text")

    @pytest.mark.asyncio
    async def test_embed_batch_success(self, embedding_service):
        """Test successful batch embedding."""
        texts = [
            "First sentence for embedding.",
            "Second sentence for embedding.",
            "Third sentence for embedding."
        ]

        result = await embedding_service.embed_batch(texts)

        assert isinstance(result, list)
        assert len(result) == 3
        assert all(len(embedding) == 1024 for embedding in result)

    @pytest.mark.asyncio
    async def test_embed_batch_empty_list(self, embedding_service):
        """Test batch embedding with empty list."""
        result = await embedding_service.embed_batch([])

        assert result == []

    @pytest.mark.asyncio
    async def test_embed_batch_single_text(self, embedding_service):
        """Test batch embedding with single text."""
        texts = ["Single sentence."]

        result = await embedding_service.embed_batch(texts)

        assert len(result) == 1
        assert len(result[0]) == 1024

    @pytest.mark.asyncio
    async def test_embed_batch_large_batch(self, embedding_service):
        """Test batch embedding with large number of texts."""
        texts = [f"Sentence number {i}" for i in range(50)]

        result = await embedding_service.embed_batch(texts)

        assert len(result) == 50
        assert all(len(embedding) == 1024 for embedding in result)

    @pytest.mark.asyncio
    async def test_embed_batch_exceeds_batch_size(self):
        """Test batch embedding that exceeds batch size limit."""
        service = EmbeddingService()

        # Create texts exceeding batch size (96)
        texts = [f"Sentence {i}" for i in range(150)]

        # Mock client
        mock_client = AsyncMock()
        batch_calls = []

        async def mock_embed(*args, **kwargs):
            batch = kwargs.get('texts', [])
            batch_calls.append(len(batch))
            response = MagicMock()
            response.embeddings = [[0.1 + i * 0.01] * 1024 for i in range(len(batch))]
            return response

        mock_client.embed = mock_embed
        service.client = mock_client

        result = await service.embed_batch(texts)

        # Should split into multiple batches
        assert len(result) == 150
        assert len(batch_calls) >= 2  # At least 2 batches
        assert all(batch_size <= 96 for batch_size in batch_calls)

    @pytest.mark.asyncio
    async def test_embed_batch_with_rate_limit_retry(self):
        """Test batch embedding with rate limit retry."""
        service = EmbeddingService()

        texts = ["Text 1", "Text 2", "Text 3"]

        # Mock client that fails once then succeeds
        mock_client = AsyncMock()
        call_count = 0

        async def mock_embed(*args, **kwargs):
            nonlocal call_count
            call_count += 1

            if call_count == 1:
                raise cohere.errors.TooManyRequestsError("Rate limit exceeded")
            else:
                batch = kwargs.get('texts', [])
                response = MagicMock()
                response.embeddings = [[0.1] * 1024 for _ in batch]
                return response

        mock_client.embed = mock_embed
        service.client = mock_client

        # Should succeed after retry
        result = await service.embed_batch(texts)

        assert len(result) == 3
        assert call_count == 2

    @pytest.mark.asyncio
    async def test_embed_batch_rate_limit_exhausted(self):
        """Test batch embedding when rate limit retries are exhausted."""
        service = EmbeddingService()

        texts = ["Text 1", "Text 2"]

        # Mock client that always fails
        mock_client = AsyncMock()

        async def mock_embed_always_fails(*args, **kwargs):
            raise cohere.errors.TooManyRequestsError("Rate limit exceeded")

        mock_client.embed = mock_embed_always_fails
        service.client = mock_client

        # Should raise CohereAPIError
        with pytest.raises(CohereAPIError, match="failed after 3 attempts"):
            await service.embed_batch(texts)

    @pytest.mark.asyncio
    async def test_embed_batch_api_error(self):
        """Test batch embedding with API error."""
        service = EmbeddingService()

        texts = ["Text 1", "Text 2"]

        # Mock client that raises error
        mock_client = AsyncMock()

        async def mock_embed_with_error(*args, **kwargs):
            raise Exception("API error")

        mock_client.embed = mock_embed_with_error
        service.client = mock_client

        # Should raise CohereAPIError
        with pytest.raises(CohereAPIError, match="Batch embedding generation failed"):
            await service.embed_batch(texts)

    @pytest.mark.asyncio
    async def test_embed_batch_invalid_dimension(self):
        """Test batch embedding with invalid dimension in response."""
        service = EmbeddingService()

        texts = ["Text 1", "Text 2"]

        # Mock client returning wrong dimension
        mock_client = AsyncMock()

        async def mock_embed_wrong_dimension(*args, **kwargs):
            batch = kwargs.get('texts', [])
            response = MagicMock()
            # First embedding is correct, second is wrong
            response.embeddings = [
                [0.1] * 1024,
                [0.2] * 512  # Wrong dimension
            ]
            return response

        mock_client.embed = mock_embed_wrong_dimension
        service.client = mock_client

        # Should raise CohereAPIError
        with pytest.raises(CohereAPIError, match="Invalid embedding dimension at index 1"):
            await service.embed_batch(texts)

    @pytest.mark.asyncio
    async def test_embed_batch_mixed_text_lengths(self, embedding_service):
        """Test batch embedding with varied text lengths."""
        texts = [
            "Short",
            "Medium length text for testing.",
            "Very long text. " * 100
        ]

        result = await embedding_service.embed_batch(texts)

        assert len(result) == 3
        assert all(len(embedding) == 1024 for embedding in result)

    @pytest.mark.asyncio
    async def test_model_parameter(self, embedding_service):
        """Test that correct model is used."""
        assert embedding_service.model == "embed-english-v3.0"

    @pytest.mark.asyncio
    async def test_max_retries_parameter(self, embedding_service):
        """Test that max_retries is configured correctly."""
        assert embedding_service.max_retries == 3
