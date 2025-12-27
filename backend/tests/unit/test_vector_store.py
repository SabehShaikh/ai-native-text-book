"""
Unit tests for VectorStoreService.

Tests vector database operations including upsert, search, health checks,
and Qdrant client mocking.
"""

from unittest.mock import MagicMock, patch
from uuid import uuid4

import pytest
from qdrant_client import models
from qdrant_client.http.exceptions import UnexpectedResponse

from src.services.vector_store import VectorStoreService
from src.utils.exceptions import QdrantError, QdrantTimeout


class TestVectorStoreService:
    """Test suite for VectorStoreService."""

    @pytest.fixture
    def vector_store_service(self, mock_qdrant_client):
        """Create VectorStoreService with mocked client."""
        with patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client):
            service = VectorStoreService()
            return service

    @pytest.mark.asyncio
    async def test_ensure_collection_exists_already_exists(self, vector_store_service):
        """Test ensure_collection_exists when collection already exists."""
        # Mock collection already exists
        mock_collection = MagicMock()
        mock_collection.name = vector_store_service.collection_name
        vector_store_service.client.get_collections.return_value.collections = [mock_collection]

        # Should not create new collection
        await vector_store_service.ensure_collection_exists()

        # Verify create_collection was not called
        vector_store_service.client.create_collection.assert_not_called()

    @pytest.mark.asyncio
    async def test_ensure_collection_exists_creates_new(self, vector_store_service):
        """Test ensure_collection_exists creates new collection."""
        # Mock collection does not exist
        vector_store_service.client.get_collections.return_value.collections = []

        await vector_store_service.ensure_collection_exists()

        # Verify create_collection was called with correct parameters
        vector_store_service.client.create_collection.assert_called_once()
        call_args = vector_store_service.client.create_collection.call_args

        assert call_args.kwargs['collection_name'] == vector_store_service.collection_name
        assert call_args.kwargs['vectors_config'].size == 1024
        assert call_args.kwargs['vectors_config'].distance == models.Distance.COSINE

    @pytest.mark.asyncio
    async def test_ensure_collection_exists_api_error(self, vector_store_service):
        """Test ensure_collection_exists handles API errors."""
        # Mock API error
        vector_store_service.client.get_collections.side_effect = UnexpectedResponse(
            status_code=500,
            reason_phrase="Internal Server Error"
        )

        with pytest.raises(QdrantError, match="Collection creation failed"):
            await vector_store_service.ensure_collection_exists()

    @pytest.mark.asyncio
    async def test_upsert_chunks_success(self, vector_store_service, sample_text_chunks):
        """Test successful chunk upsert."""
        result = await vector_store_service.upsert_chunks(sample_text_chunks)

        assert result == len(sample_text_chunks)
        vector_store_service.client.upsert.assert_called_once()

        # Verify upsert was called with correct parameters
        call_args = vector_store_service.client.upsert.call_args
        assert call_args.kwargs['collection_name'] == vector_store_service.collection_name
        assert len(call_args.kwargs['points']) == len(sample_text_chunks)
        assert call_args.kwargs['wait'] is True

    @pytest.mark.asyncio
    async def test_upsert_chunks_empty_list(self, vector_store_service):
        """Test upsert with empty chunk list."""
        result = await vector_store_service.upsert_chunks([])

        assert result == 0
        vector_store_service.client.upsert.assert_not_called()

    @pytest.mark.asyncio
    async def test_upsert_chunks_verifies_payload(self, vector_store_service, sample_text_chunks):
        """Test that upsert includes correct payload structure."""
        await vector_store_service.upsert_chunks(sample_text_chunks)

        call_args = vector_store_service.client.upsert.call_args
        points = call_args.kwargs['points']

        for i, point in enumerate(points):
            assert point.id == str(sample_text_chunks[i].id)
            assert point.vector == sample_text_chunks[i].vector
            assert point.payload['text'] == sample_text_chunks[i].text
            assert point.payload['url'] == sample_text_chunks[i].url
            assert point.payload['title'] == sample_text_chunks[i].title
            assert point.payload['chunk_index'] == sample_text_chunks[i].chunk_index

    @pytest.mark.asyncio
    async def test_upsert_chunks_api_error(self, vector_store_service, sample_text_chunks):
        """Test upsert handles API errors."""
        vector_store_service.client.upsert.side_effect = UnexpectedResponse(
            status_code=500,
            reason_phrase="Internal Server Error"
        )

        with pytest.raises(QdrantError, match="Upsert failed"):
            await vector_store_service.upsert_chunks(sample_text_chunks)

    @pytest.mark.asyncio
    async def test_upsert_chunks_generic_error(self, vector_store_service, sample_text_chunks):
        """Test upsert handles generic errors."""
        vector_store_service.client.upsert.side_effect = Exception("Connection error")

        with pytest.raises(QdrantError, match="Upsert failed"):
            await vector_store_service.upsert_chunks(sample_text_chunks)

    @pytest.mark.asyncio
    async def test_search_success(self, vector_store_service):
        """Test successful vector search."""
        query_vector = [0.5] * 1024
        top_k = 5

        results = await vector_store_service.search(query_vector, top_k)

        assert len(results) <= top_k
        assert all('id' in r for r in results)
        assert all('score' in r for r in results)
        assert all('text' in r for r in results)
        assert all('url' in r for r in results)
        assert all('title' in r for r in results)
        assert all('chunk_index' in r for r in results)

        vector_store_service.client.search.assert_called_once()

    @pytest.mark.asyncio
    async def test_search_custom_top_k(self, vector_store_service):
        """Test search with custom top_k parameter."""
        query_vector = [0.5] * 1024
        top_k = 10

        results = await vector_store_service.search(query_vector, top_k)

        call_args = vector_store_service.client.search.call_args
        assert call_args.kwargs['limit'] == top_k

    @pytest.mark.asyncio
    async def test_search_invalid_vector_dimension(self, vector_store_service):
        """Test search with invalid vector dimension."""
        query_vector = [0.5] * 512  # Wrong dimension

        with pytest.raises(QdrantError, match="Invalid query vector dimension"):
            await vector_store_service.search(query_vector, 5)

    @pytest.mark.asyncio
    async def test_search_timeout(self, vector_store_service):
        """Test search timeout handling."""
        query_vector = [0.5] * 1024

        vector_store_service.client.search.side_effect = UnexpectedResponse(
            status_code=504,
            reason_phrase="Gateway Timeout"
        )

        with pytest.raises(QdrantTimeout, match="Search operation timed out"):
            await vector_store_service.search(query_vector, 5)

    @pytest.mark.asyncio
    async def test_search_api_error(self, vector_store_service):
        """Test search handles API errors."""
        query_vector = [0.5] * 1024

        vector_store_service.client.search.side_effect = UnexpectedResponse(
            status_code=500,
            reason_phrase="Internal Server Error"
        )

        with pytest.raises(QdrantError, match="Search failed"):
            await vector_store_service.search(query_vector, 5)

    @pytest.mark.asyncio
    async def test_search_generic_error(self, vector_store_service):
        """Test search handles generic errors."""
        query_vector = [0.5] * 1024

        vector_store_service.client.search.side_effect = Exception("Connection error")

        with pytest.raises(QdrantError, match="Search failed"):
            await vector_store_service.search(query_vector, 5)

    @pytest.mark.asyncio
    async def test_search_returns_sorted_by_score(self, vector_store_service):
        """Test that search results are sorted by score."""
        query_vector = [0.5] * 1024

        results = await vector_store_service.search(query_vector, 5)

        # Results should be sorted by score (descending)
        if len(results) > 1:
            scores = [r['score'] for r in results]
            assert scores == sorted(scores, reverse=True)

    @pytest.mark.asyncio
    async def test_get_collection_count_success(self, vector_store_service):
        """Test getting collection count."""
        count = await vector_store_service.get_collection_count()

        assert isinstance(count, int)
        assert count >= 0
        vector_store_service.client.get_collection.assert_called_once_with(
            collection_name=vector_store_service.collection_name
        )

    @pytest.mark.asyncio
    async def test_get_collection_count_empty_collection(self, vector_store_service):
        """Test getting count for empty collection."""
        vector_store_service.client.get_collection.return_value.points_count = 0

        count = await vector_store_service.get_collection_count()

        assert count == 0

    @pytest.mark.asyncio
    async def test_get_collection_count_none_value(self, vector_store_service):
        """Test getting count when points_count is None."""
        vector_store_service.client.get_collection.return_value.points_count = None

        count = await vector_store_service.get_collection_count()

        assert count == 0

    @pytest.mark.asyncio
    async def test_get_collection_count_api_error(self, vector_store_service):
        """Test get_collection_count handles API errors."""
        vector_store_service.client.get_collection.side_effect = UnexpectedResponse(
            status_code=500,
            reason_phrase="Internal Server Error"
        )

        with pytest.raises(QdrantError, match="Failed to get collection count"):
            await vector_store_service.get_collection_count()

    @pytest.mark.asyncio
    async def test_get_collection_count_generic_error(self, vector_store_service):
        """Test get_collection_count handles generic errors."""
        vector_store_service.client.get_collection.side_effect = Exception("Connection error")

        with pytest.raises(QdrantError, match="Failed to get collection count"):
            await vector_store_service.get_collection_count()

    @pytest.mark.asyncio
    async def test_health_check_healthy(self, vector_store_service):
        """Test health check when Qdrant is healthy."""
        is_healthy = await vector_store_service.health_check()

        assert is_healthy is True
        vector_store_service.client.get_collection.assert_called_once()

    @pytest.mark.asyncio
    async def test_health_check_unhealthy(self, vector_store_service):
        """Test health check when Qdrant is unhealthy."""
        vector_store_service.client.get_collection.side_effect = Exception("Connection failed")

        is_healthy = await vector_store_service.health_check()

        assert is_healthy is False

    @pytest.mark.asyncio
    async def test_health_check_api_error(self, vector_store_service):
        """Test health check with API error."""
        vector_store_service.client.get_collection.side_effect = UnexpectedResponse(
            status_code=503,
            reason_phrase="Service Unavailable"
        )

        is_healthy = await vector_store_service.health_check()

        assert is_healthy is False

    @pytest.mark.asyncio
    async def test_health_check_timeout(self, vector_store_service):
        """Test health check with timeout."""
        vector_store_service.client.get_collection.side_effect = TimeoutError("Request timeout")

        is_healthy = await vector_store_service.health_check()

        assert is_healthy is False

    def test_initialization_success(self, mock_qdrant_client):
        """Test successful VectorStoreService initialization."""
        with patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client):
            service = VectorStoreService()

            assert service.client is not None
            assert service.collection_name is not None

    def test_initialization_failure(self):
        """Test VectorStoreService initialization failure."""
        with patch('src.services.vector_store.QdrantClient', side_effect=Exception("Connection failed")):
            with pytest.raises(QdrantError, match="Qdrant initialization failed"):
                VectorStoreService()

    @pytest.mark.asyncio
    async def test_search_empty_results(self, vector_store_service):
        """Test search with no matching results."""
        query_vector = [0.5] * 1024

        # Mock empty search results
        vector_store_service.client.search.return_value = []

        results = await vector_store_service.search(query_vector, 5)

        assert results == []

    @pytest.mark.asyncio
    async def test_upsert_chunks_single_chunk(self, vector_store_service, sample_text_chunks):
        """Test upserting single chunk."""
        single_chunk = [sample_text_chunks[0]]

        result = await vector_store_service.upsert_chunks(single_chunk)

        assert result == 1
        vector_store_service.client.upsert.assert_called_once()

    @pytest.mark.asyncio
    async def test_search_with_payload_verification(self, vector_store_service):
        """Test that search includes payload in results."""
        query_vector = [0.5] * 1024

        results = await vector_store_service.search(query_vector, 3)

        # Verify payload fields are present
        for result in results:
            assert 'text' in result
            assert 'url' in result
            assert 'title' in result
            assert 'chunk_index' in result
            assert isinstance(result['text'], str)
            assert isinstance(result['url'], str)
            assert isinstance(result['title'], str)
            assert isinstance(result['chunk_index'], int)
