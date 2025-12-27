"""
Unit tests for RetrievalService.

Tests the semantic search retrieval pipeline including embedding generation
and vector search integration.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

from src.models.chat import RetrievalResult
from src.models.ingest import TextChunk
from src.services.retrieval import RetrievalService
from src.utils.exceptions import CohereAPIError, QdrantError, QdrantTimeout


@pytest.mark.asyncio
async def test_retrieve_success(mock_cohere_client, mock_qdrant_client):
    """Test successful retrieval with top 5 results."""
    # Setup
    with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client):
        from src.services.embedding import EmbeddingService
        from src.services.vector_store import VectorStoreService

        embedding_service = EmbeddingService()

        # Mock vector store service with proper search results
        vector_store_service = MagicMock(spec=VectorStoreService)
        search_results = [
            {
                "id": str(uuid4()),
                "text": f"Sample chunk {i}",
                "url": f"https://example.com/page{i % 3}",  # Duplicate URLs to test deduplication
                "title": f"Page {i % 3}",
                "chunk_index": i,
                "score": 0.9 - (i * 0.1)
            }
            for i in range(5)
        ]
        vector_store_service.search = AsyncMock(return_value=search_results)

        retrieval_service = RetrievalService(
            embedding_service=embedding_service,
            vector_store_service=vector_store_service
        )

        # Execute
        result = await retrieval_service.retrieve(
            query="What is physical AI?",
            top_k=5
        )

        # Verify
        assert isinstance(result, RetrievalResult)
        assert len(result.chunks) == 5
        assert len(result.scores) == 5

        # Verify top 5 results returned
        for i, chunk in enumerate(result.chunks):
            assert isinstance(chunk, TextChunk)
            assert chunk.text == f"Sample chunk {i}"
            assert chunk.url == f"https://example.com/page{i % 3}"
            assert result.scores[i] == 0.9 - (i * 0.1)

        # Verify unique source extraction (should have 3 unique URLs from 5 chunks)
        assert len(result.sources) == 3
        assert result.sources == [
            "https://example.com/page0",
            "https://example.com/page1",
            "https://example.com/page2"
        ]

        # Verify services called correctly
        vector_store_service.search.assert_called_once()
        call_args = vector_store_service.search.call_args
        assert call_args[1]["top_k"] == 5


@pytest.mark.asyncio
async def test_retrieve_unique_sources_preserved_order(mock_cohere_client, mock_qdrant_client):
    """Test that unique source URLs are extracted in order of appearance."""
    with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client):
        from src.services.embedding import EmbeddingService
        from src.services.vector_store import VectorStoreService

        embedding_service = EmbeddingService()
        vector_store_service = MagicMock(spec=VectorStoreService)

        # Create search results with repeated URLs in specific order
        search_results = [
            {
                "id": str(uuid4()),
                "text": "Chunk from page A",
                "url": "https://example.com/pageA",
                "title": "Page A",
                "chunk_index": 0,
                "score": 0.95
            },
            {
                "id": str(uuid4()),
                "text": "Another chunk from page A",
                "url": "https://example.com/pageA",  # Duplicate
                "title": "Page A",
                "chunk_index": 1,
                "score": 0.90
            },
            {
                "id": str(uuid4()),
                "text": "Chunk from page B",
                "url": "https://example.com/pageB",
                "title": "Page B",
                "chunk_index": 0,
                "score": 0.85
            },
            {
                "id": str(uuid4()),
                "text": "Chunk from page C",
                "url": "https://example.com/pageC",
                "title": "Page C",
                "chunk_index": 0,
                "score": 0.80
            },
            {
                "id": str(uuid4()),
                "text": "Another chunk from page B",
                "url": "https://example.com/pageB",  # Duplicate
                "title": "Page B",
                "chunk_index": 1,
                "score": 0.75
            }
        ]
        vector_store_service.search = AsyncMock(return_value=search_results)

        retrieval_service = RetrievalService(
            embedding_service=embedding_service,
            vector_store_service=vector_store_service
        )

        # Execute
        result = await retrieval_service.retrieve("test query", top_k=5)

        # Verify unique sources in order of first appearance
        assert result.sources == [
            "https://example.com/pageA",
            "https://example.com/pageB",
            "https://example.com/pageC"
        ]


@pytest.mark.asyncio
async def test_retrieve_default_top_k(mock_cohere_client, mock_qdrant_client):
    """Test that default top_k from config is used when not specified."""
    with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client):
        with patch('src.config.settings') as mock_settings:
            mock_settings.TOP_K_RESULTS = 5

            from src.services.embedding import EmbeddingService
            from src.services.vector_store import VectorStoreService

            embedding_service = EmbeddingService()
            vector_store_service = MagicMock(spec=VectorStoreService)

            search_results = [
                {
                    "id": str(uuid4()),
                    "text": f"Chunk {i}",
                    "url": f"https://example.com/page{i}",
                    "title": f"Page {i}",
                    "chunk_index": i,
                    "score": 0.9 - (i * 0.1)
                }
                for i in range(5)
            ]
            vector_store_service.search = AsyncMock(return_value=search_results)

            retrieval_service = RetrievalService(
                embedding_service=embedding_service,
                vector_store_service=vector_store_service
            )

            # Execute without specifying top_k
            result = await retrieval_service.retrieve("test query")

            # Verify default top_k was used
            assert len(result.chunks) == 5
            vector_store_service.search.assert_called_once()
            call_args = vector_store_service.search.call_args
            assert call_args[1]["top_k"] == 5


@pytest.mark.asyncio
async def test_retrieve_embedding_failure(mock_cohere_client, mock_qdrant_client):
    """Test that CohereAPIError is propagated when embedding fails."""
    with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client):
        from src.services.embedding import EmbeddingService
        from src.services.vector_store import VectorStoreService

        # Mock embedding service to raise error
        embedding_service = MagicMock(spec=EmbeddingService)
        embedding_service.embed_text = AsyncMock(
            side_effect=CohereAPIError("Rate limit exceeded")
        )

        vector_store_service = MagicMock(spec=VectorStoreService)

        retrieval_service = RetrievalService(
            embedding_service=embedding_service,
            vector_store_service=vector_store_service
        )

        # Execute and verify exception
        with pytest.raises(CohereAPIError) as exc_info:
            await retrieval_service.retrieve("test query", top_k=5)

        assert "Rate limit exceeded" in str(exc_info.value)


@pytest.mark.asyncio
async def test_retrieve_qdrant_search_failure(mock_cohere_client):
    """Test that QdrantError is propagated when vector search fails."""
    with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client):
        from src.services.embedding import EmbeddingService
        from src.services.vector_store import VectorStoreService

        embedding_service = EmbeddingService()

        # Mock vector store service to raise error
        vector_store_service = MagicMock(spec=VectorStoreService)
        vector_store_service.search = AsyncMock(
            side_effect=QdrantError("Connection failed")
        )

        retrieval_service = RetrievalService(
            embedding_service=embedding_service,
            vector_store_service=vector_store_service
        )

        # Execute and verify exception
        with pytest.raises(QdrantError) as exc_info:
            await retrieval_service.retrieve("test query", top_k=5)

        assert "Connection failed" in str(exc_info.value)


@pytest.mark.asyncio
async def test_retrieve_qdrant_timeout(mock_cohere_client):
    """Test that QdrantTimeout is propagated when search times out."""
    with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client):
        from src.services.embedding import EmbeddingService
        from src.services.vector_store import VectorStoreService

        embedding_service = EmbeddingService()

        # Mock vector store service to raise timeout error
        vector_store_service = MagicMock(spec=VectorStoreService)
        vector_store_service.search = AsyncMock(
            side_effect=QdrantTimeout("Search exceeded 5 second timeout")
        )

        retrieval_service = RetrievalService(
            embedding_service=embedding_service,
            vector_store_service=vector_store_service
        )

        # Execute and verify exception
        with pytest.raises(QdrantTimeout) as exc_info:
            await retrieval_service.retrieve("test query", top_k=5)

        assert "timeout" in str(exc_info.value).lower()


@pytest.mark.asyncio
async def test_retrieve_empty_results(mock_cohere_client):
    """Test retrieval when no results are found."""
    with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client):
        from src.services.embedding import EmbeddingService
        from src.services.vector_store import VectorStoreService

        embedding_service = EmbeddingService()
        vector_store_service = MagicMock(spec=VectorStoreService)

        # Return empty search results
        vector_store_service.search = AsyncMock(return_value=[])

        retrieval_service = RetrievalService(
            embedding_service=embedding_service,
            vector_store_service=vector_store_service
        )

        # Execute
        result = await retrieval_service.retrieve("test query", top_k=5)

        # Verify empty result
        assert isinstance(result, RetrievalResult)
        assert len(result.chunks) == 0
        assert len(result.sources) == 0
        assert len(result.scores) == 0


@pytest.mark.asyncio
async def test_retrieve_single_source_multiple_chunks(mock_cohere_client):
    """Test retrieval when all chunks come from the same source."""
    with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client):
        from src.services.embedding import EmbeddingService
        from src.services.vector_store import VectorStoreService

        embedding_service = EmbeddingService()
        vector_store_service = MagicMock(spec=VectorStoreService)

        # All chunks from same URL
        search_results = [
            {
                "id": str(uuid4()),
                "text": f"Chunk {i}",
                "url": "https://example.com/single-page",
                "title": "Single Page",
                "chunk_index": i,
                "score": 0.9 - (i * 0.05)
            }
            for i in range(5)
        ]
        vector_store_service.search = AsyncMock(return_value=search_results)

        retrieval_service = RetrievalService(
            embedding_service=embedding_service,
            vector_store_service=vector_store_service
        )

        # Execute
        result = await retrieval_service.retrieve("test query", top_k=5)

        # Verify single unique source
        assert len(result.chunks) == 5
        assert len(result.sources) == 1
        assert result.sources[0] == "https://example.com/single-page"
