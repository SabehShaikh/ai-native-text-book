"""
Test fixtures for the RAG Chatbot API test suite.

This module provides reusable pytest fixtures for mocking external services
including Cohere API, Qdrant vector database, and HTTP clients for sitemap/page fetching.
"""

from typing import List
from unittest.mock import AsyncMock, MagicMock
from uuid import uuid4

import pytest
from qdrant_client import models


@pytest.fixture
def mock_cohere_client():
    """
    Mock Cohere client for embedding tests.

    Returns:
        AsyncMock configured to simulate Cohere API responses
    """
    client = AsyncMock()

    # Mock single embedding response
    mock_single_response = MagicMock()
    mock_single_response.embeddings = [[0.1] * 1024]  # 1024-dimensional vector

    # Mock batch embedding response
    def create_batch_response(texts: List[str]):
        response = MagicMock()
        response.embeddings = [[0.1 + i * 0.01] * 1024 for i in range(len(texts))]
        return response

    async def mock_embed(*args, **kwargs):
        texts = kwargs.get('texts', [])
        if len(texts) == 1:
            return mock_single_response
        else:
            return create_batch_response(texts)

    client.embed = mock_embed

    return client


@pytest.fixture
def mock_cohere_client_with_rate_limit():
    """
    Mock Cohere client that simulates rate limiting.

    Returns:
        AsyncMock that raises TooManyRequestsError on first call
    """
    import cohere

    client = AsyncMock()
    call_count = 0

    async def mock_embed_with_retry(*args, **kwargs):
        nonlocal call_count
        call_count += 1

        if call_count == 1:
            # First call fails with rate limit
            raise cohere.errors.TooManyRequestsError("Rate limit exceeded")
        else:
            # Subsequent calls succeed
            texts = kwargs.get('texts', [])
            response = MagicMock()
            response.embeddings = [[0.1] * 1024 for _ in texts]
            return response

    client.embed = mock_embed_with_retry

    return client


@pytest.fixture
def mock_qdrant_client():
    """
    Mock Qdrant client for vector store tests.

    Returns:
        MagicMock configured to simulate Qdrant API responses
    """
    client = MagicMock()

    # Mock get_collections response
    mock_collections = MagicMock()
    mock_collections.collections = []
    client.get_collections.return_value = mock_collections

    # Mock create_collection response
    client.create_collection.return_value = True

    # Mock upsert response
    mock_operation_info = MagicMock()
    mock_operation_info.status = "completed"
    client.upsert.return_value = mock_operation_info

    # Mock search response
    def create_search_results(limit: int):
        results = []
        for i in range(limit):
            result = MagicMock()
            result.id = str(uuid4())
            result.score = 0.9 - (i * 0.1)
            result.payload = {
                "text": f"Sample chunk {i}",
                "url": f"https://example.com/page{i}",
                "title": f"Page {i}",
                "chunk_index": i
            }
            results.append(result)
        return results

    client.search.side_effect = lambda collection_name, query_vector, limit, **kwargs: create_search_results(limit)

    # Mock get_collection response
    mock_collection_info = MagicMock()
    mock_collection_info.points_count = 100
    client.get_collection.return_value = mock_collection_info

    return client


@pytest.fixture
def mock_qdrant_client_unhealthy():
    """
    Mock Qdrant client that simulates connection failures.

    Returns:
        MagicMock that raises exceptions on operations
    """
    from qdrant_client.http.exceptions import UnexpectedResponse

    client = MagicMock()
    client.get_collection.side_effect = UnexpectedResponse(
        status_code=503,
        reason_phrase="Service Unavailable"
    )
    client.get_collections.side_effect = UnexpectedResponse(
        status_code=503,
        reason_phrase="Service Unavailable"
    )

    return client


@pytest.fixture
def mock_httpx_client():
    """
    Mock httpx.AsyncClient for sitemap and page fetching tests.

    Returns:
        AsyncMock configured to simulate HTTP responses
    """
    client = AsyncMock()

    # Mock sitemap.xml response
    sitemap_xml = """<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
    <url>
        <loc>https://example.com/page1</loc>
    </url>
    <url>
        <loc>https://example.com/page2</loc>
    </url>
    <url>
        <loc>https://example.com/page3</loc>
    </url>
</urlset>"""

    # Mock HTML page response
    page_html = """<!DOCTYPE html>
<html>
<head>
    <title>Test Page</title>
</head>
<body>
    <h1>Test Page Title</h1>
    <p>This is a test page with enough content to be chunked. """ + ("Lorem ipsum dolor sit amet. " * 50) + """</p>
</body>
</html>"""

    async def mock_get(url, *args, **kwargs):
        response = MagicMock()
        response.status_code = 200

        if "sitemap.xml" in url:
            response.content = sitemap_xml.encode('utf-8')
            response.text = sitemap_xml
        else:
            response.content = page_html.encode('utf-8')
            response.text = page_html

        response.raise_for_status = MagicMock()
        return response

    client.get = mock_get
    client.__aenter__ = AsyncMock(return_value=client)
    client.__aexit__ = AsyncMock(return_value=None)

    return client


@pytest.fixture
def mock_httpx_client_with_errors():
    """
    Mock httpx.AsyncClient that simulates HTTP errors.

    Returns:
        AsyncMock that raises HTTPStatusError
    """
    import httpx

    client = AsyncMock()

    async def mock_get_with_error(url, *args, **kwargs):
        request = httpx.Request("GET", url)
        response = httpx.Response(404, request=request)
        raise httpx.HTTPStatusError(
            "Not Found",
            request=request,
            response=response
        )

    client.get = mock_get_with_error
    client.__aenter__ = AsyncMock(return_value=client)
    client.__aexit__ = AsyncMock(return_value=None)

    return client


@pytest.fixture
def mock_httpx_client_malformed_sitemap():
    """
    Mock httpx.AsyncClient that returns malformed sitemap.

    Returns:
        AsyncMock that returns invalid XML
    """
    client = AsyncMock()

    async def mock_get(url, *args, **kwargs):
        response = MagicMock()
        response.status_code = 200

        # Malformed XML
        malformed_xml = """<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
    <url>
        <loc>https://example.com/page1</loc>
    </url>
    <url>
        <loc>https://example.com/page2</loc>
    <!-- Missing closing tags -->"""

        response.content = malformed_xml.encode('utf-8')
        response.text = malformed_xml
        response.raise_for_status = MagicMock()
        return response

    client.get = mock_get
    client.__aenter__ = AsyncMock(return_value=client)
    client.__aexit__ = AsyncMock(return_value=None)

    return client


@pytest.fixture
def sample_text_short():
    """
    Short text for chunking tests.

    Returns:
        String with less than default chunk size
    """
    return "This is a short text that fits in one chunk."


@pytest.fixture
def sample_text_long():
    """
    Long text for chunking tests.

    Returns:
        String requiring multiple chunks
    """
    return "Lorem ipsum dolor sit amet. " * 100  # ~2700 characters


@pytest.fixture
def sample_text_chunks():
    """
    Sample text chunks with metadata for testing.

    Returns:
        List of TextChunk objects
    """
    from src.models.ingest import TextChunk

    return [
        TextChunk(
            id=uuid4(),
            text="First chunk of text with sufficient content for testing embeddings.",
            url="https://example.com/page1",
            title="Test Page 1",
            chunk_index=0,
            vector=[0.1] * 1024
        ),
        TextChunk(
            id=uuid4(),
            text="Second chunk of text with different content for variety in testing.",
            url="https://example.com/page1",
            title="Test Page 1",
            chunk_index=1,
            vector=[0.2] * 1024
        ),
        TextChunk(
            id=uuid4(),
            text="Third chunk from a different page to test multi-page scenarios.",
            url="https://example.com/page2",
            title="Test Page 2",
            chunk_index=0,
            vector=[0.3] * 1024
        ),
    ]
