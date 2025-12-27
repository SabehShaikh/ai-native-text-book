"""
Integration tests for POST /chat endpoint.

Tests the complete chat flow including request validation, retrieval,
AI generation, and error handling with mocked external services.
"""

import json
import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

from fastapi.testclient import TestClient
from httpx import AsyncClient

from src.main import app
from src.models.ingest import TextChunk
from src.models.chat import RetrievalResult
from src.utils.exceptions import CohereAPIError, GeminiAPIError, QdrantError, QdrantTimeout


@pytest.fixture
def mock_successful_services():
    """Mock all services for successful chat flow."""
    # Mock embedding service
    mock_embedding = MagicMock()
    mock_embedding.embed_text = AsyncMock(return_value=[0.1] * 1024)

    # Mock vector store service
    mock_vector_store = MagicMock()
    search_results = [
        {
            "id": str(uuid4()),
            "text": f"Physical AI systems integrate artificial intelligence with physical world interactions through sensors and actuators.",
            "url": f"https://example.com/chapter1",
            "title": "Introduction to Physical AI",
            "chunk_index": 0,
            "score": 0.95
        },
        {
            "id": str(uuid4()),
            "text": "These systems enable robots to perceive their environment and make autonomous decisions.",
            "url": "https://example.com/chapter1",
            "title": "Introduction to Physical AI",
            "chunk_index": 1,
            "score": 0.90
        },
        {
            "id": str(uuid4()),
            "text": "Applications include warehouse automation, self-driving vehicles, and assistive robotics.",
            "url": "https://example.com/chapter2",
            "title": "Applications of Physical AI",
            "chunk_index": 0,
            "score": 0.85
        }
    ]
    mock_vector_store.search = AsyncMock(return_value=search_results)

    # Mock OpenAI client for Gemini
    mock_openai = MagicMock()

    # First response - tool call
    tool_call_response = MagicMock()
    tool_call_response.choices = [MagicMock()]
    tool_call_response.choices[0].message = MagicMock()
    tool_call_response.choices[0].message.content = None

    tool_call = MagicMock()
    tool_call.id = "call_123"
    tool_call.function.name = "retrieve"
    tool_call.function.arguments = json.dumps({"query": "physical AI"})

    tool_call_response.choices[0].message.tool_calls = [tool_call]

    # Second response - final answer
    final_response = MagicMock()
    final_response.choices = [MagicMock()]
    final_response.choices[0].message = MagicMock()
    final_response.choices[0].message.content = "Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators. As described in the Introduction to Physical AI chapter, these systems enable autonomous behavior in real environments. Applications include warehouse automation and self-driving vehicles."

    call_count = 0

    async def mock_create(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            return tool_call_response
        else:
            return final_response

    mock_openai.chat.completions.create = mock_create

    return {
        "embedding": mock_embedding,
        "vector_store": mock_vector_store,
        "openai": mock_openai
    }


@pytest.mark.asyncio
async def test_chat_successful_query(mock_successful_services):
    """Test successful chat query with mocked services."""
    with patch('src.services.embedding.EmbeddingService') as mock_emb_class, \
         patch('src.services.vector_store.VectorStoreService') as mock_vs_class, \
         patch('src.services.agent.AsyncOpenAI') as mock_openai_class:

        # Configure mocks
        mock_emb_class.return_value = mock_successful_services["embedding"]
        mock_vs_class.return_value = mock_successful_services["vector_store"]
        mock_openai_class.return_value = mock_successful_services["openai"]

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"query": "What is physical AI?"}
            )

        # Verify response
        assert response.status_code == 200

        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert "response_time" in data
        assert "request_id" in data

        # Verify answer content
        assert len(data["answer"]) > 0
        assert "physical" in data["answer"].lower() or "AI" in data["answer"]

        # Verify sources included
        assert isinstance(data["sources"], list)
        assert len(data["sources"]) >= 1
        assert all(url.startswith("http") for url in data["sources"])

        # Verify response time is reasonable
        assert data["response_time"] >= 0
        assert data["response_time"] < 3.0  # Should be < 3 seconds

        # Verify request_id is UUID format
        assert len(data["request_id"]) == 36  # UUID with hyphens


@pytest.mark.asyncio
async def test_chat_empty_query():
    """Test that empty query returns 400 error."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/chat",
            json={"query": ""}
        )

    assert response.status_code == 422  # Pydantic validation error
    data = response.json()
    assert "detail" in data


@pytest.mark.asyncio
async def test_chat_whitespace_only_query():
    """Test that whitespace-only query returns 400 error."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/chat",
            json={"query": "   "}
        )

    assert response.status_code == 422  # Pydantic validation error
    data = response.json()
    assert "detail" in data


@pytest.mark.asyncio
async def test_chat_long_query():
    """Test that query exceeding max length returns 400 error."""
    long_query = "a" * 501  # Exceeds 500 char limit

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/chat",
            json={"query": long_query}
        )

    assert response.status_code == 422  # Pydantic validation error
    data = response.json()
    assert "detail" in data


@pytest.mark.asyncio
async def test_chat_no_relevant_content(mock_successful_services):
    """Test response when no relevant content is found (I don't know response)."""
    # Mock empty search results
    mock_vector_store = MagicMock()
    mock_vector_store.search = AsyncMock(return_value=[])

    # Mock OpenAI to return "I don't know" response
    mock_openai = MagicMock()

    tool_call_response = MagicMock()
    tool_call_response.choices = [MagicMock()]
    tool_call_response.choices[0].message = MagicMock()
    tool_call_response.choices[0].message.content = None

    tool_call = MagicMock()
    tool_call.id = "call_123"
    tool_call.function.name = "retrieve"
    tool_call.function.arguments = json.dumps({"query": "nonexistent topic"})

    tool_call_response.choices[0].message.tool_calls = [tool_call]

    final_response = MagicMock()
    final_response.choices = [MagicMock()]
    final_response.choices[0].message = MagicMock()
    final_response.choices[0].message.content = "I don't have information about this in the textbook"

    call_count = 0

    async def mock_create(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            return tool_call_response
        else:
            return final_response

    mock_openai.chat.completions.create = mock_create

    with patch('src.services.embedding.EmbeddingService') as mock_emb_class, \
         patch('src.services.vector_store.VectorStoreService') as mock_vs_class, \
         patch('src.services.agent.AsyncOpenAI') as mock_openai_class:

        mock_emb_class.return_value = mock_successful_services["embedding"]
        mock_vs_class.return_value = mock_vector_store
        mock_openai_class.return_value = mock_openai

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"query": "What is quantum entanglement?"}
            )

        assert response.status_code == 200
        data = response.json()

        # Verify "I don't know" type response
        assert "don't have information" in data["answer"].lower() or \
               "not in the textbook" in data["answer"].lower()
        assert len(data["sources"]) == 0


@pytest.mark.asyncio
async def test_chat_cohere_failure():
    """Test that Cohere API failure returns 503 error."""
    mock_embedding = MagicMock()
    mock_embedding.embed_text = AsyncMock(
        side_effect=CohereAPIError("Rate limit exceeded")
    )

    with patch('src.services.embedding.EmbeddingService') as mock_emb_class:
        mock_emb_class.return_value = mock_embedding

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"query": "What is physical AI?"}
            )

        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "unavailable" in data["detail"].lower()
        assert "embedding" in data["detail"].lower()


@pytest.mark.asyncio
async def test_chat_gemini_failure(mock_successful_services):
    """Test that Gemini API failure returns 503 error."""
    # Mock OpenAI to raise error
    mock_openai = MagicMock()

    async def mock_create_error(*args, **kwargs):
        raise Exception("API connection timeout")

    mock_openai.chat.completions.create = mock_create_error

    with patch('src.services.embedding.EmbeddingService') as mock_emb_class, \
         patch('src.services.vector_store.VectorStoreService') as mock_vs_class, \
         patch('src.services.agent.AsyncOpenAI') as mock_openai_class:

        mock_emb_class.return_value = mock_successful_services["embedding"]
        mock_vs_class.return_value = mock_successful_services["vector_store"]
        mock_openai_class.return_value = mock_openai

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"query": "What is physical AI?"}
            )

        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "unavailable" in data["detail"].lower()
        assert "answer generation" in data["detail"].lower()


@pytest.mark.asyncio
async def test_chat_qdrant_failure(mock_successful_services):
    """Test that Qdrant failure returns 503 error."""
    mock_vector_store = MagicMock()
    mock_vector_store.search = AsyncMock(
        side_effect=QdrantError("Connection refused")
    )

    with patch('src.services.embedding.EmbeddingService') as mock_emb_class, \
         patch('src.services.vector_store.VectorStoreService') as mock_vs_class:

        mock_emb_class.return_value = mock_successful_services["embedding"]
        mock_vs_class.return_value = mock_vector_store

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"query": "What is physical AI?"}
            )

        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "unavailable" in data["detail"].lower()
        assert "search" in data["detail"].lower()


@pytest.mark.asyncio
async def test_chat_qdrant_timeout(mock_successful_services):
    """Test that Qdrant timeout returns 504 error."""
    mock_vector_store = MagicMock()
    mock_vector_store.search = AsyncMock(
        side_effect=QdrantTimeout("Search exceeded 5 second timeout")
    )

    with patch('src.services.embedding.EmbeddingService') as mock_emb_class, \
         patch('src.services.vector_store.VectorStoreService') as mock_vs_class:

        mock_emb_class.return_value = mock_successful_services["embedding"]
        mock_vs_class.return_value = mock_vector_store

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"query": "What is physical AI?"}
            )

        assert response.status_code == 504
        data = response.json()
        assert "detail" in data
        assert "timeout" in data["detail"].lower()


@pytest.mark.asyncio
async def test_chat_response_time_under_3_seconds(mock_successful_services):
    """Test that response time is under 3 seconds."""
    with patch('src.services.embedding.EmbeddingService') as mock_emb_class, \
         patch('src.services.vector_store.VectorStoreService') as mock_vs_class, \
         patch('src.services.agent.AsyncOpenAI') as mock_openai_class:

        mock_emb_class.return_value = mock_successful_services["embedding"]
        mock_vs_class.return_value = mock_successful_services["vector_store"]
        mock_openai_class.return_value = mock_successful_services["openai"]

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"query": "What is physical AI?"}
            )

        assert response.status_code == 200
        data = response.json()

        # Verify response time is under 3 seconds
        assert data["response_time"] < 3.0
        # Also verify it's a reasonable value (not negative or zero)
        assert data["response_time"] > 0


@pytest.mark.asyncio
async def test_chat_source_urls_included(mock_successful_services):
    """Test that source URLs are included in response."""
    with patch('src.services.embedding.EmbeddingService') as mock_emb_class, \
         patch('src.services.vector_store.VectorStoreService') as mock_vs_class, \
         patch('src.services.agent.AsyncOpenAI') as mock_openai_class:

        mock_emb_class.return_value = mock_successful_services["embedding"]
        mock_vs_class.return_value = mock_successful_services["vector_store"]
        mock_openai_class.return_value = mock_successful_services["openai"]

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"query": "What is physical AI?"}
            )

        assert response.status_code == 200
        data = response.json()

        # Verify sources are included
        assert "sources" in data
        assert isinstance(data["sources"], list)
        assert len(data["sources"]) >= 1

        # Verify sources are valid URLs
        for source in data["sources"]:
            assert isinstance(source, str)
            assert source.startswith("http://") or source.startswith("https://")
            assert len(source) > 10  # Reasonable URL length

        # Verify sources match our mock data (2 unique URLs)
        assert len(data["sources"]) == 2
        assert "https://example.com/chapter1" in data["sources"]
        assert "https://example.com/chapter2" in data["sources"]


@pytest.mark.asyncio
async def test_chat_request_id_generated():
    """Test that each request gets a unique request_id."""
    with patch('src.services.embedding.EmbeddingService') as mock_emb_class, \
         patch('src.services.vector_store.VectorStoreService') as mock_vs_class, \
         patch('src.services.agent.AsyncOpenAI') as mock_openai_class:

        # Setup mocks with minimal responses
        mock_embedding = MagicMock()
        mock_embedding.embed_text = AsyncMock(return_value=[0.1] * 1024)

        mock_vector_store = MagicMock()
        mock_vector_store.search = AsyncMock(return_value=[])

        mock_openai = MagicMock()
        response_obj = MagicMock()
        response_obj.choices = [MagicMock()]
        response_obj.choices[0].message = MagicMock()
        response_obj.choices[0].message.content = "Test answer"
        response_obj.choices[0].message.tool_calls = None

        async def mock_create(*args, **kwargs):
            return response_obj

        mock_openai.chat.completions.create = mock_create

        mock_emb_class.return_value = mock_embedding
        mock_vs_class.return_value = mock_vector_store
        mock_openai_class.return_value = mock_openai

        # Make two requests
        async with AsyncClient(app=app, base_url="http://test") as client:
            response1 = await client.post(
                "/chat",
                json={"query": "First question"}
            )
            response2 = await client.post(
                "/chat",
                json={"query": "Second question"}
            )

        data1 = response1.json()
        data2 = response2.json()

        # Verify both have request_ids
        assert "request_id" in data1
        assert "request_id" in data2

        # Verify request_ids are different
        assert data1["request_id"] != data2["request_id"]

        # Verify format (UUID)
        assert len(data1["request_id"]) == 36
        assert len(data2["request_id"]) == 36


@pytest.mark.asyncio
async def test_chat_query_length_validation():
    """Test edge cases for query length validation."""
    # Test minimum valid length (1 character)
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/chat",
            json={"query": "?"}
        )

    # Should accept single character (though services might fail, that's OK)
    assert response.status_code in [200, 503]  # 200 or service error

    # Test maximum valid length (500 characters)
    valid_max_query = "a" * 500

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/chat",
            json={"query": valid_max_query}
        )

    # Should accept 500 characters
    assert response.status_code in [200, 503]  # 200 or service error

    # Test over maximum (501 characters)
    invalid_query = "a" * 501

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/chat",
            json={"query": invalid_query}
        )

    # Should reject
    assert response.status_code == 422
