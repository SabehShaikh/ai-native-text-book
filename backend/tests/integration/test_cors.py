"""Integration tests for CORS configuration.

Tests CORS headers for allowed and unauthorized origins across all endpoints.
"""

import pytest
from fastapi.testclient import TestClient

from src.main import app
from src.config import settings


@pytest.fixture
def client() -> TestClient:
    """Create test client for the FastAPI application.

    Returns:
        TestClient instance
    """
    return TestClient(app)


def test_cors_chat_endpoint_allowed_origin(client: TestClient) -> None:
    """Test POST /chat with allowed origin includes correct CORS headers.

    Args:
        client: FastAPI test client
    """
    response = client.options(
        "/chat",
        headers={"Origin": settings.FRONTEND_URL}
    )

    assert response.status_code == 200
    assert "access-control-allow-origin" in response.headers
    assert response.headers["access-control-allow-origin"] == settings.FRONTEND_URL
    assert "access-control-allow-credentials" in response.headers
    assert response.headers["access-control-allow-credentials"] == "true"


def test_cors_ingest_endpoint_allowed_origin(client: TestClient) -> None:
    """Test POST /ingest with allowed origin includes correct CORS headers.

    Args:
        client: FastAPI test client
    """
    response = client.options(
        "/ingest",
        headers={"Origin": settings.FRONTEND_URL}
    )

    assert response.status_code == 200
    assert "access-control-allow-origin" in response.headers
    assert response.headers["access-control-allow-origin"] == settings.FRONTEND_URL
    assert "access-control-allow-credentials" in response.headers
    assert response.headers["access-control-allow-credentials"] == "true"


def test_cors_health_endpoint_allowed_origin(client: TestClient) -> None:
    """Test GET /health with allowed origin includes correct CORS headers.

    Args:
        client: FastAPI test client
    """
    response = client.options(
        "/health",
        headers={"Origin": settings.FRONTEND_URL}
    )

    assert response.status_code == 200
    assert "access-control-allow-origin" in response.headers
    assert response.headers["access-control-allow-origin"] == settings.FRONTEND_URL
    assert "access-control-allow-credentials" in response.headers
    assert response.headers["access-control-allow-credentials"] == "true"


def test_cors_unauthorized_origin(client: TestClient) -> None:
    """Test request from unauthorized origin does not include CORS headers.

    Args:
        client: FastAPI test client
    """
    unauthorized_origin = "https://malicious-site.com"

    response = client.options(
        "/chat",
        headers={"Origin": unauthorized_origin}
    )

    assert response.status_code == 200
    # CORS middleware should not include allow-origin header for unauthorized origins
    if "access-control-allow-origin" in response.headers:
        assert response.headers["access-control-allow-origin"] != unauthorized_origin


def test_cors_preflight_chat(client: TestClient) -> None:
    """Test OPTIONS preflight request for /chat endpoint.

    Args:
        client: FastAPI test client
    """
    response = client.options(
        "/chat",
        headers={
            "Origin": settings.FRONTEND_URL,
            "Access-Control-Request-Method": "POST",
            "Access-Control-Request-Headers": "content-type",
        }
    )

    assert response.status_code == 200
    assert "access-control-allow-origin" in response.headers
    assert response.headers["access-control-allow-origin"] == settings.FRONTEND_URL
    assert "access-control-allow-methods" in response.headers
    assert "POST" in response.headers["access-control-allow-methods"]


def test_cors_actual_request_chat(client: TestClient, mocker) -> None:
    """Test actual POST request to /chat includes CORS headers.

    Args:
        client: FastAPI test client
        mocker: pytest-mock fixture
    """
    # Mock the retrieval and agent services to avoid actual API calls
    mock_retrieval = mocker.patch("src.api.chat.retrieval_service.retrieve")
    mock_agent = mocker.patch("src.api.chat.agent_service.answer_question")

    mock_retrieval.return_value = mocker.MagicMock(
        sources=["https://example.com/page1"]
    )
    mock_agent.return_value = "Test answer"

    response = client.post(
        "/chat",
        json={"query": "What is physics?"},
        headers={"Origin": settings.FRONTEND_URL}
    )

    assert "access-control-allow-origin" in response.headers
    assert response.headers["access-control-allow-origin"] == settings.FRONTEND_URL
    assert "access-control-allow-credentials" in response.headers
    assert response.headers["access-control-allow-credentials"] == "true"


def test_cors_actual_request_health(client: TestClient, mocker) -> None:
    """Test actual GET request to /health includes CORS headers.

    Args:
        client: FastAPI test client
        mocker: pytest-mock fixture
    """
    # Mock the vector store service
    mock_health_check = mocker.patch(
        "src.api.health.vector_store_service.health_check"
    )
    mock_get_count = mocker.patch(
        "src.api.health.vector_store_service.get_collection_count"
    )

    mock_health_check.return_value = True
    mock_get_count.return_value = 100

    response = client.get(
        "/health",
        headers={"Origin": settings.FRONTEND_URL}
    )

    assert "access-control-allow-origin" in response.headers
    assert response.headers["access-control-allow-origin"] == settings.FRONTEND_URL
    assert "access-control-allow-credentials" in response.headers
    assert response.headers["access-control-allow-credentials"] == "true"
