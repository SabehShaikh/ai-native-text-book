"""
Integration tests for GET /health endpoint.

Tests the complete health check flow including Qdrant connectivity,
collection existence checks, and graceful error handling.
"""

import time
from datetime import datetime
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient
from qdrant_client.http.exceptions import UnexpectedResponse

from src.main import app


client = TestClient(app)


@pytest.mark.asyncio
async def test_health_check_healthy_with_collection(mock_qdrant_client):
    """
    Test GET /health returns healthy status when Qdrant is connected
    and collection exists with data.

    Verifies:
    - Status code is 200
    - Response indicates healthy status
    - Qdrant status is connected
    - Collection exists
    - Collection count is returned
    - Response time < 100ms
    - Timestamp is in ISO8601 format
    """
    with patch("src.api.health.VectorStoreService") as MockVectorStore:
        # Configure mock
        mock_service = MagicMock()
        mock_service.health_check.return_value = True
        mock_service.get_collection_count.return_value = 150
        MockVectorStore.return_value = mock_service

        # Make request and measure time
        start_time = time.time()
        response = client.get("/health")
        duration_ms = (time.time() - start_time) * 1000

        # Verify response
        assert response.status_code == 200

        data = response.json()
        assert data["status"] == "healthy"
        assert data["qdrant_status"] == "connected"
        assert data["collection_exists"] is True
        assert data["collection_count"] == 150
        assert "timestamp" in data

        # Verify timestamp format (ISO8601 with Z suffix)
        timestamp = data["timestamp"]
        assert timestamp.endswith("Z")
        # Verify it can be parsed as ISO8601
        datetime.fromisoformat(timestamp.replace("Z", "+00:00"))

        # Verify response time < 100ms target
        assert duration_ms < 100, f"Response time {duration_ms}ms exceeds 100ms target"

        # Verify service methods were called
        mock_service.health_check.assert_called_once()
        mock_service.get_collection_count.assert_called_once()


@pytest.mark.asyncio
async def test_health_check_unhealthy_qdrant_unreachable():
    """
    Test GET /health returns unhealthy status when Qdrant is unreachable.

    Verifies:
    - Status code is 200 (always returns 200, never raises)
    - Response indicates unhealthy status
    - Qdrant status is disconnected
    - Collection exists is False
    - Collection count is None
    - Graceful degradation (no exceptions)
    """
    with patch("src.api.health.VectorStoreService") as MockVectorStore:
        # Configure mock to simulate Qdrant unreachable
        mock_service = MagicMock()
        mock_service.health_check.return_value = False
        MockVectorStore.return_value = mock_service

        # Make request
        response = client.get("/health")

        # Verify response - always returns 200, even when unhealthy
        assert response.status_code == 200

        data = response.json()
        assert data["status"] == "unhealthy"
        assert data["qdrant_status"] == "disconnected"
        assert data["collection_exists"] is False
        assert data["collection_count"] is None
        assert "timestamp" in data

        # Verify timestamp format
        timestamp = data["timestamp"]
        assert timestamp.endswith("Z")
        datetime.fromisoformat(timestamp.replace("Z", "+00:00"))

        # Verify health_check was called but get_collection_count was not
        mock_service.health_check.assert_called_once()
        mock_service.get_collection_count.assert_not_called()


@pytest.mark.asyncio
async def test_health_check_collection_not_initialized():
    """
    Test GET /health when Qdrant is connected but collection doesn't exist.

    Verifies:
    - Status code is 200
    - Response indicates unhealthy status
    - Qdrant status is connected
    - Collection exists is False
    - Collection count is None
    - Handles collection query failures gracefully
    """
    with patch("src.api.health.VectorStoreService") as MockVectorStore:
        # Configure mock - health_check passes but get_collection_count fails
        mock_service = MagicMock()
        mock_service.health_check.return_value = True
        mock_service.get_collection_count.side_effect = UnexpectedResponse(
            status_code=404,
            reason_phrase="Collection not found"
        )
        MockVectorStore.return_value = mock_service

        # Make request
        response = client.get("/health")

        # Verify response
        assert response.status_code == 200

        data = response.json()
        assert data["status"] == "unhealthy"
        assert data["qdrant_status"] == "connected"
        assert data["collection_exists"] is False
        assert data["collection_count"] is None
        assert "timestamp" in data

        # Verify both methods were called
        mock_service.health_check.assert_called_once()
        mock_service.get_collection_count.assert_called_once()


@pytest.mark.asyncio
async def test_health_check_initialization_error():
    """
    Test GET /health handles VectorStoreService initialization errors gracefully.

    Verifies:
    - Status code is 200 (never raises exceptions)
    - Response indicates unhealthy status
    - Qdrant status is disconnected
    - Collection exists is False
    - Collection count is None
    """
    with patch("src.api.health.VectorStoreService") as MockVectorStore:
        # Configure mock to raise exception during initialization
        MockVectorStore.side_effect = Exception("Connection refused")

        # Make request
        response = client.get("/health")

        # Verify response - should handle gracefully
        assert response.status_code == 200

        data = response.json()
        assert data["status"] == "unhealthy"
        assert data["qdrant_status"] == "disconnected"
        assert data["collection_exists"] is False
        assert data["collection_count"] is None
        assert "timestamp" in data


@pytest.mark.asyncio
async def test_health_check_response_time_performance():
    """
    Test GET /health meets <100ms response time requirement.

    Verifies:
    - Response time is under 100ms
    - Health check completes quickly even under load
    """
    with patch("src.api.health.VectorStoreService") as MockVectorStore:
        # Configure mock
        mock_service = MagicMock()
        mock_service.health_check.return_value = True
        mock_service.get_collection_count.return_value = 200
        MockVectorStore.return_value = mock_service

        # Run multiple times to check consistent performance
        timings = []
        for _ in range(10):
            start_time = time.time()
            response = client.get("/health")
            duration_ms = (time.time() - start_time) * 1000
            timings.append(duration_ms)

            assert response.status_code == 200

        # Verify all calls were under 100ms
        max_duration = max(timings)
        avg_duration = sum(timings) / len(timings)

        assert max_duration < 100, f"Max response time {max_duration}ms exceeds 100ms"
        assert avg_duration < 50, f"Avg response time {avg_duration}ms should be well under target"


@pytest.mark.asyncio
async def test_health_check_timestamp_format():
    """
    Test GET /health returns properly formatted ISO8601 timestamp.

    Verifies:
    - Timestamp ends with Z (UTC indicator)
    - Timestamp can be parsed as ISO8601
    - Timestamp is recent (within last 5 seconds)
    """
    with patch("src.api.health.VectorStoreService") as MockVectorStore:
        # Configure mock
        mock_service = MagicMock()
        mock_service.health_check.return_value = True
        mock_service.get_collection_count.return_value = 100
        MockVectorStore.return_value = mock_service

        # Make request
        before_request = datetime.utcnow()
        response = client.get("/health")
        after_request = datetime.utcnow()

        assert response.status_code == 200

        data = response.json()
        timestamp_str = data["timestamp"]

        # Verify format
        assert timestamp_str.endswith("Z"), "Timestamp should end with Z"

        # Parse timestamp
        timestamp = datetime.fromisoformat(timestamp_str.replace("Z", "+00:00"))

        # Verify it's recent (within the request window)
        assert before_request <= timestamp.replace(tzinfo=None) <= after_request, \
            "Timestamp should be within request time window"


@pytest.mark.asyncio
async def test_health_check_empty_collection():
    """
    Test GET /health when collection exists but is empty.

    Verifies:
    - Status code is 200
    - Response indicates healthy status (collection exists, even if empty)
    - Collection count is 0
    """
    with patch("src.api.health.VectorStoreService") as MockVectorStore:
        # Configure mock - collection exists but is empty
        mock_service = MagicMock()
        mock_service.health_check.return_value = True
        mock_service.get_collection_count.return_value = 0
        MockVectorStore.return_value = mock_service

        # Make request
        response = client.get("/health")

        # Verify response
        assert response.status_code == 200

        data = response.json()
        assert data["status"] == "healthy"
        assert data["qdrant_status"] == "connected"
        assert data["collection_exists"] is True
        assert data["collection_count"] == 0
        assert "timestamp" in data
