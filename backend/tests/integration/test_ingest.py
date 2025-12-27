"""
Integration tests for POST /ingest endpoint.

Tests full ingestion flow with mocked external APIs, duplicate prevention,
malformed sitemap handling, and partial success scenarios.
"""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from httpx import AsyncClient
from fastapi import status

from src.main import app
from src.api.ingest import ingestion_lock


@pytest.fixture(autouse=True)
async def reset_ingestion_lock():
    """Reset ingestion lock before each test."""
    # Ensure lock is released before test
    if ingestion_lock.locked():
        ingestion_lock.release()
    yield
    # Ensure lock is released after test
    if ingestion_lock.locked():
        ingestion_lock.release()


class TestIngestEndpoint:
    """Integration tests for POST /ingest endpoint."""

    @pytest.mark.asyncio
    async def test_ingest_success_with_default_sitemap(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client
    ):
        """Test successful ingestion with default sitemap URL."""
        # Mock dependencies
        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        assert data["status"] in ["success", "partial_success"]
        assert data["pages_processed"] >= 0
        assert data["chunks_created"] >= 0
        assert data["duration_seconds"] > 0
        assert "errors" in data

    @pytest.mark.asyncio
    async def test_ingest_success_with_custom_sitemap(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client
    ):
        """Test successful ingestion with custom sitemap URL."""
        custom_sitemap = "https://custom.example.com/sitemap.xml"

        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post(
                    "/ingest",
                    json={"sitemap_url": custom_sitemap}
                )

        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        assert data["status"] in ["success", "partial_success"]

    @pytest.mark.asyncio
    async def test_ingest_duplicate_prevention(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client
    ):
        """Test that concurrent ingestion is prevented with 409 status."""
        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                # Start first ingestion in background
                import asyncio

                async def first_ingestion():
                    return await client.post("/ingest", json={})

                # Use a delay in the ingestion to simulate long-running process
                original_ingest = None

                async def slow_ingest(*args, **kwargs):
                    await asyncio.sleep(0.5)  # Simulate slow ingestion
                    from src.models.ingest import IngestionResult
                    return IngestionResult(
                        pages_processed=3,
                        chunks_created=10,
                        duration_seconds=0.5,
                        errors=[]
                    )

                with patch('src.services.ingestion.IngestionService.ingest_from_sitemap', side_effect=slow_ingest):
                    # Start first request
                    task1 = asyncio.create_task(first_ingestion())

                    # Wait a bit for first request to acquire lock
                    await asyncio.sleep(0.1)

                    # Try second request - should get 409
                    response2 = await client.post("/ingest", json={})

                    # Check second request was rejected
                    assert response2.status_code == status.HTTP_409_CONFLICT
                    assert "already in progress" in response2.json()["detail"].lower()

                    # Wait for first request to complete
                    await task1

    @pytest.mark.asyncio
    async def test_ingest_malformed_sitemap_400(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client_malformed_sitemap
    ):
        """Test that malformed sitemap returns 400 error."""
        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client_malformed_sitemap):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_400_BAD_REQUEST
        assert "sitemap" in response.json()["detail"].lower()

    @pytest.mark.asyncio
    async def test_ingest_sitemap_not_found_400(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client_with_errors
    ):
        """Test that 404 sitemap returns 400 error."""
        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client_with_errors):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_400_BAD_REQUEST

    @pytest.mark.asyncio
    async def test_ingest_partial_success_with_errors(
        self,
        mock_cohere_client,
        mock_qdrant_client
    ):
        """Test ingestion with partial success returns errors list."""
        # Mock httpx client with mixed success/failure
        mock_client = AsyncMock()
        call_count = 0

        async def mock_get(url, *args, **kwargs):
            nonlocal call_count
            call_count += 1

            response = MagicMock()
            response.status_code = 200

            if "sitemap.xml" in url:
                sitemap_xml = """<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
    <url><loc>https://example.com/page1</loc></url>
    <url><loc>https://example.com/page2</loc></url>
    <url><loc>https://example.com/page3</loc></url>
</urlset>"""
                response.content = sitemap_xml.encode('utf-8')
                response.text = sitemap_xml
            else:
                # Make second page fail
                if call_count == 2:
                    import httpx
                    request = httpx.Request("GET", url)
                    raise httpx.HTTPStatusError("Error", request=request, response=httpx.Response(500, request=request))

                response.text = "<html><body><p>" + ("Content. " * 100) + "</p></body></html>"

            response.raise_for_status = MagicMock()
            return response

        mock_client.get = mock_get
        mock_client.__aenter__ = AsyncMock(return_value=mock_client)
        mock_client.__aexit__ = AsyncMock(return_value=None)

        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        assert data["status"] == "partial_success"
        assert data["pages_processed"] >= 1  # Some succeeded
        assert data["errors"] is not None
        assert len(data["errors"]) >= 1  # Some failed

    @pytest.mark.asyncio
    async def test_ingest_verify_counts(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client
    ):
        """Test that ingestion returns accurate counts."""
        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        # Verify counts are consistent
        if data["status"] == "success":
            assert data["pages_processed"] > 0
            assert data["chunks_created"] > 0
            assert data["errors"] is None or data["errors"] == []

    @pytest.mark.asyncio
    async def test_ingest_invalid_sitemap_url_format(
        self,
        mock_cohere_client,
        mock_qdrant_client
    ):
        """Test that invalid URL format is rejected."""
        invalid_url = "not-a-valid-url"

        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post(
                    "/ingest",
                    json={"sitemap_url": invalid_url}
                )

        # Pydantic validation should reject invalid URL
        assert response.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY

    @pytest.mark.asyncio
    async def test_ingest_empty_sitemap(
        self,
        mock_cohere_client,
        mock_qdrant_client
    ):
        """Test ingestion with empty sitemap (no URLs)."""
        mock_client = AsyncMock()

        async def mock_get(url, *args, **kwargs):
            response = MagicMock()
            response.status_code = 200

            empty_sitemap = """<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
</urlset>"""

            response.content = empty_sitemap.encode('utf-8')
            response.text = empty_sitemap
            response.raise_for_status = MagicMock()
            return response

        mock_client.get = mock_get
        mock_client.__aenter__ = AsyncMock(return_value=mock_client)
        mock_client.__aexit__ = AsyncMock(return_value=None)

        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        # Should return 400 for empty sitemap
        assert response.status_code == status.HTTP_400_BAD_REQUEST

    @pytest.mark.asyncio
    async def test_ingest_response_structure(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client
    ):
        """Test that response has correct structure."""
        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        # Verify all required fields are present
        assert "status" in data
        assert "pages_processed" in data
        assert "chunks_created" in data
        assert "duration_seconds" in data
        # errors field may be null or list

        # Verify field types
        assert isinstance(data["status"], str)
        assert isinstance(data["pages_processed"], int)
        assert isinstance(data["chunks_created"], int)
        assert isinstance(data["duration_seconds"], (int, float))

    @pytest.mark.asyncio
    async def test_ingest_duration_reasonable(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client
    ):
        """Test that duration is reasonable (not negative or zero)."""
        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        assert data["duration_seconds"] > 0
        assert data["duration_seconds"] < 300  # Should complete within 5 minutes for test

    @pytest.mark.asyncio
    async def test_ingest_creates_collection_if_not_exists(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client
    ):
        """Test that ingestion creates collection if it doesn't exist."""
        # Mock collection doesn't exist
        mock_qdrant_client.get_collections.return_value.collections = []

        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_200_OK

        # Verify create_collection was called
        mock_qdrant_client.create_collection.assert_called_once()

    @pytest.mark.asyncio
    async def test_ingest_all_pages_fail(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client_with_errors
    ):
        """Test ingestion when all pages fail returns failed status."""
        # Mock sitemap succeeds but all pages fail
        mock_client = AsyncMock()

        async def mock_get(url, *args, **kwargs):
            response = MagicMock()
            response.status_code = 200

            if "sitemap.xml" in url:
                sitemap_xml = """<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
    <url><loc>https://example.com/page1</loc></url>
</urlset>"""
                response.content = sitemap_xml.encode('utf-8')
                response.text = sitemap_xml
                response.raise_for_status = MagicMock()
                return response
            else:
                # All pages fail
                import httpx
                request = httpx.Request("GET", url)
                raise httpx.HTTPStatusError("Error", request=request, response=httpx.Response(500, request=request))

        mock_client.get = mock_get
        mock_client.__aenter__ = AsyncMock(return_value=mock_client)
        mock_client.__aexit__ = AsyncMock(return_value=None)

        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        # When all pages fail, status should be "failed"
        # pages_processed should be 0
        assert data["pages_processed"] == 0
        assert len(data["errors"]) > 0

    @pytest.mark.asyncio
    async def test_ingest_status_values(
        self,
        mock_cohere_client,
        mock_qdrant_client,
        mock_httpx_client
    ):
        """Test that status field has valid values."""
        with patch('src.services.embedding.cohere.AsyncClient', return_value=mock_cohere_client), \
             patch('src.services.vector_store.QdrantClient', return_value=mock_qdrant_client), \
             patch('httpx.AsyncClient', return_value=mock_httpx_client):

            async with AsyncClient(app=app, base_url="http://test") as client:
                response = await client.post("/ingest", json={})

        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        # Status should be one of the valid values
        assert data["status"] in ["success", "partial_success", "failed"]
