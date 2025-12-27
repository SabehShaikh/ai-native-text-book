"""
API endpoint for ingestion operations.

Provides POST /ingest endpoint for admin to trigger textbook content ingestion.
Includes in-memory lock to prevent duplicate ingestion processes.
"""

import asyncio

from fastapi import APIRouter, Body, HTTPException

from src.config import settings
from src.logger import logger
from src.models.ingest import IngestRequest, IngestResponse
from src.services.ingestion import IngestionService
from src.utils.exceptions import SitemapFetchError

router = APIRouter()

# In-memory lock to prevent duplicate ingestion
ingestion_lock = asyncio.Lock()


@router.post("/ingest", response_model=IngestResponse)
async def ingest(
    request: IngestRequest = Body(default=IngestRequest())
) -> IngestResponse:
    """
    Ingest textbook content from sitemap.

    This endpoint triggers the complete ingestion pipeline:
    1. Fetch sitemap.xml
    2. Extract text from each page
    3. Chunk text with overlap
    4. Generate embeddings
    5. Store in vector database

    An in-memory lock prevents concurrent ingestion operations.

    Args:
        request: Optional IngestRequest with sitemap_url override

    Returns:
        IngestResponse with ingestion metrics and status

    Raises:
        HTTPException 409: If ingestion is already in progress
        HTTPException 400: If sitemap cannot be fetched or parsed
        HTTPException 503: If Cohere API fails
        HTTPException 500: If unexpected error occurs
    """
    # Check if ingestion is already running
    if ingestion_lock.locked():
        logger.warning("Ingestion already in progress")
        raise HTTPException(
            status_code=409,
            detail="Ingestion already in progress. Please wait for the current "
                   "ingestion to complete before starting a new one."
        )

    async with ingestion_lock:
        # Use provided sitemap URL or default from config
        sitemap_url = (
            str(request.sitemap_url) if request.sitemap_url
            else settings.SITEMAP_URL
        )

        logger.info(
            "Starting ingestion",
            extra={
                "sitemap_url": sitemap_url,
                "default_used": request.sitemap_url is None
            }
        )

        try:
            # Initialize ingestion service
            ingestion_service = IngestionService()

            # Execute ingestion pipeline
            result = await ingestion_service.ingest_from_sitemap(sitemap_url)

            # Determine status based on errors
            if not result.errors:
                status = "success"
            elif result.pages_processed > 0:
                status = "partial_success"
            else:
                status = "failed"

            logger.info(
                "Ingestion completed",
                extra={
                    "status": status,
                    "pages_processed": result.pages_processed,
                    "chunks_created": result.chunks_created,
                    "duration_seconds": result.duration_seconds,
                    "errors_count": len(result.errors)
                }
            )

            return IngestResponse(
                status=status,
                pages_processed=result.pages_processed,
                chunks_created=result.chunks_created,
                duration_seconds=result.duration_seconds,
                errors=result.errors if result.errors else None
            )

        except SitemapFetchError as e:
            logger.error(
                "Sitemap fetch failed",
                extra={
                    "sitemap_url": sitemap_url,
                    "error": str(e)
                }
            )
            raise HTTPException(
                status_code=400,
                detail=f"Cannot fetch or parse sitemap: {str(e)}"
            )

        except Exception as e:
            logger.error(
                "Ingestion failed with unexpected error",
                extra={
                    "sitemap_url": sitemap_url,
                    "error": str(e),
                    "error_type": type(e).__name__
                }
            )
            raise HTTPException(
                status_code=500,
                detail=f"Ingestion failed: {str(e)}"
            )
