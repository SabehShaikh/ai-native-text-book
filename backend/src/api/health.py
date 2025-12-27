"""
Health check API endpoint for system monitoring.

Provides GET /health endpoint for administrators and monitoring services
to check API health and Qdrant connectivity.
"""

from datetime import datetime

from fastapi import APIRouter

from src.logger import logger
from src.models.health import HealthResponse
from src.services.vector_store import VectorStoreService

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health() -> HealthResponse:
    """
    Check system health and Qdrant connectivity.

    Process:
    1. Check Qdrant connectivity using health_check()
    2. Get collection count if Qdrant is healthy
    3. Return health status with timestamp
    4. Always return 200 status with health indicator in response

    Returns:
        HealthResponse with status, Qdrant info, and timestamp

    Note:
        This endpoint never raises exceptions - it always returns 200
        with a status field indicating health. This ensures monitoring
        systems can reliably check health without triggering alerts
        on transient failures.
    """
    timestamp = datetime.utcnow().isoformat() + "Z"

    try:
        # Initialize vector store service
        vector_store_service = VectorStoreService()

        # Check Qdrant health
        is_healthy = await vector_store_service.health_check()

        if is_healthy:
            # Try to get collection count
            try:
                collection_count = await vector_store_service.get_collection_count()

                logger.info(
                    "Health check passed",
                    extra={
                        "endpoint": "/health",
                        "qdrant_status": "connected",
                        "collection_exists": True,
                        "collection_count": collection_count
                    }
                )

                return HealthResponse(
                    status="healthy",
                    qdrant_status="connected",
                    collection_exists=True,
                    collection_count=collection_count,
                    timestamp=timestamp
                )
            except Exception as e:
                # Qdrant connected but collection might not exist
                logger.warning(
                    "Health check: Qdrant connected but collection query failed",
                    extra={
                        "endpoint": "/health",
                        "error": str(e),
                        "error_type": type(e).__name__
                    }
                )

                return HealthResponse(
                    status="unhealthy",
                    qdrant_status="connected",
                    collection_exists=False,
                    collection_count=None,
                    timestamp=timestamp
                )
        else:
            # Qdrant health check failed
            logger.warning(
                "Health check failed: Qdrant unreachable",
                extra={
                    "endpoint": "/health",
                    "qdrant_status": "disconnected"
                }
            )

            return HealthResponse(
                status="unhealthy",
                qdrant_status="disconnected",
                collection_exists=False,
                collection_count=None,
                timestamp=timestamp
            )

    except Exception as e:
        # Catch any unexpected errors during initialization
        logger.error(
            "Health check: Unexpected error",
            extra={
                "endpoint": "/health",
                "error": str(e),
                "error_type": type(e).__name__
            }
        )

        return HealthResponse(
            status="unhealthy",
            qdrant_status="disconnected",
            collection_exists=False,
            collection_count=None,
            timestamp=timestamp
        )
