"""
Data models for health check endpoints.

This module contains Pydantic models for the health check API response.
"""

from typing import Optional

from pydantic import BaseModel, Field


class HealthResponse(BaseModel):
    """
    Response model for GET /health endpoint.

    Attributes:
        status: Overall health status - "healthy" or "unhealthy"
        qdrant_status: Qdrant connectivity status - "connected" or "disconnected"
        collection_exists: Whether the Qdrant collection exists
        collection_count: Optional number of vectors in the collection
        timestamp: ISO8601 timestamp of the health check
    """
    status: str = Field(
        ...,
        description="Overall health status: healthy | unhealthy"
    )
    qdrant_status: str = Field(
        ...,
        description="Qdrant connection status: connected | disconnected"
    )
    collection_exists: bool = Field(
        ...,
        description="Whether the Qdrant collection exists"
    )
    collection_count: Optional[int] = Field(
        None,
        ge=0,
        description="Number of vectors in the collection (if available)"
    )
    timestamp: str = Field(
        ...,
        description="ISO8601 timestamp of the health check"
    )
