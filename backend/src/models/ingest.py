"""
Data models for ingestion endpoints and processes.

This module contains Pydantic models for API request/response and internal
dataclasses for ingestion pipeline processing.
"""

from dataclasses import dataclass
from typing import List, Optional
from uuid import UUID

from pydantic import BaseModel, Field, HttpUrl


class IngestRequest(BaseModel):
    """
    Request model for POST /ingest endpoint.

    Attributes:
        sitemap_url: Optional URL override for sitemap location.
                    Defaults to config.SITEMAP_URL if not provided.
    """
    sitemap_url: Optional[HttpUrl] = Field(
        None,
        description="Override default sitemap URL"
    )


class IngestResponse(BaseModel):
    """
    Response model for POST /ingest endpoint.

    Attributes:
        status: Ingestion status - "success", "partial_success", or "failed"
        pages_processed: Number of pages successfully processed
        chunks_created: Total number of text chunks created and stored
        duration_seconds: Total ingestion time in seconds
        errors: Optional list of URLs that failed to process
    """
    status: str = Field(
        ...,
        description="success | partial_success | failed"
    )
    pages_processed: int = Field(
        ...,
        ge=0,
        description="Number of pages processed"
    )
    chunks_created: int = Field(
        ...,
        ge=0,
        description="Number of chunks created"
    )
    duration_seconds: float = Field(
        ...,
        ge=0,
        description="Total ingestion time"
    )
    errors: Optional[List[str]] = Field(
        None,
        description="URLs that failed to process"
    )


@dataclass
class IngestionResult:
    """
    Internal dataclass for ingestion pipeline results.

    Used to pass ingestion metrics between service layers.

    Attributes:
        pages_processed: Number of pages successfully processed
        chunks_created: Total number of chunks created
        duration_seconds: Total processing time in seconds
        errors: List of URLs that failed to process
    """
    pages_processed: int
    chunks_created: int
    duration_seconds: float
    errors: List[str]


@dataclass
class TextChunk:
    """
    Internal dataclass representing a text chunk with embedding.

    Used during ingestion pipeline and retrieval operations.

    Attributes:
        id: Unique identifier for the chunk
        text: The chunk's text content
        url: Source page URL
        title: Source page title
        chunk_index: Position index within the source page (0-based)
        vector: 1024-dimensional embedding vector from Cohere
    """
    id: UUID
    text: str
    url: str
    title: str
    chunk_index: int
    vector: List[float]
