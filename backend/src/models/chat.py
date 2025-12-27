"""
Data models for chat endpoints and retrieval processes.

This module contains Pydantic models for chat API request/response and internal
dataclasses for retrieval and search operations.
"""

from dataclasses import dataclass
from typing import List

from pydantic import BaseModel, Field, field_validator

from src.models.ingest import TextChunk


class ChatRequest(BaseModel):
    """
    Request model for POST /chat endpoint.

    Attributes:
        query: User's question about the textbook (1-500 characters)
    """
    query: str = Field(
        ...,
        min_length=1,
        max_length=500,
        description="User question about the textbook"
    )

    @field_validator('query')
    @classmethod
    def query_not_empty(cls, v: str) -> str:
        """
        Validate that query is not empty or whitespace-only.

        Args:
            v: The query string to validate

        Returns:
            Stripped query string

        Raises:
            ValueError: If query is empty or whitespace-only
        """
        if not v.strip():
            raise ValueError("Question cannot be empty or whitespace-only")
        return v.strip()


class ChatResponse(BaseModel):
    """
    Response model for POST /chat endpoint.

    Attributes:
        answer: AI-generated answer to the user's question
        sources: List of source URLs from the textbook used to answer
        response_time: Total response time in seconds
        request_id: Unique request identifier for tracing
    """
    answer: str = Field(
        ...,
        description="AI-generated answer from textbook content"
    )
    sources: List[str] = Field(
        ...,
        description="Source URLs from textbook"
    )
    response_time: float = Field(
        ...,
        ge=0,
        description="Response time in seconds"
    )
    request_id: str = Field(
        ...,
        description="Request trace ID"
    )


@dataclass
class SearchResult:
    """
    Internal dataclass representing a single search result.

    Used during retrieval operations to pair chunks with similarity scores.

    Attributes:
        chunk: The retrieved text chunk
        score: Similarity score from vector search (0-1, higher is better)
    """
    chunk: TextChunk
    score: float


@dataclass
class RetrievalResult:
    """
    Internal dataclass for retrieval pipeline results.

    Used to pass retrieved chunks and metadata between service layers.

    Attributes:
        chunks: List of retrieved text chunks
        sources: List of unique source URLs from retrieved chunks
        scores: List of similarity scores corresponding to each chunk
    """
    chunks: List[TextChunk]
    sources: List[str]
    scores: List[float]
