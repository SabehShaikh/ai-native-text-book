"""Configuration management using Pydantic BaseSettings.

This module loads and validates all environment variables required by the application.
All configuration values are loaded from environment variables with proper type validation.
"""

from pydantic_settings import BaseSettings
from pydantic import Field


class Settings(BaseSettings):
    """Application settings loaded from environment variables.

    All values are validated at startup. Missing required values will cause
    the application to fail fast with clear error messages.
    """

    # API Keys (Required)
    GEMINI_API_KEY: str = Field(..., description="Gemini API key for LLM generation")
    COHERE_API_KEY: str = Field(..., description="Cohere API key for embeddings")
    QDRANT_API_KEY: str = Field(..., description="Qdrant API key for vector database")

    # Service URLs (Required)
    QDRANT_URL: str = Field(..., description="Qdrant Cloud instance URL")
    FRONTEND_URL: str = Field(..., description="Frontend URL for CORS configuration")

    # Sitemap Configuration
    SITEMAP_URL: str = Field(
        default="https://ai-native-text-book.vercel.app/sitemap.xml",
        description="Default sitemap URL for content ingestion"
    )

    # Collection Configuration
    QDRANT_COLLECTION: str = Field(
        default="ai-textbook",
        description="Qdrant collection name for storing chunks"
    )

    # Chunking Configuration
    CHUNK_SIZE: int = Field(
        default=1000,
        description="Size of text chunks in characters",
        ge=100,
        le=5000
    )
    CHUNK_OVERLAP: int = Field(
        default=200,
        description="Overlap between consecutive chunks in characters",
        ge=0,
        le=1000
    )

    # Performance Configuration
    MAX_QUERY_LENGTH: int = Field(
        default=500,
        description="Maximum allowed query length in characters",
        ge=1,
        le=2000
    )
    TOP_K_RESULTS: int = Field(
        default=5,
        description="Number of top results to retrieve from Qdrant",
        ge=1,
        le=20
    )
    EMBEDDING_BATCH_SIZE: int = Field(
        default=96,
        description="Batch size for Cohere embeddings (free tier limit)",
        ge=1,
        le=96
    )

    class Config:
        """Pydantic configuration."""
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = True
        extra = "ignore"


# Global settings instance
settings = Settings()
