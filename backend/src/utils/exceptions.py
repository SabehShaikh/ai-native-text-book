"""Custom exception classes for RAG Chatbot API.

This module defines all custom exceptions used throughout the application.
Each exception represents a specific error condition with appropriate context.
"""


class RAGChatbotException(Exception):
    """Base exception for all RAG Chatbot errors.

    All custom exceptions in this application inherit from this base class.
    This allows for catching all application-specific errors in one place.
    """

    def __init__(self, message: str, details: dict = None):
        """Initialize the exception.

        Args:
            message: Human-readable error message
            details: Optional dictionary with additional error context
        """
        super().__init__(message)
        self.message = message
        self.details = details or {}


class CohereAPIError(RAGChatbotException):
    """Raised when Cohere API calls fail.

    This includes rate limits, authentication errors, network failures,
    and any other errors from the Cohere embedding service.
    """

    pass


class GeminiAPIError(RAGChatbotException):
    """Raised when Gemini API calls fail.

    This includes rate limits, authentication errors, network failures,
    and any other errors from the Gemini generation service.
    """

    pass


class QdrantError(RAGChatbotException):
    """Raised when Qdrant operations fail.

    This includes connection errors, query failures, upsert failures,
    and any other errors from the Qdrant vector database.
    """

    pass


class QdrantTimeout(QdrantError):
    """Raised when Qdrant query exceeds timeout threshold.

    Specifically for operations that take longer than the configured
    timeout (default 5 seconds). Inherits from QdrantError for
    hierarchical exception handling.
    """

    pass


class SitemapFetchError(RAGChatbotException):
    """Raised when sitemap fetching or parsing fails.

    This includes network errors when fetching sitemap.xml,
    XML parsing errors, and missing or invalid sitemap structure.
    """

    pass


class ValidationError(RAGChatbotException):
    """Raised when input validation fails.

    This includes empty queries, queries that are too long,
    invalid URLs, and any other input validation failures.
    """

    pass
