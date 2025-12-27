"""FastAPI application entry point.

This module initializes the FastAPI application with CORS middleware,
request ID middleware, exception handlers, and route registration.
"""

import time
import uuid
from contextlib import asynccontextmanager
from datetime import datetime
from typing import AsyncGenerator

from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from src.config import settings
from src.logger import logger
from src.utils.exceptions import (
    RAGChatbotException,
    CohereAPIError,
    GeminiAPIError,
    QdrantError,
    QdrantTimeout,
    SitemapFetchError,
    ValidationError,
)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Application lifespan manager.

    Handles startup and shutdown events for the application.

    Args:
        app: FastAPI application instance

    Yields:
        None
    """
    # Startup
    logger.info(
        "Starting RAG Chatbot API",
        environment="production",
        frontend_url=settings.FRONTEND_URL,
        qdrant_collection=settings.QDRANT_COLLECTION
    )

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API")


# Initialize FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    description="Physical AI Textbook Question Answering System",
    lifespan=lifespan,
)


# CORS Middleware - Allow only the frontend URL
app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.FRONTEND_URL],
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)


@app.middleware("http")
async def add_request_id_middleware(request: Request, call_next):
    """Add unique request ID to each request for tracing.

    Generates a UUID for each request and adds it to request.state
    for use in logging and error responses.

    Args:
        request: The incoming request
        call_next: The next middleware/handler in the chain

    Returns:
        Response with request_id header
    """
    request_id = str(uuid.uuid4())
    request.state.request_id = request_id

    # Log request start
    start_time = time.time()
    logger.info(
        "Request started",
        request_id=request_id,
        endpoint=str(request.url.path),
        method=request.method,
    )

    # Process request
    response = await call_next(request)

    # Log request completion
    duration_ms = (time.time() - start_time) * 1000
    logger.info(
        "Request completed",
        request_id=request_id,
        endpoint=str(request.url.path),
        method=request.method,
        status_code=response.status_code,
        duration_ms=round(duration_ms, 2),
    )

    # Add request ID to response headers
    response.headers["X-Request-ID"] = request_id

    return response


@app.exception_handler(RAGChatbotException)
async def ragchatbot_exception_handler(
    request: Request, exc: RAGChatbotException
) -> JSONResponse:
    """Handle all custom RAGChatbot exceptions.

    Maps exception types to appropriate HTTP status codes and
    returns structured error responses.

    Args:
        request: The request that caused the exception
        exc: The exception instance

    Returns:
        JSON response with error details
    """
    request_id = getattr(request.state, "request_id", None)

    # Determine status code based on exception type
    status_code = status.HTTP_500_INTERNAL_SERVER_ERROR

    if isinstance(exc, ValidationError):
        status_code = status.HTTP_400_BAD_REQUEST
    elif isinstance(exc, (CohereAPIError, GeminiAPIError, QdrantError)):
        status_code = status.HTTP_503_SERVICE_UNAVAILABLE
    elif isinstance(exc, QdrantTimeout):
        status_code = status.HTTP_504_GATEWAY_TIMEOUT
    elif isinstance(exc, SitemapFetchError):
        status_code = status.HTTP_400_BAD_REQUEST

    # Log the error
    logger.error(
        f"Application error: {exc.message}",
        request_id=request_id,
        endpoint=str(request.url.path),
        error_type=exc.__class__.__name__,
        error_details=exc.details,
    )

    # Return structured error response
    return JSONResponse(
        status_code=status_code,
        content={
            "detail": exc.message,
            "error_type": exc.__class__.__name__,
            "request_id": request_id,
            "timestamp": datetime.utcnow().isoformat() + "Z",
        },
    )


@app.exception_handler(Exception)
async def general_exception_handler(
    request: Request, exc: Exception
) -> JSONResponse:
    """Handle unexpected exceptions.

    Catches all exceptions not handled by specific handlers and
    returns a generic error response without exposing internal details.

    Args:
        request: The request that caused the exception
        exc: The exception instance

    Returns:
        JSON response with generic error message
    """
    request_id = getattr(request.state, "request_id", None)

    # Log the unexpected error with full details
    logger.error(
        f"Unexpected error: {str(exc)}",
        request_id=request_id,
        endpoint=str(request.url.path),
        error_type=exc.__class__.__name__,
        exc_info=True,
    )

    # Return generic error response (don't expose internal details)
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={
            "detail": "An unexpected error occurred. Please try again later.",
            "request_id": request_id,
            "timestamp": datetime.utcnow().isoformat() + "Z",
        },
    )


# Root endpoint
@app.get("/")
async def root() -> dict:
    """Root endpoint returning API information.

    Returns:
        Dictionary with API name and version
    """
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "description": "Physical AI Textbook Question Answering System",
    }


# Register API routes
from src.api import chat, health, ingest

app.include_router(ingest.router, tags=["ingestion"])
app.include_router(chat.router, tags=["chat"])
app.include_router(health.router, tags=["health"])
