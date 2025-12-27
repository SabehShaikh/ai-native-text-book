"""
Chat API endpoint for question answering.

Provides POST /chat endpoint for students to ask questions about the textbook
and receive AI-generated answers with source citations.
"""

import time
from uuid import uuid4

from fastapi import APIRouter, HTTPException

from src.logger import logger
from src.models.chat import ChatRequest, ChatResponse
from src.services.agent import AgentService
from src.services.embedding import EmbeddingService
from src.services.retrieval import RetrievalService
from src.services.vector_store import VectorStoreService
from src.utils.exceptions import (
    CohereAPIError,
    GeminiAPIError,
    QdrantError,
    QdrantTimeout
)

router = APIRouter()


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Answer a user question about the textbook.

    Process:
    1. Generate request ID for tracing
    2. Validate input (handled by Pydantic)
    3. Retrieve relevant textbook chunks
    4. Generate AI answer with sources
    5. Return response with timing info

    Args:
        request: ChatRequest with user query

    Returns:
        ChatResponse with answer, sources, timing, and request ID

    Raises:
        HTTPException 400: Invalid input (empty/too long query)
        HTTPException 503: AI service unavailable (Cohere/Gemini/Qdrant failure)
        HTTPException 504: Search timeout (Qdrant >5s)
        HTTPException 500: Unexpected error
    """
    request_id = str(uuid4())
    start_time = time.time()

    try:
        logger.info(
            "Chat request received",
            extra={
                "request_id": request_id,
                "query_length": len(request.query),
                "endpoint": "/chat"
            }
        )

        # Initialize services
        embedding_service = EmbeddingService()
        vector_store_service = VectorStoreService()
        retrieval_service = RetrievalService(
            embedding_service=embedding_service,
            vector_store_service=vector_store_service
        )
        agent_service = AgentService(retrieval_service=retrieval_service)

        # Generate answer with agent
        answer, sources = await agent_service.answer_question(request.query)

        # Calculate response time
        response_time = time.time() - start_time

        logger.info(
            "Chat request completed",
            extra={
                "request_id": request_id,
                "response_time": response_time,
                "duration_ms": response_time * 1000,
                "sources_count": len(sources),
                "answer_length": len(answer),
                "endpoint": "/chat"
            }
        )

        return ChatResponse(
            answer=answer,
            sources=sources,
            response_time=response_time,
            request_id=request_id
        )

    except CohereAPIError as e:
        logger.error(
            "Cohere API error in chat",
            extra={
                "request_id": request_id,
                "error": str(e),
                "endpoint": "/chat"
            }
        )
        raise HTTPException(
            status_code=503,
            detail="AI service temporarily unavailable (embedding service)"
        )

    except GeminiAPIError as e:
        logger.error(
            "Gemini API error in chat",
            extra={
                "request_id": request_id,
                "error": str(e),
                "endpoint": "/chat"
            }
        )
        raise HTTPException(
            status_code=503,
            detail="AI service temporarily unavailable (answer generation)"
        )

    except QdrantTimeout as e:
        logger.error(
            "Qdrant timeout in chat",
            extra={
                "request_id": request_id,
                "error": str(e),
                "endpoint": "/chat"
            }
        )
        raise HTTPException(
            status_code=504,
            detail="Search service timeout - please try again"
        )

    except QdrantError as e:
        logger.error(
            "Qdrant error in chat",
            extra={
                "request_id": request_id,
                "error": str(e),
                "endpoint": "/chat"
            }
        )
        raise HTTPException(
            status_code=503,
            detail="Search service temporarily unavailable"
        )

    except Exception as e:
        logger.error(
            "Unexpected error in chat",
            extra={
                "request_id": request_id,
                "error": str(e),
                "error_type": type(e).__name__,
                "endpoint": "/chat"
            }
        )
        raise HTTPException(
            status_code=500,
            detail="Internal server error"
        )
