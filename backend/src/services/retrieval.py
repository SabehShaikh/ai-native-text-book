"""
Retrieval service for semantic search of textbook content.

Combines embedding generation and vector search to retrieve relevant
text chunks for user queries.
"""

from typing import List
from uuid import UUID

from src.config import settings
from src.logger import logger
from src.models.chat import RetrievalResult, SearchResult
from src.models.ingest import TextChunk
from src.services.embedding import EmbeddingService
from src.services.vector_store import VectorStoreService


class RetrievalService:
    """
    Service for retrieving relevant textbook chunks for user queries.

    Orchestrates embedding generation and vector search to find the most
    relevant content from the textbook.

    Attributes:
        embedding_service: Service for generating query embeddings
        vector_store_service: Service for vector similarity search
    """

    def __init__(
        self,
        embedding_service: EmbeddingService,
        vector_store_service: VectorStoreService
    ):
        """
        Initialize the retrieval service.

        Args:
            embedding_service: Embedding service instance
            vector_store_service: Vector store service instance
        """
        self.embedding_service = embedding_service
        self.vector_store_service = vector_store_service

    async def retrieve(
        self,
        query: str,
        top_k: int = None
    ) -> RetrievalResult:
        """
        Retrieve relevant textbook chunks for a user query.

        Process:
        1. Generate embedding for user query
        2. Search Qdrant for top-k similar chunks
        3. Extract unique source URLs
        4. Return results with chunks, sources, and scores

        Args:
            query: User's question
            top_k: Number of top results to retrieve (default: from config)

        Returns:
            RetrievalResult with chunks, sources, and scores

        Raises:
            CohereAPIError: If embedding generation fails
            QdrantError: If vector search fails
            QdrantTimeout: If search times out
        """
        if top_k is None:
            top_k = settings.TOP_K_RESULTS

        logger.info(
            "Starting retrieval",
            extra={
                "query_length": len(query),
                "top_k": top_k
            }
        )

        # Step 1: Embed user query
        query_vector = await self.embedding_service.embed_text(query)

        logger.info(
            "Query embedded",
            extra={
                "vector_dimension": len(query_vector)
            }
        )

        # Step 2: Search Qdrant for similar chunks
        search_results = await self.vector_store_service.search(
            query_vector=query_vector,
            top_k=top_k
        )

        # Step 3: Convert search results to TextChunk objects
        chunks: List[TextChunk] = []
        scores: List[float] = []

        for result in search_results:
            chunk = TextChunk(
                id=UUID(result["id"]),
                text=result["text"],
                url=result["url"],
                title=result["title"],
                chunk_index=result["chunk_index"],
                vector=query_vector  # Not used in response, placeholder
            )
            chunks.append(chunk)
            scores.append(result["score"])

        # Step 4: Extract unique source URLs (preserve order)
        seen_urls = set()
        sources: List[str] = []
        for chunk in chunks:
            if chunk.url not in seen_urls:
                sources.append(chunk.url)
                seen_urls.add(chunk.url)

        logger.info(
            "Retrieval completed",
            extra={
                "chunks_retrieved": len(chunks),
                "unique_sources": len(sources),
                "top_score": scores[0] if scores else 0.0
            }
        )

        return RetrievalResult(
            chunks=chunks,
            sources=sources,
            scores=scores
        )
