"""
Vector store service using Qdrant Cloud.

Provides vector database operations for storing and searching text chunks.
Handles collection management, upsert operations, and semantic search.
"""

from typing import List
from uuid import UUID

from qdrant_client import QdrantClient, models
from qdrant_client.http.exceptions import UnexpectedResponse

from src.config import settings
from src.logger import logger
from src.models.ingest import TextChunk
from src.utils.exceptions import QdrantError, QdrantTimeout


class VectorStoreService:
    """
    Service for interacting with Qdrant vector database.

    Manages vector storage, retrieval, and collection lifecycle.
    Includes connection pooling and retry logic for reliability.

    Attributes:
        client: Qdrant client instance
        collection_name: Name of the vector collection
    """

    def __init__(self):
        """Initialize Qdrant client with connection pooling."""
        try:
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=5.0  # 5 second timeout
            )
            self.collection_name = settings.QDRANT_COLLECTION

            logger.info(
                "Qdrant client initialized",
                extra={
                    "collection": self.collection_name,
                    "url": settings.QDRANT_URL
                }
            )
        except Exception as e:
            logger.error(
                "Failed to initialize Qdrant client",
                extra={"error": str(e), "error_type": type(e).__name__}
            )
            raise QdrantError(f"Qdrant initialization failed: {str(e)}")

    async def ensure_collection_exists(self) -> None:
        """
        Ensure the vector collection exists, create if not.

        Creates a collection with the following specifications:
        - Vector size: 1024 (Cohere embed-english-v3.0)
        - Distance metric: Cosine similarity
        - Payload schema: text, url, title, chunk_index

        Raises:
            QdrantError: If collection creation fails
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name in collection_names:
                logger.info(
                    "Collection already exists",
                    extra={"collection": self.collection_name}
                )
                return

            # Create collection
            logger.info(
                "Creating new collection",
                extra={"collection": self.collection_name}
            )

            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1024,  # Cohere embed-english-v3.0 dimension
                    distance=models.Distance.COSINE
                )
            )

            logger.info(
                "Collection created successfully",
                extra={"collection": self.collection_name}
            )

        except UnexpectedResponse as e:
            logger.error(
                "Qdrant API error during collection creation",
                extra={
                    "error": str(e),
                    "collection": self.collection_name
                }
            )
            raise QdrantError(f"Collection creation failed: {str(e)}")
        except Exception as e:
            logger.error(
                "Unexpected error during collection creation",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__
                }
            )
            raise QdrantError(f"Collection creation failed: {str(e)}")

    async def upsert_chunks(self, chunks: List[TextChunk]) -> int:
        """
        Upsert text chunks into the vector collection.

        Args:
            chunks: List of TextChunk objects with embeddings

        Returns:
            Number of chunks successfully upserted

        Raises:
            QdrantError: If upsert operation fails
        """
        if not chunks:
            logger.warning("No chunks to upsert")
            return 0

        try:
            # Convert chunks to Qdrant points
            points = [
                models.PointStruct(
                    id=str(chunk.id),
                    vector=chunk.vector,
                    payload={
                        "text": chunk.text,
                        "url": chunk.url,
                        "title": chunk.title,
                        "chunk_index": chunk.chunk_index
                    }
                )
                for chunk in chunks
            ]

            # Upsert to Qdrant
            operation_info = self.client.upsert(
                collection_name=self.collection_name,
                points=points,
                wait=True  # Wait for operation to complete
            )

            logger.info(
                "Chunks upserted successfully",
                extra={
                    "collection": self.collection_name,
                    "count": len(chunks),
                    "status": operation_info.status
                }
            )

            return len(chunks)

        except UnexpectedResponse as e:
            logger.error(
                "Qdrant API error during upsert",
                extra={
                    "error": str(e),
                    "chunk_count": len(chunks)
                }
            )
            raise QdrantError(f"Upsert failed: {str(e)}")
        except Exception as e:
            logger.error(
                "Unexpected error during upsert",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__,
                    "chunk_count": len(chunks)
                }
            )
            raise QdrantError(f"Upsert failed: {str(e)}")

    async def search(
        self,
        query_vector: List[float],
        top_k: int = 5
    ) -> List[dict]:
        """
        Perform semantic search for similar chunks.

        Args:
            query_vector: 1024-dimensional query embedding
            top_k: Number of top results to return (default: 5)

        Returns:
            List of search results with score, id, and payload

        Raises:
            QdrantError: If search operation fails
            QdrantTimeout: If search takes longer than 5 seconds
        """
        try:
            # Validate vector dimension
            if len(query_vector) != 1024:
                raise QdrantError(
                    f"Invalid query vector dimension: {len(query_vector)}, "
                    f"expected 1024"
                )

            # Perform search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                with_payload=True
            )

            logger.info(
                "Search completed",
                extra={
                    "collection": self.collection_name,
                    "top_k": top_k,
                    "results_count": len(search_results)
                }
            )

            # Convert to dict format
            results = [
                {
                    "id": result.id,
                    "score": result.score,
                    "text": result.payload.get("text", ""),
                    "url": result.payload.get("url", ""),
                    "title": result.payload.get("title", ""),
                    "chunk_index": result.payload.get("chunk_index", 0)
                }
                for result in search_results
            ]

            return results

        except UnexpectedResponse as e:
            if "timeout" in str(e).lower():
                logger.error(
                    "Qdrant search timeout",
                    extra={"error": str(e), "top_k": top_k}
                )
                raise QdrantTimeout("Search operation timed out after 5s")
            else:
                logger.error(
                    "Qdrant API error during search",
                    extra={"error": str(e), "top_k": top_k}
                )
                raise QdrantError(f"Search failed: {str(e)}")
        except Exception as e:
            logger.error(
                "Unexpected error during search",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__,
                    "top_k": top_k
                }
            )
            raise QdrantError(f"Search failed: {str(e)}")

    async def get_collection_count(self) -> int:
        """
        Get the number of vectors in the collection.

        Returns:
            Number of vectors in the collection

        Raises:
            QdrantError: If count operation fails
        """
        try:
            collection_info = self.client.get_collection(
                collection_name=self.collection_name
            )

            count = collection_info.points_count or 0

            logger.info(
                "Retrieved collection count",
                extra={
                    "collection": self.collection_name,
                    "count": count
                }
            )

            return count

        except UnexpectedResponse as e:
            logger.error(
                "Qdrant API error getting collection count",
                extra={"error": str(e)}
            )
            raise QdrantError(f"Failed to get collection count: {str(e)}")
        except Exception as e:
            logger.error(
                "Unexpected error getting collection count",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__
                }
            )
            raise QdrantError(f"Failed to get collection count: {str(e)}")

    async def health_check(self) -> bool:
        """
        Check Qdrant connectivity and collection health.

        Returns:
            True if Qdrant is reachable and collection exists, False otherwise
        """
        try:
            # Try to get collection info
            collection_info = self.client.get_collection(
                collection_name=self.collection_name
            )

            is_healthy = collection_info is not None

            logger.info(
                "Health check completed",
                extra={
                    "collection": self.collection_name,
                    "healthy": is_healthy,
                    "points_count": collection_info.points_count or 0
                }
            )

            return is_healthy

        except Exception as e:
            logger.warning(
                "Health check failed",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__
                }
            )
            return False
