"""
Embedding service using Cohere API.

Provides text embedding functionality using Cohere's embed-english-v3.0 model.
Supports both single-text and batch embedding operations with retry logic.
"""

import asyncio
from typing import List

import cohere

from src.config import settings
from src.logger import logger
from src.utils.exceptions import CohereAPIError


class EmbeddingService:
    """
    Service for generating text embeddings using Cohere API.

    Uses the embed-english-v3.0 model to generate 1024-dimensional vectors.
    Includes exponential backoff retry logic for handling rate limits.

    Attributes:
        client: Async Cohere client instance
        model: Cohere embedding model name
        max_retries: Maximum number of retry attempts
    """

    def __init__(self):
        """Initialize the Cohere client."""
        self.client = cohere.AsyncClient(api_key=settings.COHERE_API_KEY)
        self.model = "embed-english-v3.0"
        self.max_retries = 3

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: The text to embed

        Returns:
            1024-dimensional embedding vector

        Raises:
            CohereAPIError: If embedding generation fails after retries
        """
        for attempt in range(self.max_retries):
            try:
                start_time = asyncio.get_event_loop().time()

                response = await self.client.embed(
                    texts=[text],
                    model=self.model,
                    input_type="search_document"
                )

                duration = asyncio.get_event_loop().time() - start_time

                logger.info(
                    "Generated single embedding",
                    extra={
                        "duration_ms": duration * 1000,
                        "text_length": len(text),
                        "attempt": attempt + 1
                    }
                )

                # Validate output dimensions
                vector = response.embeddings[0]
                if len(vector) != 1024:
                    raise CohereAPIError(
                        f"Invalid embedding dimension: {len(vector)}, expected 1024"
                    )

                return vector

            except cohere.errors.TooManyRequestsError as e:
                wait_time = 2 ** attempt  # Exponential backoff: 1s, 2s, 4s
                logger.warning(
                    f"Cohere rate limit hit, retrying in {wait_time}s",
                    extra={
                        "attempt": attempt + 1,
                        "max_retries": self.max_retries,
                        "error": str(e)
                    }
                )
                if attempt < self.max_retries - 1:
                    await asyncio.sleep(wait_time)
                else:
                    raise CohereAPIError(
                        f"Embedding failed after {self.max_retries} attempts: {str(e)}"
                    )

            except Exception as e:
                logger.error(
                    "Cohere API error",
                    extra={
                        "attempt": attempt + 1,
                        "error": str(e),
                        "error_type": type(e).__name__
                    }
                )
                raise CohereAPIError(f"Embedding generation failed: {str(e)}")

    async def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batch.

        Uses batch processing for efficiency. Automatically respects
        Cohere free tier batch size limit (96 texts per request).

        Args:
            texts: List of texts to embed

        Returns:
            List of 1024-dimensional embedding vectors

        Raises:
            CohereAPIError: If embedding generation fails after retries
        """
        if not texts:
            return []

        batch_size = settings.EMBEDDING_BATCH_SIZE

        # Process in batches if needed
        if len(texts) > batch_size:
            logger.info(
                f"Processing {len(texts)} texts in batches of {batch_size}",
                extra={"total_texts": len(texts), "batch_size": batch_size}
            )

            all_embeddings = []
            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                batch_embeddings = await self._embed_batch_internal(batch)
                all_embeddings.extend(batch_embeddings)

            return all_embeddings
        else:
            return await self._embed_batch_internal(texts)

    async def _embed_batch_internal(
        self,
        texts: List[str]
    ) -> List[List[float]]:
        """
        Internal method to embed a batch within size limits.

        Args:
            texts: List of texts to embed (must be <= batch_size)

        Returns:
            List of embedding vectors

        Raises:
            CohereAPIError: If embedding generation fails
        """
        for attempt in range(self.max_retries):
            try:
                start_time = asyncio.get_event_loop().time()

                response = await self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type="search_document"
                )

                duration = asyncio.get_event_loop().time() - start_time

                logger.info(
                    "Generated batch embeddings",
                    extra={
                        "duration_ms": duration * 1000,
                        "batch_size": len(texts),
                        "attempt": attempt + 1
                    }
                )

                # Validate output dimensions
                vectors = response.embeddings
                for i, vector in enumerate(vectors):
                    if len(vector) != 1024:
                        raise CohereAPIError(
                            f"Invalid embedding dimension at index {i}: "
                            f"{len(vector)}, expected 1024"
                        )

                return vectors

            except cohere.errors.TooManyRequestsError as e:
                wait_time = 2 ** attempt  # Exponential backoff
                logger.warning(
                    f"Cohere rate limit hit, retrying in {wait_time}s",
                    extra={
                        "attempt": attempt + 1,
                        "max_retries": self.max_retries,
                        "batch_size": len(texts),
                        "error": str(e)
                    }
                )
                if attempt < self.max_retries - 1:
                    await asyncio.sleep(wait_time)
                else:
                    raise CohereAPIError(
                        f"Batch embedding failed after {self.max_retries} "
                        f"attempts: {str(e)}"
                    )

            except Exception as e:
                logger.error(
                    "Cohere batch API error",
                    extra={
                        "attempt": attempt + 1,
                        "batch_size": len(texts),
                        "error": str(e),
                        "error_type": type(e).__name__
                    }
                )
                raise CohereAPIError(
                    f"Batch embedding generation failed: {str(e)}"
                )
