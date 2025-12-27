"""
Ingestion service for processing textbook content.

Handles fetching sitemap, extracting text, chunking, embedding,
and storing in vector database.
"""

import time
import xml.etree.ElementTree as ET
from typing import List
from uuid import uuid4

import httpx
import trafilatura

from src.config import settings
from src.logger import logger
from src.models.ingest import IngestionResult, TextChunk
from src.services.embedding import EmbeddingService
from src.services.vector_store import VectorStoreService
from src.utils.chunking import chunk_text
from src.utils.exceptions import SitemapFetchError


class IngestionService:
    """
    Service for ingesting textbook content from sitemap.

    Orchestrates the complete ingestion pipeline:
    1. Fetch and parse sitemap.xml
    2. Extract text from each page
    3. Chunk text with overlap
    4. Generate embeddings in batches
    5. Upsert to vector database

    Attributes:
        embedding_service: Service for generating embeddings
        vector_store: Service for vector database operations
    """

    def __init__(self):
        """Initialize ingestion service with dependencies."""
        self.embedding_service = EmbeddingService()
        self.vector_store = VectorStoreService()

    async def ingest_from_sitemap(self, sitemap_url: str) -> IngestionResult:
        """
        Ingest textbook content from sitemap URL.

        Complete pipeline:
        1. Fetch sitemap.xml
        2. Parse URLs from sitemap
        3. For each URL:
           - Fetch HTML
           - Extract text with trafilatura
           - Chunk text (1000 chars, 200 overlap)
        4. Batch embed chunks (96 per batch)
        5. Upsert to Qdrant
        6. Log progress every 10 pages

        Args:
            sitemap_url: URL of the sitemap.xml file

        Returns:
            IngestionResult with metrics and errors

        Raises:
            SitemapFetchError: If sitemap cannot be fetched or parsed
        """
        start_time = time.time()
        errors = []
        pages_processed = 0
        chunks_created = 0

        try:
            # Ensure collection exists
            await self.vector_store.ensure_collection_exists()

            # Fetch and parse sitemap
            logger.info(
                "Fetching sitemap",
                extra={"sitemap_url": sitemap_url}
            )
            urls = await self._fetch_sitemap_urls(sitemap_url)

            logger.info(
                f"Found {len(urls)} URLs in sitemap",
                extra={"url_count": len(urls)}
            )

            # Process pages ONE AT A TIME to avoid memory issues
            async with httpx.AsyncClient(timeout=30.0) as client:
                for i, url in enumerate(urls, 1):
                    try:
                        logger.info(
                            f"Processing page {i}/{len(urls)}: {url}",
                            extra={
                                "page_num": i,
                                "total_pages": len(urls),
                                "url": url
                            }
                        )

                        # 1. Extract and chunk this page
                        page_chunks = await self._process_page(client, url)

                        if page_chunks:
                            # 2. Embed and upload THIS PAGE's chunks immediately
                            await self._embed_and_upsert_chunks(page_chunks)

                            # 3. Update counters
                            pages_processed += 1
                            chunks_created += len(page_chunks)

                            logger.info(
                                f"Page {i}/{len(urls)} completed: "
                                f"{len(page_chunks)} chunks uploaded",
                                extra={
                                    "page_num": i,
                                    "page_chunks": len(page_chunks),
                                    "total_chunks": chunks_created,
                                    "total_pages_processed": pages_processed
                                }
                            )

                            # 4. Clear memory (Python will GC the chunks)
                            del page_chunks

                            # Log progress every 10 pages
                            if i % 10 == 0:
                                logger.info(
                                    f"Progress: {i}/{len(urls)} pages",
                                    extra={
                                        "pages_processed": pages_processed,
                                        "chunks_created": chunks_created,
                                        "urls_remaining": len(urls) - i
                                    }
                                )

                    except Exception as e:
                        logger.error(
                            f"Failed to process page: {url}",
                            extra={
                                "url": url,
                                "error": str(e),
                                "error_type": type(e).__name__
                            }
                        )
                        errors.append(url)

            duration = time.time() - start_time

            logger.info(
                "Ingestion completed",
                extra={
                    "pages_processed": pages_processed,
                    "chunks_created": chunks_created,
                    "duration_seconds": duration,
                    "errors_count": len(errors)
                }
            )

            return IngestionResult(
                pages_processed=pages_processed,
                chunks_created=chunks_created,
                duration_seconds=duration,
                errors=errors
            )

        except SitemapFetchError:
            raise
        except Exception as e:
            logger.error(
                "Ingestion failed",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__,
                    "pages_processed": pages_processed
                }
            )
            duration = time.time() - start_time
            return IngestionResult(
                pages_processed=pages_processed,
                chunks_created=chunks_created,
                duration_seconds=duration,
                errors=errors + ["Fatal error: " + str(e)]
            )

    async def _fetch_sitemap_urls(self, sitemap_url: str) -> List[str]:
        """
        Fetch and parse URLs from sitemap.xml.

        Args:
            sitemap_url: URL of the sitemap.xml file

        Returns:
            List of URLs extracted from sitemap

        Raises:
            SitemapFetchError: If sitemap cannot be fetched or parsed
        """
        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.get(sitemap_url)
                response.raise_for_status()

                # Parse XML
                root = ET.fromstring(response.content)

                # Extract URLs from sitemap
                # Handle both sitemap formats: with and without namespace
                namespace = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}

                # Try with namespace first
                urls = [
                    loc.text for loc in root.findall(".//ns:url/ns:loc", namespace)
                    if loc.text
                ]

                # If no URLs found, try without namespace
                if not urls:
                    urls = [
                        loc.text for loc in root.findall(".//url/loc")
                        if loc.text
                    ]

                if not urls:
                    raise SitemapFetchError("No URLs found in sitemap")

                return urls

        except httpx.HTTPStatusError as e:
            logger.error(
                "HTTP error fetching sitemap",
                extra={
                    "sitemap_url": sitemap_url,
                    "status_code": e.response.status_code,
                    "error": str(e)
                }
            )
            raise SitemapFetchError(
                f"Failed to fetch sitemap (HTTP {e.response.status_code})"
            )
        except httpx.RequestError as e:
            logger.error(
                "Network error fetching sitemap",
                extra={
                    "sitemap_url": sitemap_url,
                    "error": str(e)
                }
            )
            raise SitemapFetchError(f"Network error fetching sitemap: {str(e)}")
        except ET.ParseError as e:
            logger.error(
                "XML parse error",
                extra={
                    "sitemap_url": sitemap_url,
                    "error": str(e)
                }
            )
            raise SitemapFetchError(f"Invalid sitemap XML: {str(e)}")
        except Exception as e:
            logger.error(
                "Unexpected error fetching sitemap",
                extra={
                    "sitemap_url": sitemap_url,
                    "error": str(e),
                    "error_type": type(e).__name__
                }
            )
            raise SitemapFetchError(f"Failed to fetch sitemap: {str(e)}")

    async def _process_page(
        self,
        client: httpx.AsyncClient,
        url: str
    ) -> List[TextChunk]:
        """
        Process a single page: fetch, extract, chunk.

        Args:
            client: HTTP client for fetching
            url: URL of the page to process

        Returns:
            List of TextChunk objects (without embeddings yet)

        Raises:
            Exception: If page processing fails
        """
        # Fetch page HTML
        response = await client.get(url)
        response.raise_for_status()

        # Extract text with Trafilatura
        text = trafilatura.extract(
            response.text,
            include_comments=False,
            include_tables=True
        )

        if not text or len(text.strip()) < 100:
            logger.warning(
                f"No significant text extracted from {url}",
                extra={"url": url, "text_length": len(text) if text else 0}
            )
            return []

        # Extract title (try from HTML first, fallback to URL)
        title = self._extract_title(response.text) or url.split("/")[-1]

        # Chunk text
        text_chunks = chunk_text(
            text,
            chunk_size=settings.CHUNK_SIZE,
            overlap=settings.CHUNK_OVERLAP
        )

        logger.info(
            f"Processed page: {url}",
            extra={
                "url": url,
                "text_length": len(text),
                "chunks_count": len(text_chunks)
            }
        )

        # Create TextChunk objects (without embeddings)
        chunks = [
            TextChunk(
                id=uuid4(),
                text=chunk,
                url=url,
                title=title,
                chunk_index=i,
                vector=[]  # Will be filled during embedding
            )
            for i, chunk in enumerate(text_chunks)
        ]

        return chunks

    def _extract_title(self, html: str) -> str:
        """
        Extract title from HTML.

        Args:
            html: HTML content

        Returns:
            Title text or empty string if not found
        """
        try:
            # Simple title extraction
            if "<title>" in html and "</title>" in html:
                start = html.find("<title>") + 7
                end = html.find("</title>")
                return html[start:end].strip()
            return ""
        except Exception:
            return ""

    async def _embed_and_upsert_chunks(
        self,
        chunks: List[TextChunk]
    ) -> None:
        """
        Embed chunks in batches and upsert to vector store.

        Args:
            chunks: List of TextChunk objects without embeddings

        Raises:
            CohereAPIError: If embedding fails
            QdrantError: If upsert fails
        """
        # Extract texts for embedding
        texts = [chunk.text for chunk in chunks]

        # Batch embed with error handling
        logger.info(
            f"Embedding {len(texts)} chunks",
            extra={"chunk_count": len(texts)}
        )

        try:
            embeddings = await self.embedding_service.embed_batch(texts)

            # Validate embeddings
            if not embeddings:
                raise ValueError("Embedding service returned empty list")

            if len(embeddings) != len(texts):
                raise ValueError(
                    f"Embedding count mismatch: got {len(embeddings)}, "
                    f"expected {len(texts)}"
                )

            logger.info(
                "Embeddings generated successfully",
                extra={
                    "embeddings_count": len(embeddings),
                    "vector_dimension": len(embeddings[0]) if embeddings else 0
                }
            )

        except Exception as e:
            logger.error(
                "Failed to generate embeddings",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__,
                    "chunk_count": len(texts)
                }
            )
            raise

        # Assign embeddings to chunks
        for chunk, embedding in zip(chunks, embeddings):
            chunk.vector = embedding

        # Upsert to vector store in batches with error handling
        batch_size = settings.EMBEDDING_BATCH_SIZE
        total_upserted = 0

        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            batch_num = i // batch_size + 1

            try:
                upserted_count = await self.vector_store.upsert_chunks(batch)
                total_upserted += upserted_count

                logger.info(
                    f"Upserted batch {batch_num}",
                    extra={
                        "batch_num": batch_num,
                        "batch_start": i,
                        "batch_size": len(batch),
                        "upserted_count": upserted_count,
                        "total_upserted": total_upserted,
                        "total_chunks": len(chunks)
                    }
                )

            except Exception as e:
                logger.error(
                    f"Failed to upsert batch {batch_num}",
                    extra={
                        "batch_num": batch_num,
                        "batch_start": i,
                        "batch_size": len(batch),
                        "error": str(e),
                        "error_type": type(e).__name__
                    }
                )
                raise

        # Final validation
        logger.info(
            "All chunks upserted successfully",
            extra={
                "total_chunks": len(chunks),
                "total_upserted": total_upserted
            }
        )
