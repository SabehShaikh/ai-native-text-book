"""Text chunking utility for splitting documents into overlapping chunks.

This module provides functionality to split text into fixed-size chunks
with configurable overlap, respecting word boundaries where possible.
"""

from typing import List


def chunk_text(
    text: str,
    chunk_size: int = 1000,
    overlap: int = 200
) -> List[str]:
    """Split text into overlapping chunks respecting word boundaries.

    Splits the input text into chunks of approximately chunk_size characters
    with the specified overlap between consecutive chunks. Attempts to split
    at word boundaries to avoid breaking words in the middle.

    Args:
        text: The text to chunk
        chunk_size: Target size of each chunk in characters (default: 1000)
        overlap: Number of characters to overlap between chunks (default: 200)

    Returns:
        List of text chunks

    Raises:
        ValueError: If chunk_size <= 0 or overlap < 0 or overlap >= chunk_size

    Examples:
        >>> text = "This is a long document that needs to be split."
        >>> chunks = chunk_text(text, chunk_size=20, overlap=5)
        >>> len(chunks)
        3
    """
    # Validate inputs
    if chunk_size <= 0:
        raise ValueError("chunk_size must be greater than 0")
    if overlap < 0:
        raise ValueError("overlap must be non-negative")
    if overlap >= chunk_size:
        raise ValueError("overlap must be less than chunk_size")

    # Handle empty or very short text
    if not text or len(text) <= chunk_size:
        return [text] if text else []

    chunks: List[str] = []
    position = 0
    text_length = len(text)

    while position < text_length:
        # Calculate end position for this chunk
        end_position = min(position + chunk_size, text_length)

        # Extract the chunk
        chunk = text[position:end_position]

        # If this is not the last chunk and we're not at the end,
        # try to find the last space to avoid breaking words
        if end_position < text_length:
            # Look for the last space in the last 20% of the chunk
            # to avoid splitting words while not losing too much content
            search_start = max(0, len(chunk) - int(chunk_size * 0.2))
            last_space = chunk.rfind(' ', search_start)

            # Only split at space if we found one in the search region
            if last_space > 0:
                chunk = chunk[:last_space]
                # Adjust end_position to account for the shortened chunk
                end_position = position + last_space

        # Add the chunk if it's not empty
        if chunk.strip():
            chunks.append(chunk)

        # Move position forward by (chunk_size - overlap)
        # This creates the overlap between consecutive chunks
        position = end_position - overlap

        # Prevent infinite loop: ensure we always move forward
        if position <= chunks[-1].__len__() - len(chunk) if chunks else 0:
            position = end_position

    return chunks


def chunk_text_with_metadata(
    text: str,
    chunk_size: int = 1000,
    overlap: int = 200,
    url: str = "",
    title: str = ""
) -> List[dict]:
    """Split text into chunks with metadata.

    Convenience function that splits text and returns chunks with
    associated metadata like URL, title, and chunk index.

    Args:
        text: The text to chunk
        chunk_size: Target size of each chunk in characters (default: 1000)
        overlap: Number of characters to overlap between chunks (default: 200)
        url: Source URL for the text
        title: Title of the document

    Returns:
        List of dictionaries containing chunk text and metadata

    Examples:
        >>> text = "Document content here"
        >>> result = chunk_text_with_metadata(
        ...     text, url="https://example.com", title="Example"
        ... )
        >>> result[0]['chunk_index']
        0
    """
    chunks = chunk_text(text, chunk_size=chunk_size, overlap=overlap)

    return [
        {
            "text": chunk,
            "url": url,
            "title": title,
            "chunk_index": idx,
        }
        for idx, chunk in enumerate(chunks)
    ]
