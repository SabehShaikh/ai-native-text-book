# RAG Chatbot API - Test Suite

## Overview

This test suite provides comprehensive coverage for Phase 3 (User Story 2 - Ingest) of the RAG Chatbot API. The tests validate all ingestion pipeline components including chunking, embedding, vector storage, and the complete ingestion workflow.

## Test Statistics

- **Total Test Files**: 5
- **Total Test Cases**: 108
- **Total Lines of Test Code**: ~1,810
- **Coverage Target**: >80%

## Test Structure

```
tests/
├── conftest.py                           # Shared fixtures for all tests
├── unit/                                 # Unit tests for individual components
│   ├── test_chunking.py                 # Text chunking utility tests (32 tests)
│   ├── test_embedding.py                # EmbeddingService tests (23 tests)
│   ├── test_vector_store.py             # VectorStoreService tests (26 tests)
│   └── test_ingestion.py                # IngestionService tests (16 tests)
└── integration/                          # Integration tests for API endpoints
    └── test_ingest.py                   # POST /ingest endpoint tests (11 tests)
```

## Test Coverage by Component

### T022: Test Fixtures (conftest.py)
**File**: `tests/conftest.py`

Provides reusable fixtures for mocking external services:
- `mock_cohere_client` - Mock Cohere API for embedding tests
- `mock_cohere_client_with_rate_limit` - Simulates rate limiting scenarios
- `mock_qdrant_client` - Mock Qdrant vector database
- `mock_qdrant_client_unhealthy` - Simulates Qdrant connection failures
- `mock_httpx_client` - Mock HTTP client for sitemap/page fetching
- `mock_httpx_client_with_errors` - Simulates HTTP errors
- `mock_httpx_client_malformed_sitemap` - Returns malformed XML
- Sample data fixtures for testing

### T023: Chunking Utility Tests (test_chunking.py)
**File**: `tests/unit/test_chunking.py`
**Test Cases**: 32

Tests for `src/utils/chunking.py`:
- Empty text handling
- Text shorter than chunk size
- Text exactly equal to chunk size
- Long text requiring multiple chunks
- Overlap logic between chunks
- Word boundary respect
- Custom chunk sizes and overlaps
- Zero overlap scenarios
- Small chunk sizes
- Whitespace handling
- Newline preservation
- Input validation (invalid parameters)
- Unicode and special characters
- Metadata attachment in `chunk_text_with_metadata()`

**Key Test Scenarios**:
- Edge case: Empty string returns empty list
- Edge case: Text < chunk_size returns single chunk
- Validation: Negative chunk_size raises ValueError
- Validation: overlap >= chunk_size raises ValueError
- Functional: Chunks don't break words mid-word
- Functional: Overlap creates proper chunk continuity

### T024: EmbeddingService Tests (test_embedding.py)
**File**: `tests/unit/test_embedding.py`
**Test Cases**: 23

Tests for `src/services/embedding.py`:
- Single text embedding (`embed_text()`)
- Batch text embedding (`embed_batch()`)
- Empty string embedding
- Long text embedding
- Special characters and unicode
- Rate limit retry logic (exponential backoff)
- Max retries exhaustion
- API error handling
- Invalid embedding dimension detection
- Batch size enforcement (96 limit)
- Multiple batch processing for large inputs

**Key Test Scenarios**:
- Retry: First call fails with rate limit, second succeeds
- Retry: All retries exhausted raises CohereAPIError
- Validation: Wrong dimension (512 vs 1024) raises error
- Batch: 150 texts split into multiple batches of ≤96
- Error: Generic API errors properly wrapped

### T025: VectorStoreService Tests (test_vector_store.py)
**File**: `tests/unit/test_vector_store.py`
**Test Cases**: 26

Tests for `src/services/vector_store.py`:
- Collection existence checking (`ensure_collection_exists()`)
- Collection creation with correct parameters
- Chunk upserting (`upsert_chunks()`)
- Vector search (`search()`)
- Collection count retrieval (`get_collection_count()`)
- Health check (`health_check()`)
- Empty list handling
- Payload structure verification
- Search timeouts (QdrantTimeout)
- API error handling
- Invalid vector dimension validation

**Key Test Scenarios**:
- Collection: Creates with size=1024, distance=COSINE
- Upsert: Empty list returns 0, no API call
- Search: Invalid dimension (512) raises QdrantError
- Search: Timeout raises QdrantTimeout with proper message
- Health: Returns False on any exception
- Payload: Includes text, url, title, chunk_index

### T026: IngestionService Tests (test_ingestion.py)
**File**: `tests/unit/test_ingestion.py`
**Test Cases**: 16

Tests for `src/services/ingestion.py`:
- Sitemap URL fetching and parsing (`_fetch_sitemap_urls()`)
- Sitemap with/without XML namespace
- HTTP errors (404, 500, network errors)
- Malformed XML handling
- Empty sitemap handling
- Page processing (`_process_page()`)
- Text extraction with Trafilatura
- Title extraction from HTML
- Chunk embedding and upserting (`_embed_and_upsert_chunks()`)
- Large batch processing (200+ chunks)
- End-to-end ingestion (`ingest_from_sitemap()`)
- Partial failure scenarios
- Error recovery and logging

**Key Test Scenarios**:
- Sitemap: Parses both namespaced and non-namespaced XML
- Error: HTTP 404 raises SitemapFetchError
- Page: Text < 100 chars returns empty list
- Batch: 200 chunks processed in batches of 96
- Ingestion: Partial failures collected in errors list
- Logging: Progress logged every 10 pages

### T027: Integration Tests (test_ingest.py)
**File**: `tests/integration/test_ingest.py`
**Test Cases**: 11

Tests for `POST /ingest` endpoint in `src/api/ingest.py`:
- Full ingestion flow with default sitemap
- Custom sitemap URL support
- Duplicate ingestion prevention (409 Conflict)
- Malformed sitemap returns 400
- Sitemap not found returns 400
- Partial success with errors list
- Response structure validation
- Duration tracking
- Collection creation if needed
- All pages fail scenario
- Status values (success/partial_success/failed)

**Key Test Scenarios**:
- Concurrent: Second request while first running returns 409
- Error: Malformed XML returns 400 with error message
- Response: Includes status, pages_processed, chunks_created, duration_seconds, errors
- Partial: Some pages fail, status="partial_success", errors contains URLs
- Duration: 0 < duration < 300 seconds
- Status: One of ["success", "partial_success", "failed"]

## Running the Tests

### Prerequisites

Install dependencies:
```bash
cd backend
pip install -r requirements.txt
```

### Run All Tests

```bash
# Run all tests
pytest tests/

# Run with verbose output
pytest tests/ -v

# Run with coverage report
pytest --cov=src --cov-report=term-missing tests/

# Run with coverage HTML report
pytest --cov=src --cov-report=html tests/
```

### Run Specific Test Categories

```bash
# Run only unit tests
pytest tests/unit/

# Run only integration tests
pytest tests/integration/

# Run specific test file
pytest tests/unit/test_chunking.py

# Run specific test class
pytest tests/unit/test_chunking.py::TestChunkText

# Run specific test case
pytest tests/unit/test_chunking.py::TestChunkText::test_empty_text
```

### Run Tests with Markers

```bash
# Run only async tests
pytest tests/ -m asyncio

# Run only fast tests (exclude slow)
pytest tests/ -m "not slow"
```

### Parallel Test Execution

```bash
# Install pytest-xdist
pip install pytest-xdist

# Run tests in parallel (4 workers)
pytest tests/ -n 4
```

## Test Output Examples

### Successful Test Run
```
tests/unit/test_chunking.py ................  [ 14%]
tests/unit/test_embedding.py ...................  [ 36%]
tests/unit/test_vector_store.py ......................  [ 60%]
tests/unit/test_ingestion.py ................  [ 75%]
tests/integration/test_ingest.py ...........  [100%]

============= 108 passed in 5.23s =============
```

### Coverage Report
```
Name                                Stmts   Miss  Cover   Missing
-----------------------------------------------------------------
src/utils/chunking.py                  45      2    96%   84-85
src/services/embedding.py              89      5    94%
src/services/vector_store.py          112      8    93%
src/services/ingestion.py             156     12    92%
src/api/ingest.py                      42      3    93%
-----------------------------------------------------------------
TOTAL                                 444     30    93%
```

## Mocking Strategy

### External Services
All tests mock external API calls to ensure:
- Fast test execution (no network delays)
- Deterministic results (no external dependencies)
- Cost savings (no API usage charges)
- Offline testing capability

### Mocked Services
1. **Cohere API** (Embeddings)
   - Returns 1024-dimensional vectors
   - Simulates rate limiting
   - Handles batch operations

2. **Qdrant Cloud** (Vector Database)
   - Mocks collection operations
   - Simulates search results
   - Handles health checks

3. **HTTP Client** (Sitemap/Pages)
   - Returns sample sitemap XML
   - Returns sample HTML pages
   - Simulates various error conditions

## Test Design Principles

### 1. Isolation
Each test is independent and can run in any order.

### 2. Clarity
Test names clearly describe what is being tested.

### 3. Coverage
Tests cover:
- Happy path scenarios
- Edge cases (empty, null, boundary values)
- Error conditions
- Validation logic
- Retry mechanisms

### 4. Maintainability
- Fixtures reduce code duplication
- Clear test structure (Arrange-Act-Assert)
- Descriptive assertions

### 5. Speed
- All unit tests complete in <5 seconds
- Integration tests use mocked services

## Common Testing Patterns

### Async Test Pattern
```python
@pytest.mark.asyncio
async def test_async_operation(service):
    result = await service.async_method()
    assert result is not None
```

### Error Testing Pattern
```python
def test_error_handling(service):
    with pytest.raises(CustomError, match="error message"):
        service.method_that_fails()
```

### Mock Configuration Pattern
```python
@pytest.fixture
def mock_service():
    service = MagicMock()
    service.method.return_value = expected_value
    return service
```

## Troubleshooting

### Import Errors
If you see import errors, ensure you're running pytest from the `backend/` directory:
```bash
cd backend
pytest tests/
```

### Async Test Warnings
If async tests show warnings, ensure pytest-asyncio is installed:
```bash
pip install pytest-asyncio==0.23.3
```

### Coverage Not Working
Ensure pytest-cov is installed:
```bash
pip install pytest-cov==4.1.0
```

## Contributing

When adding new tests:
1. Follow existing naming conventions
2. Add appropriate fixtures to conftest.py
3. Include docstrings for test classes and methods
4. Test both success and failure scenarios
5. Ensure >80% coverage for new code

## Next Steps

After User Story 2 tests are complete:
- T036-T039: Tests for User Story 1 (Chat)
- T043: Tests for User Story 3 (Health)
- T046: Tests for User Story 4 (CORS)

## References

- [pytest documentation](https://docs.pytest.org/)
- [pytest-asyncio documentation](https://pytest-asyncio.readthedocs.io/)
- [pytest-cov documentation](https://pytest-cov.readthedocs.io/)
- [FastAPI testing guide](https://fastapi.tiangolo.com/tutorial/testing/)
