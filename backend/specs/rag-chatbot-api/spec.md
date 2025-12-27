# Feature Specification: RAG Chatbot API

**Feature Branch**: `main`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "FastAPI backend providing RAG question-answering for Physical AI textbook using Gemini, Cohere, and Qdrant."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Asks Textbook Question (Priority: P1)

As a student studying Physical AI, I want to ask questions about textbook content and receive accurate, sourced answers so I can quickly understand concepts without manually searching through pages.

**Why this priority**: Core value proposition. Without accurate question-answering, the entire system has no purpose. This is the MVP that delivers immediate student value.

**Independent Test**: Can be fully tested by sending POST /chat requests with textbook-related questions and verifying answers contain correct information with source URLs. Delivers immediate value even without other features.

**Acceptance Scenarios**:

1. **Given** textbook content is ingested into Qdrant, **When** student asks "What is reinforcement learning?", **Then** system returns accurate answer from textbook with source URL
2. **Given** textbook content is ingested, **When** student asks question about topic not in textbook, **Then** system responds "I don't have information about this in the textbook"
3. **Given** textbook content is ingested, **When** student asks "Explain transformers", **Then** system returns answer with multiple relevant source chunks and response time < 3 seconds
4. **Given** textbook content is ingested, **When** student asks vague question "Tell me about AI", **Then** system provides focused answer from relevant textbook sections
5. **Given** API is running, **When** student sends malformed query (empty, too long, non-string), **Then** system returns 400 error with clear message

---

### User Story 2 - Admin Ingests Textbook Content (Priority: P1)

As a system administrator, I want to ingest textbook content from the sitemap so the chatbot has up-to-date knowledge base to answer questions.

**Why this priority**: P1 because chat functionality depends on ingested content. However, this is one-time operation vs repeated chat queries. Both P1 stories are required for MVP.

**Independent Test**: Can be fully tested by triggering POST /ingest and verifying all sitemap pages are extracted, chunked, embedded, and stored in Qdrant. Success measured by chunk count and Qdrant collection status.

**Acceptance Scenarios**:

1. **Given** sitemap.xml contains 50 textbook pages, **When** admin triggers POST /ingest, **Then** system fetches all pages, chunks text, embeds with Cohere, stores in Qdrant, and returns success with count
2. **Given** ingestion is triggered, **When** a page fails to fetch (404), **Then** system logs error but continues processing remaining pages
3. **Given** ingestion is running, **When** process completes, **Then** total time is under 5 minutes for typical textbook size
4. **Given** Qdrant collection exists with data, **When** ingestion is triggered again, **Then** system overwrites/updates existing data idempotently
5. **Given** Cohere API rate limit is hit, **When** ingestion continues, **Then** system implements exponential backoff and retries

---

### User Story 3 - Monitor System Health (Priority: P2)

As a system administrator or monitoring service, I want to check API health and connectivity to external services so I can detect and respond to outages quickly.

**Why this priority**: P2 because system can function without health checks in MVP, but critical for production reliability and debugging. Independent from core chat functionality.

**Independent Test**: Can be fully tested by calling GET /health and verifying response includes Qdrant connectivity, collection status, and response time < 100ms. Delivers operational visibility independently.

**Acceptance Scenarios**:

1. **Given** all services are running, **When** GET /health is called, **Then** returns 200 with status "healthy" and Qdrant collection count
2. **Given** Qdrant is unreachable, **When** GET /health is called, **Then** returns 503 with status "unhealthy" and error details
3. **Given** Qdrant collection doesn't exist, **When** GET /health is called, **Then** returns 200 but indicates collection not initialized
4. **Given** API is running, **When** health check is called, **Then** response time is under 100ms

---

### User Story 4 - Frontend Integration (Priority: P2)

As a frontend application, I want to call the chat API from the Vercel-hosted React app so users can interact with the chatbot through a web interface.

**Why this priority**: P2 because API can be tested independently via Postman/curl, but web UI is essential for end-user access. Requires CORS configuration but not complex functionality.

**Independent Test**: Can be fully tested by making fetch() calls from https://ai-native-text-book.vercel.app to API endpoints and verifying CORS headers allow requests and responses are received.

**Acceptance Scenarios**:

1. **Given** API has CORS configured for frontend URL, **When** frontend makes POST /chat request, **Then** request succeeds with proper CORS headers
2. **Given** API is running, **When** request comes from unauthorized origin, **Then** CORS policy blocks the request
3. **Given** frontend makes request, **When** API returns response, **Then** response includes Access-Control-Allow-Origin header

---

### Edge Cases

- **Empty Query**: What happens when user sends empty or whitespace-only question?
  - System returns 400 with message "Question cannot be empty"

- **Very Long Query**: What happens when query exceeds 500 characters?
  - System returns 400 with message "Question too long (max 500 characters)"

- **No Relevant Content**: What happens when query is valid but Qdrant returns no similar chunks (similarity < 0.5)?
  - Agent responds "I don't have information about this in the textbook"

- **Qdrant Timeout**: What happens when Qdrant search takes > 5 seconds?
  - System returns 504 Gateway Timeout with message "Search service timeout"

- **Cohere API Failure During Ingestion**: What happens when Cohere embedding fails mid-ingestion?
  - System logs error, skips that chunk, continues processing, returns partial success

- **Gemini API Failure During Chat**: What happens when Gemini API is down?
  - System returns 503 with message "AI service temporarily unavailable"

- **Malformed Sitemap**: What happens when sitemap.xml is invalid or unreachable?
  - Ingestion returns 400 with message "Cannot fetch or parse sitemap"

- **Duplicate Ingestion**: What happens when POST /ingest is called while ingestion is running?
  - System returns 409 Conflict with message "Ingestion already in progress"

- **Rate Limiting**: What happens when user sends 100 queries in 1 minute?
  - Future consideration - currently no rate limiting in MVP

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept POST /chat with JSON body containing "query" string field
- **FR-002**: System MUST embed user query using Cohere embed-english-v3.0 model
- **FR-003**: System MUST search Qdrant collection "ai-textbook" for top 5 most similar chunks
- **FR-004**: System MUST use Gemini 2.0 Flash Exp via OpenAI SDK with retrieve() tool
- **FR-005**: System MUST return JSON response with "answer", "sources" (array of URLs), and "response_time" (float)
- **FR-006**: System MUST refuse to answer questions outside textbook content with explicit message
- **FR-007**: System MUST accept POST /ingest to trigger content ingestion from sitemap
- **FR-008**: System MUST fetch sitemap.xml from configured URL and extract all page URLs
- **FR-009**: System MUST extract text content from each page using Trafilatura library
- **FR-010**: System MUST chunk text into 1000-character segments with overlap
- **FR-011**: System MUST embed each chunk using Cohere and store in Qdrant with metadata (text, url, title, chunk_index)
- **FR-012**: System MUST return ingestion status with pages_processed and chunks_created counts
- **FR-013**: System MUST accept GET /health and return service status
- **FR-014**: System MUST check Qdrant connectivity and collection status in health endpoint
- **FR-015**: System MUST respond to health checks within 100ms
- **FR-016**: System MUST configure CORS to allow requests from https://ai-native-text-book.vercel.app
- **FR-017**: System MUST validate query input (non-empty, <= 500 characters)
- **FR-018**: System MUST log all requests with timestamp, endpoint, duration, and status
- **FR-019**: System MUST return appropriate HTTP status codes (200, 400, 500, 503, 504)
- **FR-020**: System MUST include request_id in all log entries for tracing
- **FR-021**: System MUST load configuration from environment variables (no hardcoded secrets)
- **FR-022**: System MUST handle Qdrant connection failures gracefully with retries (3 attempts, exponential backoff)
- **FR-023**: System MUST handle Cohere API failures during chat with fallback error response
- **FR-024**: System MUST handle Gemini API failures with user-friendly error message
- **FR-025**: System MUST generate OpenAPI documentation automatically via FastAPI

### Key Entities

- **TextChunk**: Represents a segment of textbook content
  - Attributes: id (UUID), text (string, 1000 chars), url (source page), title (page title), chunk_index (int), embedding (1024-dim vector)
  - Stored in: Qdrant collection "ai-textbook"
  - Relationships: Multiple chunks per textbook page

- **ChatQuery**: Represents a user question
  - Attributes: query (string, 1-500 chars), timestamp (ISO8601), request_id (UUID)
  - Transient: Not persisted, only logged

- **ChatResponse**: Represents system answer
  - Attributes: answer (string), sources (array of URLs), response_time (float seconds), request_id (UUID)
  - Transient: Not persisted, returned to client

- **IngestionJob**: Represents content ingestion operation
  - Attributes: job_id (UUID), status (string: running|completed|failed), pages_processed (int), chunks_created (int), started_at (timestamp), completed_at (timestamp)
  - Future: May be persisted for audit trail, currently in-memory only

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chat endpoint responds in under 3 seconds for 95% of queries (p95 latency < 3s)
- **SC-002**: Ingestion completes processing entire textbook sitemap in under 5 minutes
- **SC-003**: Health endpoint responds in under 100ms for 95% of requests
- **SC-004**: System provides accurate answers with source attribution for questions covered in textbook (manual validation on 20 sample questions)
- **SC-005**: System correctly refuses to answer questions outside textbook scope (manual validation on 10 out-of-scope questions)
- **SC-006**: Zero exposed secrets in code, logs, or error messages (security audit)
- **SC-007**: API successfully accepts requests from Vercel frontend with CORS enabled
- **SC-008**: System logs every request with request_id, duration, and status for traceability
- **SC-009**: Qdrant collection contains embeddings for all textbook pages after ingestion (count validation)
- **SC-010**: System handles Cohere/Gemini/Qdrant failures gracefully without crashing (chaos testing)

## Technical Specifications

### API Configuration

**Environment Variables** (from user input):
```
QDRANT_URL=https://f649b7e4-cc63-4ad6-8c2c-5eabbefb6b9d.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.aME3iadAVK4Zvv7K_srpl79AT_I8gOVGyGQbBVQuJkU
QDRANT_COLLECTION=ai-textbook
COHERE_API_KEY=SdUVIq13qDWB6tqhJZ30yQduzK8037LbxjHDX2tp
GEMINI_API_KEY=AIzaSyAOsRiQykUK7u2kWBbRIHrFwMT1RcFr95g
SITEMAP_URL=https://ai-native-text-book.vercel.app/sitemap.xml
FRONTEND_URL=https://ai-native-text-book.vercel.app
```

### Technology Stack

- **FastAPI**: Python web framework for API endpoints
- **Qdrant Python Client**: Vector database interaction
- **Cohere Python SDK**: Text embeddings (embed-english-v3.0 model)
- **OpenAI Python SDK**: Gemini API client via OpenAI-compatible endpoint
  - Base URL: https://generativelanguage.googleapis.com/v1beta/openai/
  - Model: gemini-1.5-flash
- **Trafilatura**: Web page text extraction
- **Python 3.11+**: Runtime environment

### Agent Configuration

**Gemini Agent Setup** (OpenAI Agents SDK pattern):
```python
System Prompt: "You are an AI tutor for a Physical AI textbook. Use the retrieve() tool to search for relevant content. Answer questions ONLY using information from the textbook. If the information is not in the textbook, respond with 'I don't have information about this in the textbook.'"

Tool Definition:
- Name: retrieve
- Description: "Search the textbook for relevant content"
- Parameters: query (string) - semantic search query
- Returns: List of relevant text chunks with source URLs
```

### API Endpoint Contracts

**POST /chat**
```json
Request:
{
  "query": "What is reinforcement learning?"
}

Response (200):
{
  "answer": "Reinforcement learning is a type of machine learning where an agent learns to make decisions by interacting with an environment...",
  "sources": [
    "https://ai-native-text-book.vercel.app/week4",
    "https://ai-native-text-book.vercel.app/week5"
  ],
  "response_time": 1.234
}

Error (400):
{
  "detail": "Question cannot be empty"
}

Error (503):
{
  "detail": "AI service temporarily unavailable"
}
```

**POST /ingest**
```json
Request (optional body):
{
  "sitemap_url": "https://ai-native-text-book.vercel.app/sitemap.xml"
}

Response (200):
{
  "status": "success",
  "pages_processed": 50,
  "chunks_created": 834,
  "duration_seconds": 243.5
}

Error (400):
{
  "detail": "Cannot fetch or parse sitemap"
}

Error (409):
{
  "detail": "Ingestion already in progress"
}
```

**GET /health**
```json
Response (200):
{
  "status": "healthy",
  "qdrant_status": "connected",
  "collection_exists": true,
  "collection_count": 834,
  "timestamp": "2025-12-18T10:30:00Z"
}

Response (503):
{
  "status": "unhealthy",
  "qdrant_status": "unreachable",
  "error": "Connection timeout",
  "timestamp": "2025-12-18T10:30:00Z"
}
```

### Data Flow

**Chat Flow**:
1. Client sends POST /chat with query
2. FastAPI validates input (length, type)
3. Generate request_id, log request start
4. Embed query using Cohere SDK (1024-dim vector)
5. Search Qdrant for top 5 similar chunks (cosine similarity)
6. Pass chunks to Gemini agent via retrieve() tool
7. Agent generates answer using only provided chunks
8. Format response with answer, source URLs, response_time
9. Log request completion with duration
10. Return JSON response to client

**Ingestion Flow**:
1. Admin sends POST /ingest
2. FastAPI checks if ingestion already running (409 if yes)
3. Fetch sitemap.xml from SITEMAP_URL
4. Parse XML to extract all page URLs
5. For each URL:
   a. Fetch HTML with Trafilatura
   b. Extract clean text content
   c. Split into 1000-char chunks (with 200-char overlap)
   d. Embed each chunk with Cohere
   e. Store in Qdrant with payload {text, url, title, chunk_index}
6. Log progress every 10 pages
7. Return summary with counts and duration

**Health Check Flow**:
1. Client sends GET /health
2. FastAPI attempts Qdrant connection
3. Check if collection "ai-textbook" exists
4. Query collection count
5. Return status with details (< 100ms total)

## Performance Requirements

- **Latency**:
  - POST /chat: p50 < 1s, p95 < 3s, p99 < 5s
  - GET /health: p95 < 100ms
  - POST /ingest: Total < 5 minutes for typical textbook (~50 pages)

- **Throughput**:
  - Concurrent requests: Support 10 concurrent /chat requests
  - No rate limiting in MVP (future: 100 req/min per IP)

- **Resource Limits**:
  - Memory: < 512MB under normal load
  - CPU: Single core sufficient
  - Network: Depends on Qdrant/Cohere/Gemini latency

## Security Requirements

- **Authentication**: None in MVP (future: API key auth)
- **CORS**: Strict origin whitelist (only FRONTEND_URL)
- **Input Validation**: Max query length 500 chars, type checking
- **Secrets Management**: All API keys in environment variables
- **Logging**: No secrets in logs, sanitize error messages
- **Rate Limiting**: Not implemented in MVP (future consideration)

## Error Handling

All endpoints return structured error responses:
```json
{
  "detail": "Human-readable error message",
  "request_id": "uuid-v4",
  "timestamp": "ISO8601"
}
```

**HTTP Status Codes**:
- 200: Success
- 400: Bad Request (invalid input)
- 409: Conflict (duplicate operation)
- 500: Internal Server Error (unexpected failure)
- 503: Service Unavailable (external API down)
- 504: Gateway Timeout (external API timeout)

## Out of Scope

The following features are explicitly **NOT** included in this specification:

- **User Authentication**: No login, API keys, or user sessions
- **Chat History**: No persistence of previous conversations
- **Text Selection Feature**: No ability to select specific text from textbook for context
- **Rate Limiting**: No per-user or per-IP request throttling
- **Caching**: No query result caching or embedding caching
- **Admin Dashboard**: No UI for monitoring or management
- **Usage Analytics**: No tracking of query patterns or user behavior
- **Multi-tenancy**: Single textbook, single collection
- **Streaming Responses**: No SSE or WebSocket streaming
- **Fine-tuning**: No custom model training on textbook content

## Dependencies

**External Services**:
- Qdrant Cloud (free tier): Vector storage and search
- Cohere API (free tier): Text embeddings
- Gemini API (free tier): LLM generation
- Vercel (frontend hosting): CORS origin

**Python Libraries**:
- fastapi: Web framework
- uvicorn: ASGI server
- qdrant-client: Qdrant SDK
- cohere: Cohere SDK
- openai: OpenAI SDK (for Gemini)
- trafilatura: Text extraction
- pydantic: Data validation
- python-dotenv: Environment loading

## Assumptions

1. Textbook content at sitemap URL is stable (not changing frequently)
2. Free tier limits are sufficient for expected usage (<1000 queries/day)
3. Qdrant collection can be overwritten during ingestion (no incremental updates needed)
4. English language only (no i18n required)
5. No user accounts means no personalization or history
6. Textbook pages are publicly accessible (no auth required)
7. Sitemap.xml is well-formed and contains all relevant pages
8. Text extraction via Trafilatura is sufficient (no complex PDF/image handling)
9. 1000-character chunks provide good semantic granularity
10. Top 5 chunks provide sufficient context for Gemini to answer

## Open Questions

None - all configuration and requirements provided by user.

## Risks

1. **Free Tier Limits**: Risk of hitting Cohere/Gemini/Qdrant rate limits
   - Mitigation: Monitor usage, implement exponential backoff, consider caching

2. **Chunk Size**: 1000 chars may be too large or too small for optimal retrieval
   - Mitigation: Make chunk size configurable, test with different values

3. **Hallucination**: Gemini may generate answers not strictly from textbook
   - Mitigation: Strong system prompt, validate answers include source citations

4. **Ingestion Time**: May exceed 5 minutes for large textbooks
   - Mitigation: Parallel processing, batch embeddings, optimize Trafilatura

5. **CORS Issues**: Frontend may face browser CORS restrictions
   - Mitigation: Proper CORS headers, test from actual Vercel domain

## Revision History

- **v1.0** (2025-12-18): Initial specification based on user requirements
