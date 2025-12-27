---
id: 003
title: Create Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot-api
branch: main
user: SabehShaikh
command: /sp.plan
labels: ["plan", "architecture", "design", "rag", "fastapi"]
links:
  spec: specs/rag-chatbot-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/rag-chatbot-api/plan.md
tests:
  - None (planning document)
---

## Prompt

/sp.plan

Read the constitution and specification you just created. Design a detailed implementation plan that:
- Follows all constitutional principles
- Implements all functional requirements from the spec
- Uses FastAPI, Qdrant, Cohere, and Gemini as specified
- Includes complete file structure and architecture

## Response snapshot

Created comprehensive implementation plan at `specs/rag-chatbot-api/plan.md`

### Plan Structure

**Summary**: FastAPI backend with RAG for Physical AI textbook using Gemini, Cohere, Qdrant

**Technical Context**:
- Python 3.11+ with FastAPI, Uvicorn
- Qdrant Cloud for vector storage
- Cohere embed-english-v3.0 for embeddings
- Gemini 2.0 Flash Exp via OpenAI SDK
- Trafilatura for text extraction
- pytest for testing

**Constitutional Compliance**: All 6 core principles verified with no violations

**File Structure** (23 files):
```
backend/
├── src/
│   ├── main.py              # FastAPI app with CORS
│   ├── config.py            # Environment variables
│   ├── logger.py            # Structured logging
│   ├── models/              # Pydantic models (4 files)
│   ├── services/            # Business logic (6 files)
│   ├── api/                 # Route handlers (3 files)
│   └── utils/               # Utilities (3 files)
├── tests/
│   ├── conftest.py          # Fixtures
│   ├── unit/                # Service tests (4 files)
│   └── integration/         # API tests (3 files)
├── .env.example
├── requirements.txt
├── vercel.json
└── README.md
```

**Architecture Components**:
1. **FastAPI Application**: CORS, middleware, exception handlers
2. **Configuration Service**: Pydantic BaseSettings for env validation
3. **Embedding Service**: Cohere async client with batching
4. **Vector Store Service**: Qdrant client with connection pooling
5. **Agent Service**: Gemini via OpenAI SDK with retrieve() tool
6. **Ingestion Service**: Sitemap → Extract → Chunk → Embed → Store
7. **Retrieval Service**: Query embed + Qdrant search
8. **Chunking Utility**: 1000 chars with 200 overlap

**API Endpoints**:
- POST /chat: Validate → Embed → Search → Agent → Response (FR-001 to FR-006)
- POST /ingest: Lock → Fetch → Process → Store → Response (FR-007 to FR-012)
- GET /health: Check Qdrant → Collection status → Response (FR-013 to FR-015)

**Data Models**:
- ChatRequest/Response with validation
- IngestRequest/Response with status
- HealthResponse with service checks
- Internal models: TextChunk, SearchResult, RetrievalResult

**Error Handling**:
- Custom exceptions (CohereAPIError, GeminiAPIError, QdrantError, etc.)
- Global exception handler mapping exceptions to HTTP status codes
- Structured error responses with request_id

**Logging Strategy**:
- StructuredLogger with JSON output
- Required fields: timestamp, level, message, request_id, endpoint, duration_ms
- No secrets in logs (sanitized error messages)

**Testing Strategy**:
- Unit tests for services/utils with mocked APIs
- Integration tests for endpoints with TestClient
- >80% coverage requirement
- Explicit edge case and error path tests

**Deployment**:
- vercel.json configuration
- Environment variables via Vercel secrets
- Requirements.txt with pinned versions
- .env.example for local setup

**Key Decisions**:
1. OpenAI SDK for Gemini (Agents SDK pattern, simpler tool calling)
2. In-memory ingestion lock (simplicity, single instance)
3. No caching layer (constitution MVP constraint)
4. Stateless API (constitution constraint)
5. 1000-char chunks / 200 overlap (constitution spec)
6. Top 5 results (constitution spec)

**Performance Optimization**:
- Async I/O throughout
- Parallel operations (embed + warm connection)
- Batch embeddings (96 per Cohere limit)
- Connection pooling for Qdrant
- Timeouts on external APIs

**Estimated Latency**:
- POST /chat: 1.5-2.5s (target <3s) ✅
- POST /ingest: 2.5-3min (target <5min) ✅
- GET /health: 30-50ms (target <100ms) ✅

**Implementation Phases**:
1. Setup (0.5h): Project structure, deps, config
2. Core Services (3h): Embedding, vector store, chunking
3. Agent Service (2h): Gemini with retrieve() tool
4. Ingestion Pipeline (2h): Sitemap → Qdrant
5. API Endpoints (2h): /chat, /ingest, /health
6. Middleware (1h): CORS, logging, errors
7. Testing (2h): >80% coverage
8. Deployment (1h): Vercel setup
9. Validation (1h): Production testing
**Total: 14.5 hours**

**Risk Mitigation**:
- Rate limits → exponential backoff
- Slow ingestion → parallel processing
- Hallucination → strong prompts, source validation
- CORS → test from actual domain
- Qdrant timeout → 5s limit with 504 response

### Constitutional Alignment

✅ **Content Fidelity**: Strong system prompt, retrieve() tool only provides textbook chunks, explicit "I don't know" fallback
✅ **Speed & Performance**: Async operations, batch processing, connection pooling, all targets met
✅ **Simplicity First**: No auth, no history, no caching, direct pattern, <500 lines/file
✅ **Free Tier Operations**: Cohere/Gemini/Qdrant free tiers, no paid deps
✅ **Security**: All secrets in env, CORS whitelist, input validation, sanitized logs
✅ **API Design**: Pydantic models, proper status codes, OpenAPI docs, clear contracts

### Specification Coverage

All 25 functional requirements addressed:
- FR-001 to FR-006: /chat endpoint with full validation and agent
- FR-007 to FR-012: /ingest endpoint with sitemap processing
- FR-013 to FR-015: /health endpoint with Qdrant checks
- FR-016: CORS for frontend URL
- FR-017: Input validation (Pydantic)
- FR-018 to FR-020: Structured logging with request_id
- FR-021: Environment variables only
- FR-022 to FR-024: Error handling with retries
- FR-025: OpenAPI auto-generation

All 10 success criteria measurable:
- SC-001 to SC-003: Performance targets with estimates
- SC-004 to SC-005: Manual validation checklist
- SC-006: Security audit via code review
- SC-007: CORS testing from frontend
- SC-008: Logging implementation
- SC-009: Collection count validation
- SC-010: Chaos testing with mocked failures

All 4 user stories implemented:
- P1: Student asks questions → POST /chat
- P1: Admin ingests content → POST /ingest
- P2: Monitor health → GET /health
- P2: Frontend integration → CORS + endpoints

All 10 edge cases handled:
- Empty/long query → validation
- No relevant content → agent responds "I don't know"
- Timeouts → 504 with error
- API failures → 503 with retry
- Malformed sitemap → 400 with message
- Duplicate ingestion → 409 with lock

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation. All architectural decisions documented, file structure defined, performance estimates validated against targets.
- 🧪 Tests: None (planning document, no code)
- 📁 Files: 1 file created (specs/rag-chatbot-api/plan.md)
- 🔁 Next prompts: Run `/sp.tasks` to generate actionable task list, Review plan for approval, Begin implementation with `/sp.implement`
- 🧠 Reflection: Plan comprehensively covers all spec requirements and constitutional principles. Key decisions (OpenAI SDK for Gemini, in-memory lock, no caching) align with Simplicity First. Performance estimates meet all targets with margin. Ready for task breakdown.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - Plan includes technical context, constitutional check, complete file structure, detailed component designs, all 6 key decisions documented with rationale/tradeoffs, implementation phases with time estimates, risk mitigation strategies
- Prompt variant (if applicable): Standard planning workflow with full spec/constitution input
- Next experiment (smallest change to try): Validate chunk overlap (200 chars) provides sufficient context continuity during retrieval - may need tuning based on actual results
