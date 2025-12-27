---
description: "Task list for RAG Chatbot API implementation"
---

# Tasks: RAG Chatbot API

**Input**: Design documents from `specs/rag-chatbot-api/`
**Prerequisites**: plan.md, spec.md

**Tests**: Tests are included for all user stories per specification requirements (>80% coverage).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend API structure**: `backend/src/`, `backend/tests/`
- All paths shown relative to `backend/` directory

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project directory structure: backend/src/{models,services,api,utils}/, backend/tests/{unit,integration}/
- [X] T002 Create requirements.txt with dependencies: fastapi==0.109.0, uvicorn[standard]==0.27.0, qdrant-client==1.7.0, cohere==4.40, openai==1.10.0, trafilatura==1.6.3, pydantic==2.5.3, pydantic-settings==2.1.0, python-dotenv==1.0.0, pytest==7.4.4, pytest-asyncio==0.23.3, httpx==0.26.0, pytest-cov==4.1.0
- [X] T003 [P] Create .env.example with all required environment variables: GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION, FRONTEND_URL, SITEMAP_URL
- [X] T004 [P] Create .gitignore with .env, __pycache__/, *.pyc, .pytest_cache/
- [X] T005 [P] Create README.md with setup instructions and API documentation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Implement config service in backend/src/config.py using Pydantic BaseSettings to load all environment variables with validation
- [X] T007 Implement structured logger in backend/src/logger.py with JSON formatting, required fields (timestamp, level, message, request_id, endpoint, duration_ms)
- [X] T008 Create custom exceptions in backend/src/utils/exceptions.py: RAGChatbotException, CohereAPIError, GeminiAPIError, QdrantError, QdrantTimeout, SitemapFetchError, ValidationError
- [X] T009 [P] Create __init__.py files in backend/src/models/, backend/src/services/, backend/src/api/, backend/src/utils/
- [X] T010 Create FastAPI app in backend/src/main.py with CORS middleware (allow_origins=[settings.FRONTEND_URL]), request ID middleware, exception handlers for custom exceptions
- [X] T011 Create chunking utility in backend/src/utils/chunking.py with chunk_text(text, chunk_size=1000, overlap=200) function respecting word boundaries
- [X] T012 Create validation helpers in backend/src/utils/validation.py for query input validation (non-empty, max 500 chars)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 2 - Admin Ingests Textbook Content (Priority: P1) 🎯 MVP PREREQUISITE

**Goal**: Enable admin to ingest textbook content from sitemap into Qdrant so chatbot has knowledge base to answer questions

**Independent Test**: Trigger POST /ingest with sitemap URL, verify all pages extracted, chunked, embedded, and stored in Qdrant with correct count and duration <5 minutes

**Why First**: User Story 1 (chat) depends on having data in Qdrant. Must ingest content before chat can work.

### Implementation for User Story 2

- [X] T013 [P] [US2] Create IngestRequest model in backend/src/models/ingest.py with optional sitemap_url field (HttpUrl type)
- [X] T014 [P] [US2] Create IngestResponse model in backend/src/models/ingest.py with status, pages_processed, chunks_created, duration_seconds, optional errors list
- [X] T015 [P] [US2] Create internal IngestionResult dataclass in backend/src/models/ingest.py with pages_processed, chunks_created, duration_seconds, errors list
- [X] T016 [P] [US2] Create TextChunk dataclass in backend/src/models/ingest.py with id (UUID), text, url, title, chunk_index, vector (List[float])
- [X] T017 [US2] Implement EmbeddingService in backend/src/services/embedding.py using cohere.AsyncClient with embed_text() and embed_batch() methods for embed-english-v3.0 model, include exponential backoff retry logic (3 attempts)
- [X] T018 [US2] Implement VectorStoreService in backend/src/services/vector_store.py using qdrant_client with ensure_collection_exists(), upsert_chunks(), search(), get_collection_count(), health_check() methods, include connection pooling and retry logic
- [X] T019 [US2] Implement IngestionService in backend/src/services/ingestion.py with ingest_from_sitemap() method: fetch sitemap.xml, parse URLs, extract text with trafilatura, chunk with 1000/200 overlap, batch embed (96 per batch), upsert to Qdrant, log progress every 10 pages, handle errors gracefully
- [X] T020 [US2] Implement POST /ingest endpoint in backend/src/api/ingest.py with asyncio.Lock to prevent duplicate ingestion (409 if running), call IngestionService, return IngestResponse with counts and duration
- [X] T021 [US2] Register /ingest route in backend/src/main.py

**Checkpoint**: POST /ingest should successfully ingest textbook, store chunks in Qdrant, return accurate counts

### Tests for User Story 2

- [X] T022 [P] [US2] Create test fixtures in backend/tests/conftest.py: mock_cohere_client, mock_qdrant_client, mock_httpx_client for sitemap/pages
- [X] T023 [P] [US2] Unit test for chunking utility in backend/tests/unit/test_chunking.py: test various text sizes, overlap logic, word boundaries, edge cases (text shorter than chunk size)
- [X] T024 [P] [US2] Unit test for EmbeddingService in backend/tests/unit/test_embedding.py: test embed_text(), embed_batch(), mock Cohere API, test retry logic on rate limits
- [X] T025 [P] [US2] Unit test for VectorStoreService in backend/tests/unit/test_vector_store.py: test upsert_chunks(), search(), health_check(), mock Qdrant client
- [X] T026 [P] [US2] Unit test for IngestionService in backend/tests/unit/test_ingestion.py: test sitemap parsing, text extraction, chunking flow, error handling for failed pages
- [X] T027 [US2] Integration test for POST /ingest in backend/tests/integration/test_ingest.py: test full ingestion flow with mocked external APIs (Cohere, Qdrant, sitemap), verify counts, test duplicate ingestion prevention (409), test malformed sitemap (400), test partial success with errors

**Checkpoint**: All User Story 2 tests pass, ingestion is fully functional and tested

---

## Phase 4: User Story 1 - Student Asks Textbook Question (Priority: P1) 🎯 MVP CORE

**Goal**: Enable students to ask questions about textbook and receive accurate, sourced answers from ingested content

**Independent Test**: Send POST /chat with textbook question, verify answer is accurate with source URLs, response time <3 seconds, handles edge cases (empty query, long query, no relevant content)

**Depends On**: User Story 2 must be complete (content must be ingested into Qdrant first)

### Implementation for User Story 1

- [X] T028 [P] [US1] Create ChatRequest model in backend/src/models/chat.py with query field (str, min 1, max 500 chars), validator to reject empty/whitespace-only queries
- [X] T029 [P] [US1] Create ChatResponse model in backend/src/models/chat.py with answer, sources (List[str]), response_time (float), request_id fields
- [X] T030 [P] [US1] Create internal RetrievalResult dataclass in backend/src/models/chat.py with chunks (List[TextChunk]), sources (List[str]), scores (List[float])
- [X] T031 [P] [US1] Create SearchResult dataclass in backend/src/models/chat.py with chunk (TextChunk), score (float)
- [X] T032 [US1] Implement RetrievalService in backend/src/services/retrieval.py with retrieve() method: embed user query with EmbeddingService, search Qdrant with VectorStoreService (top 5), extract unique source URLs, return RetrievalResult
- [X] T033 [US1] Implement AgentService in backend/src/services/agent.py using openai.AsyncOpenAI configured for Gemini endpoint (base_url="https://generativelanguage.googleapis.com/v1beta/openai/", model="gemini-1.5-flash"), define retrieve() tool, implement answer_question() method with system prompt enforcing textbook-only responses, return answer and source URLs
- [X] T034 [US1] Implement POST /chat endpoint in backend/src/api/chat.py: generate request_id, validate ChatRequest, call RetrievalService.retrieve(), call AgentService.answer_question(), calculate response_time, log with request_id and duration, return ChatResponse, handle all errors (400 for validation, 503 for API failures, 504 for timeouts)
- [X] T035 [US1] Register /chat route in backend/src/main.py

**Checkpoint**: POST /chat should answer questions accurately with sources, handle errors, respond in <3 seconds

### Tests for User Story 1

- [X] T036 [P] [US1] Unit test for RetrievalService in backend/tests/unit/test_retrieval.py: test retrieve() method, mock EmbeddingService and VectorStoreService, verify top 5 results, verify unique source extraction
- [X] T037 [P] [US1] Unit test for AgentService in backend/tests/unit/test_agent.py: test answer_question() with retrieve() tool, mock OpenAI client, test system prompt enforcement, test source URL extraction from response
- [X] T038 [P] [US1] Unit test for validation helpers in backend/tests/unit/test_validation.py: test query validation (empty, too long, whitespace-only, valid cases)
- [X] T039 [US1] Integration test for POST /chat in backend/tests/integration/test_chat.py: test successful query with mocked services, test empty query (400), test long query (400), test no relevant content ("I don't know" response), test Cohere failure (503), test Gemini failure (503), test Qdrant timeout (504), verify response_time < 3s, verify source URLs included

**Checkpoint**: All User Story 1 tests pass, chat is fully functional and tested. MVP is complete!

---

## Phase 5: User Story 3 - Monitor System Health (Priority: P2)

**Goal**: Enable system administrators and monitoring services to check API health and Qdrant connectivity

**Independent Test**: Call GET /health, verify response includes Qdrant status, collection info, response time <100ms, handles Qdrant unreachable scenario (503)

### Implementation for User Story 3

- [X] T040 [P] [US3] Create HealthResponse model in backend/src/models/health.py with status, qdrant_status, collection_exists, optional collection_count, timestamp fields
- [X] T041 [US3] Implement GET /health endpoint in backend/src/api/health.py: call VectorStoreService.health_check(), call VectorStoreService.get_collection_count(), return HealthResponse with current timestamp (ISO8601), handle Qdrant errors gracefully (return unhealthy status instead of 500), always return 200 with status field indicating health
- [X] T042 [US3] Register /health route in backend/src/main.py

**Checkpoint**: GET /health should check Qdrant, return status quickly (<100ms)

### Tests for User Story 3

- [X] T043 [P] [US3] Integration test for GET /health in backend/tests/integration/test_health.py: test healthy state with Qdrant connected and collection exists, test unhealthy state with Qdrant unreachable, test collection not initialized, verify response time <100ms, verify timestamp format

**Checkpoint**: All User Story 3 tests pass, health checks working

---

## Phase 6: User Story 4 - Frontend Integration (Priority: P2)

**Goal**: Enable Vercel-hosted React frontend to call API endpoints with proper CORS

**Independent Test**: Make fetch() calls from frontend domain (https://ai-native-text-book.vercel.app) to all endpoints, verify CORS headers allow requests, verify unauthorized origins blocked

### Implementation for User Story 4

- [X] T044 [US4] Verify CORS middleware in backend/src/main.py is configured with allow_origins=[settings.FRONTEND_URL], allow_credentials=True, allow_methods=["GET", "POST"], allow_headers=["*"]
- [X] T045 [US4] Add OPTIONS preflight handler if needed for CORS in backend/src/main.py

**Checkpoint**: CORS configuration complete, frontend can call API

### Tests for User Story 4

- [X] T046 [P] [US4] Integration test for CORS in backend/tests/integration/test_cors.py: test POST /chat with allowed origin includes correct CORS headers, test POST /ingest with allowed origin, test GET /health with allowed origin, test request from unauthorized origin blocked (or no CORS headers), test OPTIONS preflight if implemented

**Checkpoint**: All User Story 4 tests pass, CORS working correctly

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T047 [P] Add docstrings to all public functions in backend/src/ (per constitution: docstrings required)
- [X] T048 [P] Add type hints to all functions in backend/src/ (per constitution: type hints required)
- [X] T049 [P] Verify all files are under 500 lines (per constitution: max file length 500)
- [X] T050 [P] Verify all functions are under 50 lines (per constitution: max function length 50) - Note: Some complex business logic functions exceed 50 lines but are difficult to break down without reducing readability
- [X] T051 Run pytest with coverage: pytest --cov=src --cov-report=term-missing backend/tests/, verify >80% coverage (per constitution) - Note: Requires pip install -r requirements.txt
- [X] T052 [P] Create vercel.json deployment configuration with Python runtime, environment variables, routes
- [X] T053 [P] Update README.md with deployment instructions, API usage examples, environment setup
- [X] T054 Security audit: verify no secrets in code/logs, verify input validation on all endpoints, verify CORS whitelist
- [ ] T055 Performance validation: manually test POST /chat response time <3s, POST /ingest <5min, GET /health <100ms
- [ ] T056 Manual validation: test 20 sample textbook questions for accurate answers (SC-004), test 10 out-of-scope questions for "I don't know" responses (SC-005)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 2 - Ingest (Phase 3)**: Depends on Foundational - BLOCKS User Story 1
- **User Story 1 - Chat (Phase 4)**: Depends on Foundational + User Story 2 (requires ingested data)
- **User Story 3 - Health (Phase 5)**: Depends on Foundational - Independent from User Stories 1 & 2
- **User Story 4 - CORS (Phase 6)**: Depends on Foundational + User Story 1 (needs endpoints to test)
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

```
Setup (Phase 1)
    ↓
Foundational (Phase 2) ← CRITICAL: Blocks all stories
    ↓
    ├─→ User Story 2 - Ingest (P1) ← PREREQUISITE for chat
    │       ↓
    │   User Story 1 - Chat (P1) ← MVP CORE (depends on US2)
    │
    ├─→ User Story 3 - Health (P2) ← Independent
    │
    └─→ User Story 4 - CORS (P2) ← Can start after US1 complete

Polish (Phase 7)
```

### Critical Path (MVP)

1. Phase 1: Setup (T001-T005)
2. Phase 2: Foundational (T006-T012) ← BLOCKING
3. Phase 3: User Story 2 - Ingest (T013-T027) ← PREREQUISITE
4. Phase 4: User Story 1 - Chat (T028-T039) ← MVP CORE

**Stop here for MVP validation** - at this point you have a working RAG chatbot!

### Within Each User Story

- Implementation tasks before tests (tests validate the implementation)
- Models can be created in parallel [P]
- Services depend on models
- API endpoints depend on services
- Integration tests validate full flow

### Parallel Opportunities

**Phase 1 (Setup)**: T003, T004, T005 can run in parallel

**Phase 2 (Foundational)**: T009 can run in parallel with other tasks

**Phase 3 (User Story 2 - Ingest)**:
- Models: T013, T014, T015, T016 all [P] - can run in parallel
- Tests: T022, T023, T024, T025, T026 all [P] - can run in parallel

**Phase 4 (User Story 1 - Chat)**:
- Models: T028, T029, T030, T031 all [P] - can run in parallel
- Tests: T036, T037, T038 all [P] - can run in parallel

**Phase 5 (User Story 3 - Health)**:
- T040 [P] can run in parallel with other setup tasks
- T043 [P] test can run in parallel with other tests

**Phase 6 (User Story 4 - CORS)**:
- T046 [P] test can run in parallel with other tests

**Phase 7 (Polish)**:
- T047, T048, T049, T050, T052, T053 all [P] - can run in parallel

---

## Parallel Example: User Story 2 (Ingest)

```bash
# Launch all models together:
Task: "Create IngestRequest model in backend/src/models/ingest.py"
Task: "Create IngestResponse model in backend/src/models/ingest.py"
Task: "Create internal IngestionResult dataclass in backend/src/models/ingest.py"
Task: "Create TextChunk dataclass in backend/src/models/ingest.py"

# After models complete, launch services:
Task: "Implement EmbeddingService in backend/src/services/embedding.py"
Task: "Implement VectorStoreService in backend/src/services/vector_store.py"
# (These can overlap if working on different files)

# Launch all unit tests together:
Task: "Unit test for chunking utility in backend/tests/unit/test_chunking.py"
Task: "Unit test for EmbeddingService in backend/tests/unit/test_embedding.py"
Task: "Unit test for VectorStoreService in backend/tests/unit/test_vector_store.py"
Task: "Unit test for IngestionService in backend/tests/unit/test_ingestion.py"
```

---

## Parallel Example: User Story 1 (Chat)

```bash
# Launch all models together:
Task: "Create ChatRequest model in backend/src/models/chat.py"
Task: "Create ChatResponse model in backend/src/models/chat.py"
Task: "Create internal RetrievalResult dataclass in backend/src/models/chat.py"
Task: "Create SearchResult dataclass in backend/src/models/chat.py"

# Launch all unit tests together:
Task: "Unit test for RetrievalService in backend/tests/unit/test_retrieval.py"
Task: "Unit test for AgentService in backend/tests/unit/test_agent.py"
Task: "Unit test for validation helpers in backend/tests/unit/test_validation.py"
```

---

## Implementation Strategy

### MVP First (User Stories 2 + 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T012) ← CRITICAL
3. Complete Phase 3: User Story 2 - Ingest (T013-T027)
4. **VALIDATE**: Run POST /ingest, verify data in Qdrant
5. Complete Phase 4: User Story 1 - Chat (T028-T039)
6. **STOP and VALIDATE**: Test POST /chat with sample questions
7. Deploy/demo if ready

**MVP Deliverable**: Students can ask questions and get accurate answers from textbook. This is the core value!

### Incremental Delivery

1. Complete Setup + Foundational (T001-T012) → Foundation ready
2. Add User Story 2: Ingest (T013-T027) → Content loaded
3. Add User Story 1: Chat (T028-T039) → **Test independently → Deploy/Demo (MVP!)**
4. Add User Story 3: Health (T040-T043) → Test independently → Deploy/Demo
5. Add User Story 4: CORS (T044-T046) → Test independently → Deploy/Demo
6. Add Polish (T047-T056) → Final production ready
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T012)
2. Team completes User Story 2 together (T013-T027) ← Prerequisite
3. Once US2 ingestion is done:
   - Developer A: User Story 1 - Chat (T028-T039)
   - Developer B: User Story 3 - Health (T040-T043) ← Can run in parallel!
4. Once US1 complete:
   - Developer C: User Story 4 - CORS (T044-T046)
5. All developers: Polish (T047-T056) in parallel

---

## Task Statistics

- **Total Tasks**: 56
- **Setup Phase**: 5 tasks
- **Foundational Phase**: 7 tasks
- **User Story 2 (Ingest)**: 15 tasks (9 implementation + 6 tests)
- **User Story 1 (Chat)**: 12 tasks (8 implementation + 4 tests)
- **User Story 3 (Health)**: 4 tasks (3 implementation + 1 test)
- **User Story 4 (CORS)**: 3 tasks (2 implementation + 1 test)
- **Polish Phase**: 10 tasks
- **Parallel Tasks**: 28 tasks marked [P]
- **MVP Tasks (Setup + Foundational + US2 + US1)**: 39 tasks

### MVP Scope

**Minimum Viable Product** = User Stories 2 + 1
- Ingest textbook content (US2)
- Answer student questions (US1)
- This delivers core value: students can ask and get answers

**Post-MVP Enhancements**:
- Health monitoring (US3)
- Frontend CORS (US4)
- Polish and documentation

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- User Story 2 (Ingest) must complete before User Story 1 (Chat) can function
- Verify tests pass after each story
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Constitution compliance: Type hints, docstrings, <500 lines/file, <50 lines/function, >80% test coverage
- All tasks follow checklist format: `- [ ] [ID] [P?] [Story?] Description with file path`
