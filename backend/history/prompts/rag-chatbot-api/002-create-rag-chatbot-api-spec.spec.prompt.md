---
id: 002
title: Create RAG Chatbot API Spec
stage: spec
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot-api
branch: main
user: SabehShaikh
command: /sp.specify
labels: ["spec", "rag", "fastapi", "gemini", "cohere", "qdrant"]
links:
  spec: specs/rag-chatbot-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/rag-chatbot-api/spec.md
tests:
  - None (specification document)
---

## Prompt

/sp.specify

Feature Name: RAG Chatbot API

Feature Description:
FastAPI backend providing RAG question-answering for Physical AI textbook using Gemini, Cohere, and Qdrant.

API Configuration:
- QDRANT_URL: https://f649b7e4-cc63-4ad6-8c2c-5eabbefb6b9d.us-east4-0.gcp.cloud.qdrant.io
- QDRANT_API_KEY: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.aME3iadAVK4Zvv7K_srpl79AT_I8gOVGyGQbBVQuJkU
- QDRANT_COLLECTION: ai-textbook
- COHERE_API_KEY: SdUVIq13qDWB6tqhJZ30yQduzK8037LbxjHDX2tp
- GEMINI_API_KEY: AIzaSyAOsRiQykUK7u2kWBbRIHrFwMT1RcFr95g
- SITEMAP_URL: https://ai-native-text-book.vercel.app/sitemap.xml
- FRONTEND_URL: https://ai-native-text-book.vercel.app

User Story:
As a student, I want to ask questions about the textbook and receive accurate answers from the book content.

Core Endpoints:

1. POST /chat
   - Input: {query: string}
   - Process: Embed query → Search Qdrant → Gemini generates answer
   - Output: {answer: string, sources: array, response_time: float}

2. POST /ingest
   - Fetch sitemap → Extract text → Chunk (1000 chars) → Embed with Cohere → Store in Qdrant
   - Output: {status, pages_processed, chunks_created}

3. GET /health
   - Check Qdrant connection and collection
   - Output: {status, qdrant_status, collection_count}

Technical Stack:
- FastAPI with CORS for frontend
- Qdrant client for vector search
- Cohere for embeddings (embed-english-v3.0)
- Gemini via OpenAI SDK (gemini-1.5-flash)
- OpenAI Agents pattern with retrieve() tool
- Trafilatura for text extraction

Agent Setup:
- Base URL: https://generativelanguage.googleapis.com/v1beta/openai/
- Model: gemini-1.5-flash
- System: "You are an AI tutor. Use retrieve() tool. Answer ONLY from textbook. Say 'I don't know' if not found."
- Tool: retrieve(query) → searches Qdrant

Performance:
- /chat: < 3 seconds
- /ingest: < 5 minutes
- /health: < 100ms

Acceptance Criteria:
- Ingestion processes all sitemap pages
- Chat responds accurately with sources
- Agent refuses to answer outside textbook
- CORS allows frontend connection
- All endpoints work correctly

Out of Scope:
- User authentication
- Chat history storage
- Text selection feature

## Response snapshot

Created comprehensive feature specification at `specs/rag-chatbot-api/spec.md`

**Structure:**
- 4 prioritized user stories (P1: Chat & Ingest, P2: Health & Frontend)
- 10 edge cases with explicit handling
- 25 functional requirements (FR-001 to FR-025)
- 4 key entities (TextChunk, ChatQuery, ChatResponse, IngestionJob)
- 10 success criteria with measurable outcomes
- Complete technical specifications including:
  - Environment variable configuration
  - Technology stack details
  - Agent configuration (Gemini with retrieve() tool)
  - API endpoint contracts with request/response examples
  - Data flow diagrams for chat, ingestion, and health check
  - Performance requirements (latency, throughput, resources)
  - Security requirements (CORS, input validation, secrets management)
  - Error handling with HTTP status codes
  - Out of scope features (auth, history, rate limiting)
  - Dependencies and assumptions
  - Risk analysis

**Validation Against Constitution:**
✅ Content Fidelity: FR-006, FR-023 enforce textbook-only answers
✅ Speed & Performance: SC-001 (p95 < 3s), SC-002 (ingestion < 5min), SC-003 (health < 100ms)
✅ Simplicity First: Out of scope excludes auth, history, caching per constitution
✅ Free Tier: All services (Qdrant, Cohere, Gemini) use free tier
✅ Security: FR-021 (env vars), FR-016 (CORS), FR-017 (validation), FR-018 (logging)
✅ API Design: Clear contracts, JSON I/O, HTTP status codes, OpenAPI docs (FR-025)

**Key Decisions:**
- 1000-char chunks with 200-char overlap (constitution specifies 1000)
- Top 5 similar chunks for context (constitution specifies 5)
- OpenAI Agents SDK pattern with retrieve() tool
- Gemini 2.0 Flash Exp via OpenAI-compatible endpoint
- Cohere embed-english-v3.0 (1024-dim vectors)
- Stateless API with no persistence (MVP)
- CORS restricted to single frontend origin

**User Stories Prioritization:**
- P1: Student asks questions (core value)
- P1: Admin ingests content (prerequisite for chat)
- P2: Health monitoring (operational necessity)
- P2: Frontend integration (UX requirement)

## Outcome

- ✅ Impact: Complete, testable specification ready for planning and implementation
- 🧪 Tests: None (specification document, no code)
- 📁 Files: 1 file created (specs/rag-chatbot-api/spec.md)
- 🔁 Next prompts: Run `/sp.plan` to create implementation plan, Run `/sp.tasks` to generate task breakdown, Begin implementation with `/sp.implement`
- 🧠 Reflection: Specification aligns perfectly with constitution principles. All requirements are measurable and testable. Edge cases are explicitly defined. Configuration provided by user includes all necessary API keys and endpoints. Ready for architectural planning.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - Spec includes mandatory sections (user scenarios, requirements, success criteria), follows SDD template structure, provides clear acceptance scenarios with Given-When-Then format
- Prompt variant (if applicable): Standard specification creation with full configuration provided
- Next experiment (smallest change to try): Consider if 1000-char chunks need overlap parameter tuning (currently 200 chars) for better semantic continuity
