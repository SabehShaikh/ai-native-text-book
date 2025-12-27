---
id: 001
title: Create RAG Backend Constitution
stage: constitution
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: SabehShaikh
command: /sp.constitution
labels: ["constitution", "rag", "backend", "initialization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - None (constitution document)
---

## Prompt

/sp.constitution

Project Name: RAG Chatbot Backend

Purpose:
FastAPI backend providing RAG capabilities for the Physical AI textbook. Answers questions using ONLY textbook content.

Tech Stack:
- FastAPI (Python web framework)
- Gemini API (LLM responses)
- Cohere API (embeddings - embed-english-v3.0)
- Qdrant Cloud (vector database)
- Trafilatura (content extraction)
- OpenAI Agents SDK pattern

Architecture:
1. Ingestion: Sitemap → Extract → Chunk → Embed → Qdrant
2. Retrieval: Query → Embed → Search Qdrant → Top 5 chunks
3. Agent: User question → retrieve() tool → Gemini → Answer

Core Principles:
1. Accuracy: Answers ONLY from textbook content
2. Speed: Responses under 3 seconds
3. Simplicity: Direct implementation, minimal complexity
4. Free Tier: Works within Qdrant/Cohere limits
5. Secure: Environment variables for API keys

Endpoints:
- POST /chat: Query → Answer with sources
- POST /ingest: Trigger one-time content embedding
- GET /health: Status check

Key Features:
- Read from sitemap.xml
- Chunk text (1000 chars)
- Semantic search
- Source attribution
- CORS for Vercel frontend
- No authentication (MVP)
- No database for chat history (simplified)

Constraints:
- Free tier only
- Stateless API
- No GPU needed
- Fast deployment

Success Criteria:
- Ingestion under 5 minutes
- Chat responds under 3 seconds
- Accurate, sourced answers
- Zero hallucination
- Successful deployment

## Response snapshot

Created comprehensive constitution for RAG Chatbot Backend at `.specify/memory/constitution.md`

**Structure:**
- Project Overview with name, purpose, architecture pattern
- 6 Core Principles (Content Fidelity, Speed, Simplicity, Free Tier, Security, API Design)
- Technology Stack (required dependencies and forbidden items)
- Architecture Standards (3-phase pipeline, API endpoints, data models)
- Development Standards (code quality, testing, error handling, logging)
- Performance Standards (latency targets, resource limits, caching strategy)
- Deployment Standards (environment variables, targets, health checks)
- Operational Standards (monitoring, incident response, maintenance)
- Non-Functional Requirements (reliability, scalability, security)
- Constraints (MUST/MUST NOT/MAY have lists)
- Success Criteria (technical and business)
- Risk Mitigation (3 key risks with impact/mitigation/fallback)
- Governance (amendment process, compliance, review cadence)

**Key Highlights:**
- Zero hallucination tolerance (Content Fidelity)
- 3-second response target
- YAGNI and Simplicity First principles
- Free tier operations only
- Clear API contracts with data models
- Type hints and docstrings required
- 80%+ test coverage requirement
- Structured logging with request IDs
- Performance targets: /chat p95 < 3s, /health < 100ms
- 99.5% uptime target with circuit breakers
- Version 1.0.0 ratified 2025-12-18

## Outcome

- ✅ Impact: Established foundational project constitution defining all architectural, operational, and governance standards
- 🧪 Tests: None (constitution document, no code changes)
- 📁 Files: 1 file created (.specify/memory/constitution.md)
- 🔁 Next prompts: Create feature spec with /sp.specify, review and update constitution as needed, begin implementation planning
- 🧠 Reflection: Constitution provides clear technical and business constraints. Emphasis on Content Fidelity and Simplicity aligns with RAG chatbot requirements. Free tier constraint will require careful API usage monitoring.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - Constitution follows template structure, includes all required sections with specific measurable criteria
- Prompt variant (if applicable): Standard constitution creation flow
- Next experiment (smallest change to try): Monitor if 1000-char chunking is optimal for semantic search quality vs performance
