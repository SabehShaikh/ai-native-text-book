# RAG Chatbot Backend Constitution

## Project Overview

**Name**: RAG Chatbot Backend
**Purpose**: FastAPI backend providing RAG capabilities for the Physical AI textbook. Answers questions using ONLY textbook content.
**Architecture Pattern**: OpenAI Agents SDK pattern with retrieval-augmented generation

## Core Principles

### I. Content Fidelity (CRITICAL)
**Answers MUST be sourced exclusively from textbook content**
- Zero hallucination tolerance
- Every answer must cite specific textbook sources
- If content not found in textbook, agent must explicitly state "I don't have information about this in the textbook"
- No general knowledge responses
- Source attribution required for all claims

### II. Speed & Performance
**3-Second Response Target**
- Chat endpoint: < 3 seconds p95
- Ingestion: < 5 minutes total
- Optimize embedding batch sizes
- Use async operations where possible
- Monitor and log slow queries

### III. Simplicity First
**Direct implementation, minimal complexity**
- No premature optimization
- No authentication (MVP phase)
- No chat history persistence (stateless)
- No complex caching layers
- Clear, readable code over clever abstractions
- YAGNI principle strictly enforced

### IV. Free Tier Operations
**Works within free tier limits**
- Qdrant Cloud free tier
- Cohere embed-english-v3.0 free tier
- Gemini API free tier
- No paid dependencies
- Monitor usage to stay within limits

### V. Security & Configuration
**Environment-based configuration**
- All API keys in environment variables
- No secrets in code
- No secrets in logs
- CORS configured for Vercel frontend only
- Input validation on all endpoints
- Rate limiting considerations

### VI. API Design
**RESTful, predictable, documented**
- Clear endpoint contracts
- JSON request/response
- Proper HTTP status codes
- Error messages include context
- OpenAPI documentation auto-generated

## Technology Stack

### Required Dependencies
- **FastAPI**: Web framework
- **Gemini API**: LLM for generation
- **Cohere API**: Embeddings (embed-english-v3.0)
- **Qdrant Cloud**: Vector database
- **Trafilatura**: Content extraction
- **Python 3.11+**: Runtime

### Forbidden
- No GPU dependencies
- No paid-only services
- No complex orchestration tools
- No unnecessary middleware

## Architecture Standards

### 1. Three-Phase Pipeline

**Phase 1: Ingestion (One-time)**
```
Sitemap → Extract → Chunk → Embed → Qdrant
```
- Read sitemap.xml
- Extract with Trafilatura
- Chunk at 1000 characters
- Embed with Cohere
- Store in Qdrant with metadata

**Phase 2: Retrieval (Per Query)**
```
Query → Embed → Search Qdrant → Top 5 chunks
```
- Embed user query
- Semantic search in Qdrant
- Return top 5 most relevant chunks
- Include source URLs

**Phase 3: Generation (Per Query)**
```
Question + Context → Gemini Agent → Answer
```
- Agent uses retrieve() tool
- Gemini generates answer
- Sources included in response

### 2. API Endpoints

**POST /chat**
- Input: `{"question": "string"}`
- Output: `{"answer": "string", "sources": ["url1", "url2"]}`
- Max response time: 3 seconds

**POST /ingest**
- Input: Optional `{"sitemap_url": "string"}`
- Output: `{"status": "success", "chunks_processed": int}`
- Admin-only trigger
- Idempotent operation

**GET /health**
- Output: `{"status": "healthy", "services": {...}}`
- Checks Qdrant, Cohere, Gemini connectivity

### 3. Data Model

**Chunk Storage (Qdrant)**
```python
{
  "id": "uuid",
  "vector": [float],  # 1024-dim from Cohere
  "payload": {
    "text": "string",
    "url": "string",
    "title": "string",
    "chunk_index": int
  }
}
```

**Chat Request**
```python
{
  "question": str,  # Required, 1-500 chars
}
```

**Chat Response**
```python
{
  "answer": str,
  "sources": List[str],  # URLs
  "confidence": float,  # 0-1, optional
}
```

## Development Standards

### Code Quality
- Type hints required (Python)
- Docstrings for all public functions
- Max function length: 50 lines
- Max file length: 500 lines
- No complex inheritance
- Prefer composition over inheritance

### Testing Requirements
- Unit tests for business logic
- Integration tests for API endpoints
- Test coverage > 80%
- Mock external APIs in tests
- Test error paths explicitly

### Error Handling
- Never expose internal errors to users
- Log all errors with context
- Return user-friendly messages
- Include request IDs for tracing
- Distinguish client (4xx) vs server (5xx) errors

### Logging Standards
```python
# Required fields
{
  "timestamp": "ISO8601",
  "level": "INFO|WARN|ERROR",
  "message": "string",
  "request_id": "uuid",
  "endpoint": "string",
  "duration_ms": float
}
```

## Performance Standards

### Latency Targets
- /chat: p50 < 1s, p95 < 3s
- /health: p95 < 100ms
- /ingest: total < 5 minutes

### Resource Limits
- Max request size: 1MB
- Max response size: 5MB
- Max concurrent requests: 100
- Memory limit: 512MB

### Caching Strategy
- No caching in MVP
- Future: Cache embeddings for common queries
- Future: Cache Qdrant results for 5 minutes

## Deployment Standards

### Environment Variables Required
```
GEMINI_API_KEY=xxx
COHERE_API_KEY=xxx
QDRANT_URL=xxx
QDRANT_API_KEY=xxx
ALLOWED_ORIGINS=https://your-frontend.vercel.app
```

### Deployment Targets
- Vercel (primary)
- Railway (alternative)
- Any Python 3.11+ host

### Health Checks
- Liveness: GET /health
- Readiness: Qdrant connectivity check
- Startup: Load environment, verify APIs

## Operational Standards

### Monitoring
- Request count per endpoint
- Response latency percentiles
- Error rate by type
- External API availability
- Qdrant query performance

### Incident Response
1. Check /health endpoint
2. Verify external API keys
3. Check Qdrant connectivity
4. Review recent logs
5. Check rate limits

### Maintenance
- Weekly: Review error logs
- Monthly: Check API usage vs limits
- Quarterly: Review and prune unused code

## Non-Functional Requirements

### Reliability
- Uptime: 99.5% target
- Graceful degradation: Return cached results if Gemini unavailable
- Retry logic: 3 attempts with exponential backoff
- Circuit breakers for external APIs

### Scalability
- Horizontal scaling supported (stateless)
- No session state
- Connection pooling for Qdrant
- Async I/O throughout

### Security
- Input sanitization on all endpoints
- SQL injection N/A (no SQL)
- XSS protection in responses
- Rate limiting: 100 req/min per IP
- No PII stored

## Constraints

### MUST Have
- Free tier only
- Stateless API
- No GPU
- Fast deployment (< 10 minutes)
- Zero hallucination

### MUST NOT Have
- User authentication (MVP)
- Chat history persistence
- Complex caching
- Paid dependencies
- GPU requirements

### MAY Have (Future)
- Query result caching
- Usage analytics
- Admin dashboard
- Batch processing

## Success Criteria

### Technical Success
- ✅ Ingestion completes in < 5 minutes
- ✅ Chat responses in < 3 seconds
- ✅ Accurate answers with sources
- ✅ Zero hallucination
- ✅ Successful deployment to Vercel

### Business Success
- ✅ Users can ask questions about textbook
- ✅ Answers are accurate and sourced
- ✅ Fast enough for interactive use
- ✅ Runs on free tier
- ✅ Easy to maintain

## Risk Mitigation

### Risk 1: API Rate Limits
- **Impact**: Service degradation
- **Mitigation**: Monitor usage, implement backoff, cache results
- **Fallback**: Return cached answers or queue requests

### Risk 2: Inaccurate Answers
- **Impact**: User trust loss
- **Mitigation**: Strict prompt engineering, source validation
- **Fallback**: "I don't know" responses when uncertain

### Risk 3: Slow Responses
- **Impact**: Poor UX
- **Mitigation**: Optimize chunk sizes, parallel processing, monitoring
- **Fallback**: Return partial results with timeout indicator

## Governance

This constitution supersedes all other development practices and guidelines for the RAG Chatbot Backend project.

### Amendment Process
1. Propose change with justification
2. Document impact analysis
3. Update constitution
4. Create migration plan if needed
5. Update all dependent documentation

### Compliance
- All PRs must verify constitutional compliance
- Complexity must be justified against Simplicity First principle
- Security violations are immediate blockers
- Performance regressions require explanation

### Review Cadence
- Weekly: Check performance metrics against targets
- Monthly: Review error patterns and architecture
- Quarterly: Full constitutional review and amendments

**Version**: 1.0.0
**Ratified**: 2025-12-18
**Last Amended**: 2025-12-18
