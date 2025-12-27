# RAG Chatbot API

A FastAPI-based Retrieval-Augmented Generation (RAG) chatbot API for answering questions about textbook content using Gemini and Cohere.

## Features

- Ingest textbook content from sitemap
- Answer student questions with sourced responses
- Vector search using Qdrant
- Embeddings via Cohere
- LLM responses via Gemini 2.0 Flash
- Health monitoring endpoint
- CORS support for frontend integration

## Prerequisites

- Python 3.9+
- Gemini API key
- Cohere API key
- Qdrant instance (cloud or local)

## Setup Instructions

### 1. Clone and Navigate

```bash
cd backend
```

### 2. Create Virtual Environment

```bash
python -m venv venv

# On Windows
venv\Scripts\activate

# On Linux/Mac
source venv/bin/activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Copy the example environment file and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env` with your actual values:

```env
GEMINI_API_KEY=your_gemini_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=textbook_chunks
FRONTEND_URL=https://ai-native-text-book.vercel.app
SITEMAP_URL=https://ai-native-text-book.vercel.app/sitemap.xml
```

### 5. Run the Application

```bash
uvicorn src.main:app --reload
```

The API will be available at `http://localhost:8000`

## API Documentation

### Interactive API Docs

Once the server is running, visit:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

### Endpoints

#### POST /chat

Ask a question about the textbook content.

**Request:**
```json
{
  "query": "What is machine learning?"
}
```

**Response:**
```json
{
  "answer": "Machine learning is...",
  "sources": [
    "https://ai-native-text-book.vercel.app/week1/intro"
  ],
  "response_time": 1.23,
  "request_id": "abc-123-def"
}
```

**Status Codes:**
- `200`: Success
- `400`: Invalid request (empty query, too long)
- `503`: External API failure
- `504`: Timeout

#### POST /ingest

Ingest textbook content from sitemap (admin only).

**Request:**
```json
{
  "sitemap_url": "https://ai-native-text-book.vercel.app/sitemap.xml"
}
```

**Response:**
```json
{
  "status": "success",
  "pages_processed": 42,
  "chunks_created": 350,
  "duration_seconds": 180.5,
  "errors": []
}
```

**Status Codes:**
- `200`: Success
- `409`: Ingestion already in progress
- `400`: Invalid sitemap URL

#### GET /health

Check API and Qdrant health status.

**Response:**
```json
{
  "status": "healthy",
  "qdrant_status": "connected",
  "collection_exists": true,
  "collection_count": 350,
  "timestamp": "2025-01-15T10:30:00Z"
}
```

**Status Codes:**
- `200`: Always returns 200, check `status` field

## Project Structure

```
backend/
├── src/
│   ├── models/          # Pydantic models
│   ├── services/        # Business logic
│   ├── api/             # API endpoints
│   ├── utils/           # Utilities
│   ├── config.py        # Configuration
│   ├── logger.py        # Logging
│   └── main.py          # FastAPI app
├── tests/
│   ├── unit/            # Unit tests
│   └── integration/     # Integration tests
├── requirements.txt     # Dependencies
├── .env.example         # Environment template
├── .gitignore           # Git ignore rules
└── README.md            # This file
```

## Development

### Run Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=term-missing

# Run specific test file
pytest tests/unit/test_chunking.py
```

### Code Quality

This project follows strict code quality standards:
- Type hints on all functions
- Docstrings for public functions
- Maximum file length: 500 lines
- Maximum function length: 50 lines
- Test coverage: >80%

## Deployment

### Deploy to Vercel

This backend is configured for deployment on Vercel.

#### Prerequisites

1. Vercel account
2. Vercel CLI installed: `npm install -g vercel`
3. All API keys ready (Gemini, Cohere, Qdrant)

#### Deployment Steps

1. **Install Vercel CLI** (if not already installed):

```bash
npm install -g vercel
```

2. **Login to Vercel**:

```bash
vercel login
```

3. **Set Environment Variables in Vercel**:

Go to your Vercel project settings and add these environment variables:

```
GEMINI_API_KEY=your_gemini_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION=textbook_chunks
FRONTEND_URL=https://your-frontend.vercel.app
SITEMAP_URL=https://your-frontend.vercel.app/sitemap.xml
```

Or use the Vercel CLI:

```bash
vercel env add GEMINI_API_KEY
vercel env add COHERE_API_KEY
vercel env add QDRANT_URL
vercel env add QDRANT_API_KEY
vercel env add QDRANT_COLLECTION
vercel env add FRONTEND_URL
vercel env add SITEMAP_URL
```

4. **Deploy**:

From the `backend/` directory:

```bash
vercel
```

For production deployment:

```bash
vercel --prod
```

5. **Post-Deployment**:

- Test the health endpoint: `https://your-backend.vercel.app/health`
- Run ingestion: `POST https://your-backend.vercel.app/ingest`
- Update frontend with backend URL

#### Vercel Configuration

The `vercel.json` file is already configured with:
- Python runtime
- Route configuration
- Environment variable references

#### Important Notes

- First deployment will take longer as dependencies are installed
- Vercel serverless functions have a 10-second execution limit on Hobby plan
- For long-running ingestion, consider upgrading to Pro plan or using a separate service
- Monitor logs: `vercel logs`

## Troubleshooting

### Common Issues

**Issue: Import errors**
- Ensure virtual environment is activated
- Run `pip install -r requirements.txt`

**Issue: API connection errors**
- Check your API keys in `.env`
- Verify Qdrant instance is running
- Check network connectivity

**Issue: CORS errors**
- Verify `FRONTEND_URL` matches your frontend domain
- Check browser console for specific errors

## License

See main project LICENSE file.

## Support

For issues and questions, please refer to the main project repository.
