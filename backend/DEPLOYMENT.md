# Deployment Guide: Hugging Face Spaces

This guide covers deploying the RAG Chatbot backend to Hugging Face Spaces using Docker.

## Prerequisites

1. Hugging Face account (free tier works)
2. Git installed locally
3. Required API keys:
   - Gemini API key
   - Cohere API key
   - Qdrant Cloud URL and API key

## Deployment Steps

### 1. Create a New Space

1. Go to https://huggingface.co/spaces
2. Click "Create new Space"
3. Fill in the details:
   - **Name**: `rag-chatbot-api` (or your preferred name)
   - **License**: Apache 2.0 (or your choice)
   - **SDK**: Docker
   - **Visibility**: Public or Private
4. Click "Create Space"

### 2. Configure Environment Variables

In your Space's settings, add the following secrets:

```
GEMINI_API_KEY=your_gemini_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=ai-textbook
FRONTEND_URL=https://ai-native-text-book.vercel.app
SITEMAP_URL=https://ai-native-text-book.vercel.app/sitemap.xml
```

**Important Notes:**
- Go to Settings → Repository secrets
- Add each variable individually
- These are kept secure and won't be exposed in logs

### 3. Deploy via Git

#### Option A: Push from Local Repository

```bash
# Navigate to backend directory
cd backend

# Initialize git (if not already initialized)
git init

# Add the Hugging Face Space as remote
git remote add space https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME

# Add all files
git add Dockerfile .dockerignore requirements.txt src/ pytest.ini

# Commit
git commit -m "Initial deployment to Hugging Face Spaces"

# Push to Hugging Face
git push space main
```

#### Option B: Upload Files via Web UI

1. In your Space, click "Files" → "Add file" → "Upload files"
2. Upload these files/folders:
   - `Dockerfile`
   - `.dockerignore`
   - `requirements.txt`
   - `src/` (entire folder)
   - `pytest.ini`
3. Commit the changes

### 4. Monitor Deployment

1. After pushing, Hugging Face will automatically:
   - Build the Docker image
   - Install dependencies
   - Start the application on port 7860
2. Monitor build logs in the "Logs" tab
3. Build typically takes 3-5 minutes

### 5. Verify Deployment

Once deployed, your API will be available at:
```
https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space
```

Test endpoints:
- Root: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/`
- Health: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/health`
- Docs: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/docs`

### 6. Ingest Data

Once the API is running, trigger ingestion:

```bash
# Using curl
curl -X POST "https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/ingest" \
  -H "Content-Type: application/json"

# Or visit the interactive docs and use the Try it out feature
# https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/docs
```

**Note**: Ingestion will run on Hugging Face's infrastructure, avoiding your local PC resource constraints.

## File Structure

The deployment includes:

```
backend/
├── Dockerfile              # Container configuration
├── .dockerignore          # Files to exclude from build
├── requirements.txt       # Python dependencies
├── pytest.ini            # Test configuration
└── src/                  # Application source code
    ├── main.py           # FastAPI application
    ├── config.py         # Environment configuration
    ├── logger.py         # Logging setup
    ├── api/              # API endpoints
    ├── models/           # Pydantic models
    ├── services/         # Business logic
    └── utils/            # Utilities
```

## Troubleshooting

### Build Fails

1. Check build logs in the Spaces "Logs" tab
2. Verify all files are uploaded correctly
3. Ensure `requirements.txt` has all dependencies

### Application Won't Start

1. Check application logs
2. Verify environment variables are set correctly
3. Ensure port 7860 is exposed in Dockerfile

### API Requests Fail

1. Check CORS settings in `src/main.py`
2. Verify FRONTEND_URL environment variable
3. Check API logs for specific errors

### Ingestion Fails

1. Verify Qdrant credentials
2. Check Cohere API key and quota
3. Ensure sitemap URL is accessible
4. Monitor memory usage in Space settings

## Updating the Deployment

To deploy updates:

```bash
# Make your changes
git add .
git commit -m "Description of changes"
git push space main
```

Hugging Face will automatically rebuild and redeploy.

## Resource Limits

Hugging Face Spaces free tier provides:
- 2 CPU cores
- 16GB RAM
- 50GB storage

This should be sufficient for the RAG chatbot ingestion and query operations.

## Cost Considerations

- Hugging Face Spaces: Free tier available
- Qdrant Cloud: Has free tier (1GB storage)
- Cohere API: Free tier (100 API calls/minute)
- Gemini API: Free tier available

Monitor your usage to stay within free tier limits.

## Security Notes

1. Never commit `.env` files or API keys to the repository
2. Always use Hugging Face Secrets for sensitive data
3. Keep dependencies updated for security patches
4. Monitor API logs for unusual activity

## Next Steps

After deployment:
1. Update frontend to use the new backend URL
2. Test all endpoints thoroughly
3. Monitor performance and logs
4. Set up error alerting if needed

## Support

- Hugging Face Spaces docs: https://huggingface.co/docs/hub/spaces
- Issues: Check application logs and Space settings
- Community: Hugging Face forums and Discord
