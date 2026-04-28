# Hugging Face Spaces Deployment Guide

## Complete Step-by-Step Deployment Process

### Prerequisites
- Hugging Face account (free)
- All environment variables ready (DATABASE_URL, OPENROUTER_API_KEY, QDRANT credentials)

---

## Step 1: Create Hugging Face Space

1. Go to https://huggingface.co/new-space
2. Fill in details:
   - **Space name**: `physical-ai-humanoid-robotics-backend`
   - **License**: MIT
   - **Select SDK**: Docker
   - **Space hardware**: CPU basic (free tier)
   - **Visibility**: Public (or Private if you prefer)
3. Click "Create Space"

---

## Step 2: Upload Backend Files

### Option A: Using Git (Recommended)

```bash
# Navigate to backend directory
cd D:\physical-ai-humanoid-robotics-book\backend

# Initialize git if not already done
git init

# Add Hugging Face Space as remote
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-humanoid-robotics-backend

# Add all files
git add .

# Commit
git commit -m "Initial deployment to Hugging Face Spaces"

# Push to Hugging Face
git push hf main
```

### Option B: Using Web Interface

1. In your Space, click "Files" tab
2. Click "Add file" → "Upload files"
3. Upload these files/folders:
   - `Dockerfile`
   - `requirements.txt`
   - `README.md`
   - `.dockerignore`
   - `rag.py`
   - `src/` (entire folder with all subfolders)

---

## Step 3: Configure Environment Variables

1. In your Space, go to **Settings** tab
2. Scroll to **Repository secrets**
3. Add each variable one by one:

### Required Environment Variables:

```bash
# Security
SECRET_KEY=your-super-secret-key-here-make-it-long-and-random
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Database (Neon PostgreSQL)
DATABASE_URL=postgresql://neondb_owner:npg_X7Bw9SqJxKMF@ep-round-frost-adal7cl8-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require
DB_POOL_SIZE=10
DB_MAX_OVERFLOW=20
DB_POOL_RECYCLE=300

# Better Auth
BETTER_AUTH_URL=https://neurobotics-ai-book.vercel.app
BETTER_AUTH_JWKS_URL=https://neurobotics-ai-book.vercel.app/api/auth/jwks

# OpenRouter
OPENROUTER_API_KEY=YOUR_OPENROUTER_KEY
OPENROUTER_MODEL=mistralai/mistral-small-3.1-24b-instruct

# Qdrant Vector Database
QDRANT_URL=YOUR_QDRANT_URL
QDRANT_API_KEY=YOUR_QDRANT_KEY
QDRANT_COLLECTION_NAME=neurobotics-physical-ai-humanoid-robotics-book

# RAG Configuration
EMBEDDING_MODEL=BAAI/bge-small-en-v1.5
RAG_SIMILARITY_THRESHOLD=0.6

# App Configuration
APP_URL=https://neurobotics-ai-book.vercel.app
CORS_ORIGINS=http://localhost:3000,https://neurobotics-ai-book.vercel.app
ENVIRONMENT=production
```

**Important**: Replace placeholder values with your actual credentials!

---

## Step 4: Wait for Build

1. Hugging Face will automatically start building your Docker container
2. Check the "Logs" tab to monitor build progress
3. Build typically takes 3-5 minutes
4. Once complete, you'll see "Running" status

---

## Step 5: Test Your Deployment

Your backend will be available at:
```
https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space
```

Test endpoints:
```bash
# Health check
curl https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space/health

# API docs
https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space/docs
```

---

## Step 6: Update Frontend Configuration

Update your frontend to use the new Hugging Face backend URL:

### File: `physical-ai-humanoid-robotics/src/config/env.ts`

```typescript
// Production URLs (hardcoded)
const PRODUCTION_CONFIG = {
  AUTH_SERVER_URL: '',
  BACKEND_API_URL: 'https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space',
};
```

### File: `physical-ai-humanoid-robotics/.env.local`

```bash
BACKEND_API_URL=https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space
```

---

## Step 7: Update CORS Origins

After deployment, update the CORS_ORIGINS environment variable in Hugging Face Space settings to include your HF Space URL:

```bash
CORS_ORIGINS=http://localhost:3000,https://neurobotics-ai-book.vercel.app,https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space
```

---

## Troubleshooting

### Build Fails
- Check "Logs" tab for error messages
- Verify all files are uploaded correctly
- Ensure requirements.txt has all dependencies

### 500 Internal Server Error
- Check environment variables are set correctly
- Verify DATABASE_URL is accessible from Hugging Face
- Check QDRANT_URL and API key are valid

### CORS Errors
- Update CORS_ORIGINS to include your frontend URL
- Restart the Space after updating environment variables

### Port Issues
- Hugging Face Spaces uses port 7860 by default
- Our Dockerfile is configured correctly for this

---

## Monitoring & Logs

- **Logs Tab**: View real-time application logs
- **Settings → Logs**: Download historical logs
- **API Docs**: Test endpoints at `/docs`

---

## Cost Considerations

- **Free Tier**: CPU basic (sufficient for development/testing)
- **Paid Tiers**: If you need more resources:
  - CPU upgrade: $0.03/hour
  - GPU: Starting at $0.60/hour

---

## Next Steps After Deployment

1. ✅ Test all API endpoints
2. ✅ Verify RAG chatbot works
3. ✅ Test authentication flow
4. ✅ Update frontend to use new backend URL
5. ✅ Deploy frontend changes to Vercel
6. ✅ Test end-to-end flow

---

## Support

If you encounter issues:
1. Check Hugging Face Spaces documentation: https://huggingface.co/docs/hub/spaces
2. Review application logs in the Logs tab
3. Test locally first with Docker: `docker build -t backend . && docker run -p 7860:7860 backend`

---

## Author
Muskan Atiq - Fullstack Agentic AI Engineer
