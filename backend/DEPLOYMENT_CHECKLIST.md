# Deployment Checklist

## Pre-Deployment
- [ ] All environment variables collected
- [ ] Database (Neon) is accessible
- [ ] Qdrant vector database is set up
- [ ] OpenRouter API key is valid
- [ ] Better Auth is configured on frontend

## Hugging Face Spaces Setup
- [ ] Created new Space with Docker SDK
- [ ] Uploaded all backend files
- [ ] Set all environment variables in Space settings
- [ ] Build completed successfully
- [ ] Space is running (check status)

## Testing
- [ ] Health endpoint works: `/health`
- [ ] API docs accessible: `/docs`
- [ ] Chat endpoint works: `/api/v1/chat`
- [ ] Auth endpoint works: `/api/v1/auth/health`
- [ ] RAG context retrieval working

## Frontend Integration
- [ ] Updated `env.ts` with HF Space URL
- [ ] Updated `.env.local` with HF Space URL
- [ ] Updated CORS_ORIGINS in HF Space settings
- [ ] Deployed frontend changes to Vercel
- [ ] Tested frontend → backend connection

## End-to-End Testing
- [ ] User can send chat messages
- [ ] Streaming responses work
- [ ] RAG context is being used
- [ ] Authentication flow works (if implemented)
- [ ] No CORS errors in browser console

## Post-Deployment
- [ ] Monitor logs for errors
- [ ] Check response times
- [ ] Verify database connections
- [ ] Test with multiple users (if applicable)

## Rollback Plan (if needed)
- [ ] Keep Render deployment URL as backup
- [ ] Document any issues encountered
- [ ] Have local development environment ready

---

## Quick Commands

### Test Health Endpoint
```bash
curl https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space/health
```

### Test Chat Endpoint
```bash
curl -X POST https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS?"}'
```

### View Logs
Go to: https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-humanoid-robotics-backend/logs

---

## Environment Variables Quick Reference

Copy these to Hugging Face Space Settings → Repository secrets:

```
SECRET_KEY=
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
DATABASE_URL=
DB_POOL_SIZE=10
DB_MAX_OVERFLOW=20
BETTER_AUTH_URL=https://neurobotics-ai-book.vercel.app
BETTER_AUTH_JWKS_URL=https://neurobotics-ai-book.vercel.app/api/auth/jwks
OPENROUTER_API_KEY=
OPENROUTER_MODEL=openrouter/free
OPENROUTER_FALLBACK_MODELS=nvidia/nemotron-3-ultra-550b-a55b:free
QDRANT_URL=
QDRANT_API_KEY=
QDRANT_COLLECTION_NAME=neurobotics-physical-ai-humanoid-robotics-book
EMBEDDING_MODEL=BAAI/bge-small-en-v1.5
RAG_SIMILARITY_THRESHOLD=0.6
APP_URL=https://neurobotics-ai-book.vercel.app
CORS_ORIGINS=http://localhost:3000,https://neurobotics-ai-book.vercel.app
ENVIRONMENT=production
```
