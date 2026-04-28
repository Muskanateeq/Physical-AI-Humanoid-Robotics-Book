---
title: Physical AI Humanoid Robotics Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics Backend API

Backend API for Physical AI and Humanoid Robotics Book Platform with RAG-powered chatbot.

## Features

- 🚀 FastAPI backend with streaming chat responses
- 🧠 RAG (Retrieval-Augmented Generation) using Qdrant vector database
- 🤖 OpenRouter LLM integration (Mistral, Claude, GPT models)
- 🔐 Better Auth JWT authentication via JWKS
- 📊 PostgreSQL database (Neon) with async support
- 🌐 CORS enabled for frontend integration

## Tech Stack

- **Framework**: FastAPI + Uvicorn
- **Database**: PostgreSQL (Neon) with SQLModel + asyncpg
- **Vector DB**: Qdrant Cloud
- **Embeddings**: FastEmbed (BAAI/bge-small-en-v1.5)
- **LLM**: OpenRouter API
- **Auth**: Better Auth (JWT/JWKS)

## API Endpoints

### Core Endpoints
- `GET /` - Root endpoint with API info
- `GET /health` - Health check
- `GET /docs` - Interactive API documentation (Swagger UI)
- `GET /redoc` - Alternative API documentation (ReDoc)

### Authentication
- `GET /api/v1/auth/me` - Get current user profile (requires JWT)
- `GET /api/v1/auth/health` - Auth system health check

### Chat (RAG-powered)
- `POST /api/v1/chat` - Chat with RAG context (streaming response)
- `GET /api/v1/chat/health` - Chat service health check

## Environment Variables

Configure these in Hugging Face Spaces Settings → Repository secrets:

```bash
# Security
SECRET_KEY=your-super-secret-key-here-make-it-long-and-random
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Database (Neon PostgreSQL)
DATABASE_URL=postgresql://username:password@host.neon.tech/dbname?sslmode=require
DB_POOL_SIZE=10
DB_MAX_OVERFLOW=20

# Better Auth (Frontend)
BETTER_AUTH_URL=https://neurobotics-ai-book.vercel.app
BETTER_AUTH_JWKS_URL=https://neurobotics-ai-book.vercel.app/api/auth/jwks

# OpenRouter LLM
OPENROUTER_API_KEY=your-openrouter-api-key-here
OPENROUTER_MODEL=mistralai/mistral-small-3.1-24b-instruct

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=neurobotics-physical-ai-humanoid-robotics-book

# RAG Configuration
EMBEDDING_MODEL=BAAI/bge-small-en-v1.5
RAG_SIMILARITY_THRESHOLD=0.6

# App Configuration
APP_URL=https://neurobotics-ai-book.vercel.app
CORS_ORIGINS=http://localhost:3000,https://neurobotics-ai-book.vercel.app
ENVIRONMENT=production
```

## Deployment on Hugging Face Spaces

### Step 1: Create New Space
1. Go to https://huggingface.co/spaces
2. Click "Create new Space"
3. Name: `physical-ai-humanoid-robotics-backend`
4. SDK: **Docker**
5. Visibility: Public or Private

### Step 2: Upload Files
Upload these files to your Space:
- `Dockerfile`
- `requirements.txt`
- `README.md` (this file)
- `rag.py`
- `src/` directory (all files)

### Step 3: Configure Environment Variables
In Space Settings → Repository secrets, add all environment variables listed above.

### Step 4: Deploy
Hugging Face will automatically build and deploy your Docker container.

### Step 5: Update Frontend
Update your frontend `env.ts` to use the Hugging Face Space URL:
```typescript
BACKEND_API_URL: 'https://your-username-physical-ai-humanoid-robotics-backend.hf.space'
```

## Local Development

```bash
# Install dependencies
pip install -r requirements.txt

# Create .env file
cp .env.example .env
# Edit .env with your credentials

# Run server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

## Author

**Muskan Atiq**
- Fullstack Agentic AI Engineer
- AI & Data Science Expert
- LinkedIn: https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert

## License

MIT License
