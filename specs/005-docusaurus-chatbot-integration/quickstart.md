# Quickstart: Docusaurus Chatbot Integration

## Overview
This guide provides step-by-step instructions to set up and deploy the Docusaurus Chatbot Integration with RAG functionality. The system uses the approved tech stack: @openai/chatkit-react for frontend, chatkit-python with FastAPI for backend, BetterAuth for authentication, Neon PostgreSQL for data storage, Qdrant Cloud for vector storage, FastEmbed for embeddings, and Google Gemini for responses.

## Prerequisites

### System Requirements
- Python 3.10 (required for chatkit-python compatibility)
- Node.js 18+ and npm/yarn
- Git
- Docker (optional, for containerized deployment)

### External Services
- Neon Serverless PostgreSQL account
- Qdrant Cloud account
- Google Gemini API key
- BetterAuth account (for authentication)

## Local Development Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd physical-ai-book
```

### 2. Backend Setup (FastAPI + ChatKit Python)

#### Navigate to Backend Directory
```bash
cd backend
```

#### Create Python Virtual Environment
```bash
python3.10 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

#### Install Dependencies
```bash
pip install -r requirements.txt
```

#### Set Up Environment Variables
Create a `.env` file in the backend directory with the following:

```env
# Database
NEON_DATABASE_URL=your_neon_postgres_connection_string

# Qdrant Vector Database
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key

# Google Gemini
GEMINI_API_KEY=your_gemini_api_key

# BetterAuth
BETTER_AUTH_SECRET=your_auth_secret
BETTER_AUTH_URL=http://localhost:8000

# Application
API_HOST=0.0.0.0
API_PORT=8000
DEBUG=true
```

#### Run Database Migrations
```bash
alembic upgrade head
```

#### Start the Backend Server
```bash
python -m src.main
```

The backend API will be available at `http://localhost:8000`.

### 3. Docusaurus Frontend Setup

#### Navigate to Docusaurus Frontend Directory
```bash
cd docusaurus-frontend
```

#### Install Dependencies
```bash
npm install
```

#### Set Up Environment Variables
Create a `.env` file in the docusaurus-frontend directory with the following:

```env
# API Configuration
REACT_APP_API_BASE_URL=http://localhost:8000
REACT_APP_CHATKIT_API_URL=http://localhost:8000/chatkit

# BetterAuth Configuration
REACT_APP_BETTER_AUTH_URL=http://localhost:8000
```

#### Start the Docusaurus Development Server
```bash
npm run start
```

The Docusaurus frontend will be available at `http://localhost:3000`.

### 4. Verify the Setup

1. Open your browser and navigate to `http://localhost:3000`
2. You should see the Docusaurus site with the integrated chatbot
3. Try the following:
   - Register a new account using the two-step process
   - Complete the onboarding questionnaire
   - Use the chatbot to ask questions about book content
   - Test both Selected-Text RAG Mode and Standard RAG Mode
   - Toggle between English and Urdu languages

## Production Deployment

### Backend Deployment

#### Railway Deployment
1. Install Railway CLI: `npm install -g @railway/cli`
2. Login: `railway login`
3. Navigate to backend directory
4. Link to your Railway project: `railway link <project-id>`
5. Set environment variables in Railway dashboard
6. Deploy: `railway up`

#### Vercel Deployment
1. Install Vercel CLI: `npm install -g vercel`
2. Login: `vercel login`
3. Navigate to backend directory
4. Deploy: `vercel --platform=python`

#### Fly.io Deployment
1. Install Fly CLI: `install-flyctl`
2. Login: `fly auth login`
3. Navigate to backend directory
4. Initialize: `fly launch`
5. Deploy: `fly deploy`

### Docusaurus Frontend Deployment

#### GitHub Pages Deployment
1. Update `docusaurus.config.js` with your deployment URL
2. Build the site: `npm run build`
3. Deploy to GitHub Pages: `npm run deploy`

#### Vercel Deployment
1. Install Vercel CLI: `npm install -g vercel`
2. Login: `vercel login`
3. Navigate to docusaurus-frontend directory
4. Deploy: `vercel`

#### Netlify Deployment
1. Install Netlify CLI: `npm install -g netlify-cli`
2. Login: `netlify login`
3. Navigate to docusaurus-frontend directory
4. Deploy: `netlify deploy --prod`

## Configuration Guide

### Environment Variables

#### Backend Environment Variables
| Variable | Description | Required |
|----------|-------------|----------|
| `NEON_DATABASE_URL` | Neon PostgreSQL connection string | Yes |
| `QDRANT_URL` | Qdrant Cloud URL | Yes |
| `QDRANT_API_KEY` | Qdrant API key | Yes |
| `GEMINI_API_KEY` | Google Gemini API key | Yes |
| `BETTER_AUTH_SECRET` | BetterAuth secret key | Yes |
| `BETTER_AUTH_URL` | BetterAuth base URL | Yes |
| `API_HOST` | Host for the API server | No (default: 0.0.0.0) |
| `API_PORT` | Port for the API server | No (default: 8000) |
| `DEBUG` | Enable debug mode | No (default: false) |

#### Frontend Environment Variables
| Variable | Description | Required |
|----------|-------------|----------|
| `REACT_APP_API_BASE_URL` | Base URL for backend API | Yes |
| `REACT_APP_CHATKIT_API_URL` | ChatKit API URL | Yes |
| `REACT_APP_BETTER_AUTH_URL` | BetterAuth base URL | Yes |

### Database Migrations
The application uses Alembic for database migrations:

```bash
# Generate a new migration
alembic revision --autogenerate -m "Description of changes"

# Apply migrations
alembic upgrade head

# Downgrade migrations
alembic downgrade -1
```

### API Endpoints

#### Authentication Endpoints
- `POST /auth/register` - Register new user (Step 1)
- `POST /auth/onboarding` - Complete onboarding (Step 2)
- `POST /auth/login` - User login
- `GET /auth/me` - Get current user profile

#### RAG Chatbot Endpoints
- `POST /api/rag-chat` - Main RAG chat endpoint
- `GET /api/history` - Get conversation history
- `POST /api/feedback` - Submit user feedback
- `GET /api/health` - Health check endpoint

#### Personalization Endpoints
- `POST /api/personalize` - Personalize chapter content
- `GET /api/personalization-history` - Get personalization history

#### Translation Endpoints
- `POST /api/translate` - Translate chapter content
- `POST /api/chatbot-translate` - Translate chatbot responses

## Running Tests

### Backend Tests
```bash
# Run all tests
python -m pytest

# Run unit tests
python -m pytest tests/unit/

# Run integration tests
python -m pytest tests/integration/

# Run with coverage
python -m pytest --cov=src/
```

### Frontend Tests
```bash
# Run all tests
npm test

# Run unit tests
npm run test:unit

# Run integration tests
npm run test:integration
```

## Troubleshooting

### Common Issues

#### Database Connection Issues
- Verify your Neon PostgreSQL connection string is correct
- Ensure your database allows connections from your deployment environment
- Check that your firewall allows database connections

#### API Key Issues
- Verify all API keys are correctly set in environment variables
- Ensure API keys have the necessary permissions
- Check rate limits on your API providers

#### Authentication Issues
- Ensure BetterAuth is properly configured
- Verify that user onboarding is completed before accessing protected features
- Check that session management is working correctly

#### Chatbot Not Responding
- Verify the Gemini API key is working
- Check that Qdrant Cloud is accessible
- Ensure the RAG pipeline is functioning properly

### Development Tips

#### Hot Reloading
- Backend: Use `python -m src.main --reload` for auto-reload during development
- Frontend: Docusaurus has built-in hot reloading with `npm run start`

#### Debugging the RAG Pipeline
- Enable debug mode with `DEBUG=true`
- Check the logs for detailed information about the RAG process
- Verify that embeddings are being generated correctly

#### Testing Different RAG Modes
- Selected-Text Mode: Highlight text in a chapter and ask a question
- Standard RAG Mode: Ask a general question without highlighting text

## Security Best Practices

### API Key Management
- Never commit API keys to version control
- Use environment variables for all sensitive credentials
- Rotate API keys regularly
- Use different keys for development and production

### Database Security
- Enable encryption at rest and in transit
- Use connection pooling for production
- Implement proper access controls
- Regularly backup your database

### Authentication Security
- Use HTTPS in production
- Implement proper session management
- Validate all user inputs
- Use rate limiting to prevent abuse

## Performance Optimization

### Caching Strategy
- Implement multi-level caching (CDN, API responses, database queries)
- Use Redis for session storage in production
- Cache frequently accessed embeddings

### Database Optimization
- Use proper indexing for frequently queried fields
- Implement connection pooling
- Optimize queries for performance

### Vector Database Optimization
- Tune Qdrant search parameters for optimal performance
- Use appropriate vector dimensions
- Implement proper indexing for fast retrieval