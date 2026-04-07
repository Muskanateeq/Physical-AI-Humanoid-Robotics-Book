# Physical AI & Humanoid Robotics - Interactive Learning Platform

> An intelligent, RAG-powered chatbot and interactive documentation platform for learning Physical AI and Humanoid Robotics concepts.

[![Author](https://img.shields.io/badge/Author-Muskan%20Atiq-blue)](https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert)
[![Python](https://img.shields.io/badge/Python-3.11-green)](https://www.python.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.115-teal)](https://fastapi.tiangolo.com/)
[![React](https://img.shields.io/badge/React-19.0-blue)](https://react.dev/)
[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.9-green)](https://docusaurus.io/)

## 📖 Overview

This project is a comprehensive interactive learning platform built around the **Physical AI and Humanoid Robotics** book authored by **Muskan Atiq**. It combines cutting-edge AI technologies to create an intelligent chatbot that can answer questions about robotics concepts using Retrieval-Augmented Generation (RAG).

### 🎯 Project Goals
- Make robotics education accessible and interactive
- Provide instant, accurate answers to robotics questions
- Demonstrate practical implementation of RAG systems
- Showcase modern full-stack AI application development
- Create a reusable template for educational AI platforms

The platform features:
- **NeuroBotics AI Assistant**: An intelligent chatbot trained on the book content
- **Interactive Documentation**: Docusaurus-based website with embedded chatbot
- **RAG-Powered Q&A**: Vector search using Qdrant for accurate, context-aware responses
- **User Authentication**: Secure authentication system using Better Auth
- **Multi-Module Content**: Covers 4 comprehensive modules on robotics

### 🎓 Who Is This For?
- **Students**: Learning robotics, ROS2, and Physical AI
- **Developers**: Building RAG-powered applications
- **Researchers**: Exploring AI-assisted education
- **Educators**: Creating interactive learning materials
- **Robotics Enthusiasts**: Understanding humanoid robotics concepts

## ✨ Key Features

### 🤖 Intelligent Chatbot
- **RAG-Based Responses**: Uses vector embeddings and semantic search to find relevant book content
- **Context-Aware**: Understands user queries and provides detailed, structured answers
- **Streaming Responses**: Real-time response generation for better UX
- **Multi-Mode Operation**: 
  - Book-grounded mode (when query matches book content)
  - General robotics mode (for related topics)
  - Creator info mode (for author/chatbot information)

### 📚 Book Content Coverage
The platform covers 4 main modules:
1. **Module 1**: The Robotics Nervous System (ROS2)
2. **Module 2**: The Digital Twin (Gazebo & Unity)
3. **Module 3**: The AI Robot Brain (Nvidia Isaac)
4. **Module 4**: Vision Language Action

### 🔐 Authentication System
- Email/password authentication
- Session management
- PostgreSQL-backed user storage
- Better Auth integration

### 🎨 Modern UI/UX
- Responsive Docusaurus documentation site
- Integrated chatbot widget
- Clean, professional design
- Mobile-friendly interface

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Frontend Layer                        │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Docusaurus (React 19) + ChatKit React Component      │ │
│  │  - Documentation pages                                 │ │
│  │  - Embedded chatbot widget                            │ │
│  │  - Better Auth UI integration                         │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                            ↓ HTTP/WebSocket
┌─────────────────────────────────────────────────────────────┐
│                      Backend Services                        │
│  ┌──────────────────────┐    ┌──────────────────────────┐  │
│  │  FastAPI + ChatKit   │    │  Better Auth Server      │  │
│  │  - RAG Service       │    │  - Express.js            │  │
│  │  - LLM Integration   │    │  - PostgreSQL            │  │
│  │  - Streaming API     │    │  - Session Management    │  │
│  └──────────────────────┘    └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                      Data Layer                              │
│  ┌──────────────────────┐    ┌──────────────────────────┐  │
│  │  Qdrant Vector DB    │    │  PostgreSQL              │  │
│  │  - Book embeddings   │    │  - User accounts         │  │
│  │  - Semantic search   │    │  - Sessions              │  │
│  └──────────────────────┘    └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## 🛠️ Tech Stack

### Backend
- **FastAPI** (0.115.6) - High-performance Python web framework
- **OpenAI ChatKit** (1.4.0) - Conversational AI framework
- **OpenAI Agents** (0.6.2+) - Agent orchestration with LiteLLM
- **Qdrant Client** (1.12.0) - Vector database client
- **FastEmbed** (0.5.0+) - Fast embedding generation
- **LiteLLM** - Multi-provider LLM integration (OpenRouter/Mistral)
- **Uvicorn** - ASGI server
- **Python-dotenv** - Environment management

### Frontend
- **Docusaurus** (3.9.2) - Documentation framework
- **React** (19.0.0) - UI library
- **ChatKit React** (1.4.0) - Chatbot UI component
- **Better Auth** (1.4.9) - Authentication library
- **Better Auth UI** (3.3.9) - Pre-built auth components

### Authentication Server
- **Express.js** (4.18.2) - Node.js web framework
- **Better Auth** (1.4.9) - Auth framework
- **PostgreSQL** (via pg 8.20.0) - Relational database
- **CORS** - Cross-origin resource sharing

### AI/ML
- **Embedding Model**: BAAI/bge-small-en-v1.5
- **LLM**: Mistral Small 3.1 24B (via OpenRouter)
- **Vector Database**: Qdrant

## 📁 Project Structure

```
physical-ai-humanoid-robotics-book/
├── backend/                          # FastAPI backend with RAG
│   ├── main.py                       # Main server with ChatKit integration
│   ├── rag.py                        # RAG service (embeddings + search)
│   ├── requirements.txt              # Python dependencies
│   ├── Dockerfile                    # Docker configuration
│   └── docker-compose.yml            # Multi-container setup
│
├── physical-ai-humanoid-robotics/    # Docusaurus frontend
│   ├── docs/                         # Documentation content
│   ├── src/                          # React components
│   │   ├── components/               # Custom components
│   │   └── pages/                    # Custom pages
│   ├── static/                       # Static assets
│   ├── docusaurus.config.js          # Docusaurus configuration
│   ├── package.json                  # Node dependencies
│   └── sidebars.js                   # Sidebar configuration
│
├── auth-server/                      # Better Auth server
│   ├── server.js                     # Express server
│   ├── migrate.js                    # Database migration script
│   ├── package.json                  # Node dependencies
│   └── .env                          # Auth environment variables
│
├── specs/                            # Feature specifications
│   ├── 001-book-embedding/           # Book embedding feature
│   ├── 002-rag-chatbot-book-qa/      # RAG chatbot feature
│   ├── 003-docusaurus-chatbot-integration/
│   ├── 004-docusaurus-rag-chatbot/
│   ├── 005-docusaurus-chatbot-integration/
│   └── 006-user-auth/                # Authentication feature
│
├── .specify/                         # SpecKit Plus templates
│   ├── memory/                       # Project memory
│   ├── templates/                    # Spec templates
│   └── scripts/                      # Automation scripts
│
├── history/                          # Project history
│   ├── prompts/                      # Prompt history records
│   └── adr/                          # Architecture decision records
│
├── generate_book_embeddings.py       # Script to generate embeddings
├── .env                              # Environment variables
├── CLAUDE.md                         # Claude Code instructions
├── TESTING-GUIDE.md                  # Testing documentation
└── README.md                         # This file
```

## 🚀 Getting Started

### Prerequisites

- **Python 3.11+**
- **Node.js 20+**
- **PostgreSQL** (for authentication)
- **Qdrant** (cloud or local instance)
- **OpenRouter API Key** (for LLM access)

### Environment Variables

Create a `.env` file in the project root:

```env
# Qdrant Configuration
QDRANT_URL=https://your-qdrant-instance.cloud
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=physical_ai_humanoid_robotics

# LLM Configuration
OPENROUTER_API_KEY=your_openrouter_api_key

# Embedding Model (optional, defaults to BAAI/bge-small-en-v1.5)
EMBEDDING_MODEL=BAAI/bge-small-en-v1.5

# RAG Configuration
RAG_SIMILARITY_THRESHOLD=0.6
```

Create `auth-server/.env`:

```env
# Database
DATABASE_URL=postgresql://user:password@localhost:5432/auth_db

# Better Auth
BETTER_AUTH_SECRET=your_secret_key_here
BETTER_AUTH_URL=http://localhost:3001

# CORS
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001
```

Create `physical-ai-humanoid-robotics/.env.local`:

```env
# Auth Server
BETTER_AUTH_URL=http://localhost:3001

# Backend API
CHATKIT_API_URL=http://localhost:8001/chatkit
```

### Installation

#### 1. Backend Setup

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

#### 2. Frontend Setup

```bash
cd physical-ai-humanoid-robotics

# Install dependencies
npm install
```

#### 3. Auth Server Setup

```bash
cd auth-server

# Install dependencies
npm install

# Run database migration
node migrate.js
```

### Running the Application

You need to run three services:

#### Terminal 1: Backend Server
```bash
cd backend
python main.py
# Server runs on http://localhost:8001
```

#### Terminal 2: Auth Server
```bash
cd auth-server
npm start
# Server runs on http://localhost:3001
```

#### Terminal 3: Frontend
```bash
cd physical-ai-humanoid-robotics
npm start
# Opens browser at http://localhost:3000
```

### Generating Book Embeddings

Before using the chatbot, generate embeddings from your book content:

```bash
python generate_book_embeddings.py
```

This script:
1. Reads book content from specified directories
2. Chunks the content into manageable pieces
3. Generates embeddings using FastEmbed
4. Stores embeddings in Qdrant with metadata

## 📖 Usage

### Chatbot Features

1. **Ask Questions**: Type any question about Physical AI or Humanoid Robotics
2. **Get Detailed Answers**: Receive comprehensive, structured responses with:
   - Clear explanations
   - Relevant examples
   - Module/chapter references
   - External resources (when applicable)

3. **Query Types**:
   - Book content questions (RAG-powered)
   - General robotics questions
   - Author/creator information
   - Technical concepts

### Example Queries

```
"What is ROS2 and how does it work?"
"Explain digital twins in robotics"
"How does Nvidia Isaac help with robot AI?"
"What are vision-language-action models?"
"Who created this chatbot?"
```

## 🧪 Testing

Run tests for the frontend:

```bash
cd physical-ai-humanoid-robotics

# Run tests in watch mode
npm test

# Run tests once
npm run test:run

# Run with coverage
npm run test:coverage
```

See [TESTING-GUIDE.md](./TESTING-GUIDE.md) for detailed testing documentation.

## 🐳 Docker Deployment

Build and run with Docker:

```bash
cd backend
docker-compose up -d
```

This starts:
- FastAPI backend
- Qdrant (if using local instance)

## 📊 API Endpoints

### Backend (Port 8001)

- `POST /chatkit` - ChatKit streaming endpoint
- `GET /health` - Health check
- `GET /debug/threads` - Debug thread storage

### Auth Server (Port 3001)

- `POST /api/auth/sign-up` - User registration
- `POST /api/auth/sign-in` - User login
- `POST /api/auth/sign-out` - User logout
- `GET /api/auth/session` - Get current session

## 🤝 Contributing

This is a learning platform project. Contributions are welcome!

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## 📝 Development Workflow

This project uses **Spec-Driven Development (SDD)** with SpecKit Plus:

1. **Specification**: Define features in `specs/`
2. **Planning**: Create implementation plans
3. **Tasks**: Break down into testable tasks
4. **Implementation**: Build features incrementally
5. **Testing**: Validate with comprehensive tests
6. **Documentation**: Update docs and ADRs

## 👨‍💻 Author

**Muskan Atiq**
- Fullstack Agentic AI Engineer
- AI & Data Science Expert
- Book Author: Physical AI and Humanoid Robotics
- LinkedIn: [muskan-muhammad-atiq-agentic-ai-expert](https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert)

## 📄 License

This project is part of the Physical AI and Humanoid Robotics educational initiative.

## 🙏 Acknowledgments

- **OpenAI ChatKit** - Conversational AI framework
- **Qdrant** - Vector database
- **Docusaurus** - Documentation platform
- **Better Auth** - Authentication framework
- **FastAPI** - Backend framework
- **OpenRouter** - LLM API access

## 🔗 Related Resources

- [Docusaurus Documentation](https://docusaurus.io/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Better Auth Documentation](https://www.better-auth.com/)
- [OpenAI ChatKit](https://github.com/openai/chatkit)

## 🔧 Troubleshooting

### Common Issues

#### Backend won't start
```bash
# Check if port 8001 is already in use
netstat -ano | findstr :8001  # Windows
lsof -i :8001                 # Linux/Mac

# Verify environment variables
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print(os.getenv('OPENROUTER_API_KEY'))"
```

#### Qdrant connection errors
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Check if collection exists: `physical_ai_humanoid_robotics`
- Ensure embeddings are generated: `python generate_book_embeddings.py`

#### Auth server issues
- Verify PostgreSQL is running
- Check `DATABASE_URL` in `auth-server/.env`
- Run migration: `cd auth-server && node migrate.js`

#### Frontend build errors
```bash
# Clear cache and reinstall
cd physical-ai-humanoid-robotics
rm -rf node_modules package-lock.json
npm install
npm run clear
npm start
```

#### Chatbot not responding
1. Check backend logs for errors
2. Verify ChatKit API URL in frontend `.env.local`
3. Check browser console for CORS errors
4. Ensure all three services are running

### Debug Mode

Enable detailed logging:

```python
# backend/main.py - Add at the top
import logging
logging.basicConfig(level=logging.DEBUG)
```

## ❓ FAQ

### Q: How does the RAG system work?
**A:** The system uses semantic search to find relevant book content:
1. User query is converted to embeddings using FastEmbed
2. Qdrant searches for similar content vectors
3. Top results are filtered by similarity threshold (default: 0.6)
4. Relevant context is formatted and sent to the LLM
5. LLM generates a comprehensive answer based on book content

### Q: Can I use a different LLM?
**A:** Yes! The system uses LiteLLM, which supports multiple providers:
```python
# backend/main.py - Change the model
llm_model = LitellmModel(
    model="openrouter/anthropic/claude-3-sonnet",  # or any other model
    api_key=os.getenv("OPENROUTER_API_KEY"),
)
```

### Q: How do I add more book content?
**A:** 
1. Add your content files to the appropriate directory
2. Update `generate_book_embeddings.py` to include new paths
3. Run: `python generate_book_embeddings.py`
4. Restart the backend server

### Q: Can I deploy this to production?
**A:** Yes! See the Production Deployment section below.

### Q: How do I change the similarity threshold?
**A:** Set `RAG_SIMILARITY_THRESHOLD` in `.env`:
- Lower (0.3-0.5): More permissive, may include less relevant results
- Medium (0.6-0.7): Balanced (recommended)
- Higher (0.8-1.0): Strict, only highly relevant results

### Q: Does the chatbot work offline?
**A:** Partially:
- ✅ Vector search works with local Qdrant
- ❌ LLM requires internet (OpenRouter API)
- Consider using local LLMs (Ollama, LM Studio) for full offline support

## 🚀 Production Deployment

### Environment Setup

#### 1. Backend (Railway/Render/Fly.io)

```bash
# Set environment variables
QDRANT_URL=https://your-production-qdrant.cloud
QDRANT_API_KEY=prod_api_key
OPENROUTER_API_KEY=prod_openrouter_key
RAG_SIMILARITY_THRESHOLD=0.6
```

#### 2. Frontend (Vercel/Netlify)

```bash
# Build command
npm run build

# Output directory
build

# Environment variables
BETTER_AUTH_URL=https://your-auth-server.com
CHATKIT_API_URL=https://your-backend.com/chatkit
```

#### 3. Auth Server (Railway/Render)

```bash
# Environment variables
DATABASE_URL=postgresql://user:pass@host:5432/db
BETTER_AUTH_SECRET=generate_strong_secret_here
BETTER_AUTH_URL=https://your-auth-server.com
ALLOWED_ORIGINS=https://your-frontend.com
```

### Security Checklist

- [ ] Use strong `BETTER_AUTH_SECRET` (32+ characters)
- [ ] Enable HTTPS for all services
- [ ] Set proper CORS origins (no wildcards in production)
- [ ] Use environment variables for all secrets
- [ ] Enable rate limiting on API endpoints
- [ ] Set up database backups
- [ ] Monitor API usage and costs
- [ ] Implement proper error handling
- [ ] Add request logging and monitoring

### Performance Optimization

#### Backend
```python
# Use connection pooling for Qdrant
from qdrant_client import QdrantClient
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=30,
    prefer_grpc=True,  # Faster than HTTP
)
```

#### Frontend
```javascript
// Enable production optimizations in docusaurus.config.js
module.exports = {
  // ...
  future: {
    experimental_faster: true,
  },
  webpack: {
    jsLoader: (isServer) => ({
      loader: require.resolve('swc-loader'),
      options: {
        jsc: {
          parser: {
            syntax: 'typescript',
            tsx: true,
          },
          target: 'es2017',
        },
      },
    }),
  },
};
```

### Monitoring

Set up monitoring for:
- API response times
- Error rates
- Database connections
- Vector search latency
- LLM API costs
- User authentication events

Recommended tools:
- **Sentry** - Error tracking
- **Datadog/New Relic** - APM
- **Grafana** - Metrics visualization
- **LogRocket** - Frontend monitoring

## 📸 Screenshots

> Add screenshots of your application here

### Homepage
![Homepage](./docs/screenshots/homepage.png)

### Chatbot Interface
![Chatbot](./docs/screenshots/chatbot.png)

### Authentication
![Auth](./docs/screenshots/auth.png)

### Documentation
![Docs](./docs/screenshots/docs.png)

## 🎯 Roadmap

### Planned Features
- [ ] Multi-language support (Urdu, Arabic, Spanish)
- [ ] Voice input/output for chatbot
- [ ] PDF export of conversations
- [ ] Bookmark favorite responses
- [ ] Share conversations via link
- [ ] Admin dashboard for analytics
- [ ] Mobile app (React Native)
- [ ] Offline mode with local LLM
- [ ] Integration with robotics simulators
- [ ] Code examples playground

### Future Enhancements
- Advanced RAG techniques (hybrid search, reranking)
- Fine-tuned models on book content
- Interactive diagrams and visualizations
- Video tutorials integration
- Community forum
- Quiz and assessment system

## 📊 Project Statistics

- **Lines of Code**: ~15,000+
- **Components**: 3 main services (Backend, Frontend, Auth)
- **Dependencies**: 50+ packages
- **Modules Covered**: 4 comprehensive robotics modules
- **Supported Queries**: Unlimited (RAG-powered)

## 🔐 Security

### Best Practices Implemented
- ✅ Environment variable management
- ✅ Password hashing (Better Auth)
- ✅ Session management
- ✅ CORS configuration
- ✅ SQL injection prevention (parameterized queries)
- ✅ XSS protection (React escaping)
- ✅ HTTPS ready
- ✅ API key rotation support

### Security Recommendations
1. Never commit `.env` files
2. Rotate API keys regularly
3. Use strong passwords (12+ characters)
4. Enable 2FA for production databases
5. Regular security audits
6. Keep dependencies updated
7. Monitor for suspicious activity

## 📞 Support

For questions or issues:
1. Check the documentation in `docs/`
2. Review the FAQ section above
3. Check troubleshooting guide
4. Review existing issues
5. Contact via LinkedIn

## 🌟 Star History

If you find this project helpful, please consider giving it a star!

## 📚 Additional Resources

### Learning Materials
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [Gazebo Tutorials](https://gazebosim.org/docs)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)

### Related Projects
- [OpenAI ChatKit Examples](https://github.com/openai/chatkit)
- [Qdrant Examples](https://github.com/qdrant/examples)
- [Better Auth Examples](https://github.com/better-auth/better-auth)

### Community
- Join robotics communities
- Follow AI/ML research papers
- Participate in hackathons
- Contribute to open source

---

**Built with ❤️ by Muskan Atiq | Powered by AI & Robotics**

*Last Updated: April 2026*
