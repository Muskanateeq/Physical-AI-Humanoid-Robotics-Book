# Implementation Plan: Docusaurus Integrated RAG Chatbot

**Branch**: `004-docusaurus-rag-chatbot` | **Date**: 2025-12-06 | **Spec**: [specs/004-docusaurus-rag-chatbot/spec.md](specs/004-docusaurus-rag-chatbot/spec.md)
**Input**: Feature specification from `/specs/004-docusaurus-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Docusaurus-integrated RAG chatbot with dual-mode functionality (Normal Mode using Qdrant RAG and Selected Text Mode using only highlighted content), voice mode, and comprehensive chat history management. The system uses ChatKit frontend SDK for UI components, ChatKit Python SDK for backend processing, FastAPI for API endpoints, Gemini for AI reasoning, Qdrant Cloud for vector storage, and Neon Postgres for chat history and user data.

## Technical Context

**Language/Version**: Python 3.11 (Backend), JavaScript/TypeScript (Frontend)
**Primary Dependencies**: FastAPI, ChatKit SDKs, Qdrant, Neon Postgres, Gemini API
**Storage**: Neon Serverless Postgres (chat history, feedback, user data), Qdrant Cloud (book content vectors)
**Testing**: pytest (backend), Jest (frontend)
**Target Platform**: Web application (Docusaurus integration)
**Project Type**: Web (frontend + backend architecture)
**Performance Goals**: <5s response time for 90% of queries, support 100+ concurrent users
**Constraints**: <200ms p95 response time for API calls, secure handling of API keys, follow context7 documentation for RAG implementation
**Scale/Scope**: 1000+ daily active users, multi-modal responses (text + voice), persistent chat history

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ **Modularity and Scalability**: Architecture separates frontend (ChatKit JS) and backend (FastAPI + ChatKit Python) components with well-defined APIs
- ✅ **Security-First Approach**: Secure API endpoints with proper authentication, secure handling of API keys in environment variables
- ✅ **Performance and Responsiveness**: Design includes caching strategies and optimized API responses
- ✅ **Documentation-First Development**: Following context7 documentation guidelines for RAG chatbot implementation
- ✅ **Technology Stack Adherence**: Using chatkit-js for frontend and chatkit-python for backend as required

## Project Structure

### Documentation (this feature)

```text
specs/004-docusaurus-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── chat_session.py
│   │   ├── chat_message.py
│   │   ├── user_feedback.py
│   │   └── voice_preference.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── gemini_service.py
│   │   ├── qdrant_service.py
│   │   ├── postgres_service.py
│   │   ├── voice_service.py
│   │   └── chatkit_service.py
│   ├── api/
│   │   ├── chat_endpoints.py
│   │   ├── history_endpoints.py
│   │   ├── feedback_endpoints.py
│   │   └── voice_endpoints.py
│   └── main.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── requirements.txt
├── Dockerfile
├── docker-compose.yml
└── .env.example

frontend/
├── src/
│   ├── components/
│   │   ├── ChatWindow.jsx
│   │   ├── FloatingChatIcon.jsx
│   │   ├── ChatMessage.jsx
│   │   ├── SelectedTextCapture.js
│   │   └── VoicePlayer.jsx
│   ├── services/
│   │   ├── apiService.js
│   │   └── voiceService.js
│   ├── hooks/
│   │   ├── useChatHistory.js
│   │   └── useVoiceMode.js
│   └── styles/
│       └── chatbot.css
├── public/
└── package.json
```

**Structure Decision**: Web application with separate backend and frontend directories to maintain clear separation of concerns between ChatKit frontend responsibilities and backend AI processing.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |

## Phase 0: Research & Analysis

### Research Summary

**Decision**: Use dual-mode chatbot architecture with separate processing paths for Normal Mode and Selected Text Mode
**Rationale**: Allows for distinct handling of Qdrant-based RAG vs. selected-text-only responses while maintaining clear separation of concerns
**Alternatives considered**: Single processing pipeline with mode detection (rejected due to complexity and potential performance issues)

**Decision**: Implement Gemini-based agent with OpenAI-Agent-style architecture
**Rationale**: Provides reasoning, memory, and tool-calling capabilities as specified in requirements
**Alternatives considered**: Simple prompt-based approach (rejected as insufficient for complex reasoning tasks)

**Decision**: Use Neon Postgres for all chat history and user data
**Rationale**: Provides ACID compliance and structured storage for chat history, feedback, and preferences
**Alternatives considered**: Mixed storage (Postgres + Redis) (rejected as overly complex for initial implementation)

## Phase 1: System Architecture & Design

### 1. System Architecture Diagram

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Docusaurus    │    │   Frontend      │    │   Backend       │
│   Book Website  │◄──►│   (ChatKit JS)  │◄──►│   (FastAPI)     │
│                 │    │                 │    │                 │
│  ┌─────────────┐│    │┌──────────────┐ │    │┌──────────────┐ │
│  │Floating Icon││    ││Chat Window   │ │    ││Chat Endpoints│ │
│  │             ││    ││              │ │    ││              │ │
│  │             ││    ││Text Capture  │ │    ││RAG Service   │ │
│  │             ││    ││              │ │    ││              │ │
│  │             ││    ││Voice Player  │ │    ││Gemini Service│ │
│  └─────────────┘│    │└──────────────┘ │    │└──────────────┘ │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │                       │
                              ▼                       ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │   Qdrant Cloud  │    │  Neon Postgres  │
                    │                 │    │                 │
                    │  ┌─────────────┐│    │┌──────────────┐ │
                    │  │Book Content ││    ││Chat History  │ │
                    │  │Vectors      ││    ││              │ │
                    │  │             ││    ││User Feedback │ │
                    │  │             ││    ││              │ │
                    │  │             ││    ││Voice Pref's  │ │
                    │  └─────────────┘│    │└──────────────┘ │
                    └─────────────────┘    └─────────────────┘
                              │                       │
                              ▼                       ▼
                       ┌─────────────────┐    ┌─────────────────┐
                       │   Gemini API    │    │   User Session  │
                       │                 │    │   Management    │
                       │  ┌─────────────┐│    │                 │
                       │  │AI Reasoning ││    │                 │
                       │  │& Response   ││    │                 │
                       │  │Generation   ││    │                 │
                       │  └─────────────┘│    │                 │
                       └─────────────────┘    └─────────────────┘
```

### 2. Folder Structure

**Backend Structure:**
```
backend/
├── src/
│   ├── models/                 # Data models and schemas
│   ├── services/              # Business logic and integrations
│   ├── api/                   # API endpoints and routing
│   ├── config/                # Configuration files
│   └── utils/                 # Utility functions
├── tests/                     # Test files
├── requirements.txt           # Python dependencies
├── Dockerfile                 # Container configuration
├── docker-compose.yml         # Multi-container setup
└── .env.example              # Environment variables template
```

**Frontend Structure:**
```
frontend/
├── src/
│   ├── components/            # React components
│   ├── services/              # API service clients
│   ├── hooks/                 # Custom React hooks
│   ├── styles/                # CSS/SCSS files
│   ├── utils/                 # Utility functions
│   └── assets/                # Static assets
├── public/                    # Public files
├── package.json              # Node.js dependencies
└── vite.config.js            # Build configuration
```

### 3. API Contracts / Endpoints

#### 3.1 Chat Endpoints

| Method | Endpoint | Purpose | Request Body | Response |
|--------|----------|---------|--------------|----------|
| POST | `/chat` | Normal mode RAG chat | `{message: string, sessionId?: string, mode: "normal"}` | `{response: string, sessionId: string, timestamp: string}` |
| POST | `/select-and-chat` | Selected text mode chat | `{selectedText: string, question: string, sessionId?: string, mode: "selected-text"}` | `{response: string, sessionId: string, timestamp: string}` |
| POST | `/chat` | Combined chat (with mode parameter) | `{message: string, selectedText?: string, mode: "normal\|selected-text", sessionId?: string}` | `{response: string, sessionId: string, timestamp: string}` |

#### 3.2 History Endpoints

| Method | Endpoint | Purpose | Request Body | Response |
|--------|----------|---------|--------------|----------|
| GET | `/history` | Get chat history | `{sessionId: string}` | `{messages: Array, sessionId: string, createdAt: string}` |
| GET | `/history/list` | Get session list | `{userId?: string}` | `{sessions: Array<{id, title, lastMessage, timestamp}>}` |
| DELETE | `/history` | Clear chat history | `{sessionId: string}` | `{success: boolean}` |

#### 3.3 Feedback Endpoints

| Method | Endpoint | Purpose | Request Body | Response |
|--------|----------|---------|--------------|----------|
| POST | `/feedback` | Submit feedback on message | `{messageId: string, feedback: "like\|dislike", comment?: string}` | `{success: boolean}` |
| GET | `/feedback/stats` | Get feedback statistics | `{sessionId?: string}` | `{likes: number, dislikes: number, total: number}` |

#### 3.4 Voice Endpoints

| Method | Endpoint | Purpose | Request Body | Response |
|--------|----------|---------|--------------|----------|
| POST | `/voice/generate` | Generate voice from text | `{text: string, voiceType?: "robotic\|natural"}` | `{audioUrl: string, success: boolean}` |
| GET | `/voice/preferences` | Get user voice preferences | `{sessionId: string}` | `{enabled: boolean, voiceType: string}` |
| POST | `/voice/preferences` | Update voice preferences | `{sessionId: string, enabled: boolean, voiceType?: string}` | `{success: boolean}` |

### 4. Database Schema

#### 4.1 Chat Sessions Table
```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    title VARCHAR(255),
    metadata JSONB
);
```

#### 4.2 Chat Messages Table
```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(10) CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB,
    source_type VARCHAR(20) DEFAULT 'normal' CHECK (source_type IN ('normal', 'selected-text')),
    selected_text TEXT
);
```

#### 4.3 User Feedback Table
```sql
CREATE TABLE user_feedback (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID REFERENCES chat_messages(id) ON DELETE CASCADE,
    user_id VARCHAR(255),
    feedback_type VARCHAR(10) CHECK (feedback_type IN ('like', 'dislike')),
    comment TEXT,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(message_id, user_id)
);
```

#### 4.4 Voice Preferences Table
```sql
CREATE TABLE voice_preferences (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    user_id VARCHAR(255),
    enabled BOOLEAN DEFAULT false,
    voice_type VARCHAR(20) DEFAULT 'robotic',
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### 5. ChatKit Integration Plan

#### 5.1 Frontend JS SDK Responsibilities
- Render chat UI components with floating icon
- Handle floating icon visibility on all Docusaurus pages
- Capture selected text from book content
- Manage chat history display and toggle
- Implement Like/Dislike/Copy buttons for each response
- Handle voice mode toggle and audio playback
- Send API requests to backend endpoints
- Store voice mode preferences in browser storage

#### 5.2 Backend Python SDK Responsibilities
- Process incoming chat requests from frontend
- Route requests to appropriate services (RAG vs. selected-text)
- Integrate with Gemini for response generation
- Store and retrieve chat history from Neon Postgres
- Process feedback and store in database
- Generate voice output for responses
- Handle session management and metadata

### 6. Selected Text Mode Flow

1. User selects/highlights text on Docusaurus page
2. Frontend JavaScript captures selected text using `window.getSelection()`
3. When chat is opened, selected text is automatically sent with the query
4. Backend receives request with `mode: "selected-text"` and the selected text
5. Instead of querying Qdrant, the selected text is used as the context
6. Gemini processes the question with the selected text as context
7. If answer is not in selected text, return "This information is not available in the book"
8. Response is sent back to frontend with source metadata

### 7. Voice Mode Flow

1. User toggles voice mode on in chat UI
2. Frontend stores preference in browser storage and sends to backend
3. When new response is received, frontend triggers voice generation
4. Backend generates robotic voice from text using TTS service
5. Audio is returned to frontend or cached with URL
6. Frontend plays audio simultaneously with text display
7. User can replay audio or toggle voice mode off

### 8. Gemini Agent Integration Plan

#### 8.1 Agent Architecture
- Implement OpenAI-Agent-style architecture using Gemini
- Create memory system to maintain conversation context
- Implement tool calling for Qdrant retrieval (Normal Mode)
- Create system prompt optimized for book content Q&A
- Handle both normal RAG and selected-text modes appropriately

#### 8.2 System Prompt Design
```
You are an AI assistant for a technical book. You must:
1. Answer ONLY based on the provided context
2. If using normal mode, retrieve relevant information from the book via RAG
3. If using selected-text mode, answer only from the provided selected text
4. If answer is not in the provided context, respond with "This information is not available in the book."
5. Provide clear, concise, and accurate answers
6. Maintain a helpful and professional tone
```

### 9. Frontend UI/UX Plan

#### 9.1 Floating Robotic Icon
- Always visible on Docusaurus pages
- Smooth animations for opening/closing
- Unobtrusive design that matches Docusaurus theme
- Notification indicator for new responses

#### 9.2 Chat Window Components
- Message bubbles with clear user/assistant distinction
- Auto-scroll to latest message
- Typing indicators during AI processing
- Clear chat history button
- Toggle for viewing chat history

#### 9.3 Selected Text Capture
- Automatic detection when chat is opened
- Visual highlighting of captured text
- Option to edit or clear selected text
- Mode indicator showing "Selected Text Mode"

#### 9.4 Response Features
- Like/Dislike buttons for each response
- Copy button for easy text extraction
- Voice mode toggle per response
- Timestamps for each message
- Source indicators (Qdrant vs. selected text)

### 10. Deployment Steps

#### 10.1 Local Setup
1. Clone repository
2. Set up environment variables (`.env` file)
3. Install backend dependencies: `pip install -r requirements.txt`
4. Install frontend dependencies: `npm install`
5. Start Qdrant Cloud connection
6. Start Neon Postgres connection
7. Run backend: `uvicorn src.main:app --reload`
8. Run frontend: `npm run dev`
9. Access Docusaurus site with integrated chatbot

#### 10.2 Production Setup
**Option A: Vercel + Railway**
- Deploy frontend to Vercel
- Deploy backend to Railway
- Connect to Qdrant Cloud and Neon Postgres
- Set up environment variables in deployment platforms

**Option B: VPS**
- Deploy using Docker Compose
- Set up reverse proxy with Nginx
- Configure SSL certificates
- Set up monitoring and logging

**Option C: Railway/Render**
- Deploy full stack using platform services
- Connect to external Qdrant and Neon services
- Configure CI/CD pipeline

### 11. Security and Best Practices

#### 11.1 Authentication & Session Management
- Secure API endpoints with proper authentication
- Session management using secure tokens
- Rate limiting to prevent abuse
- Input validation for all user inputs

#### 11.2 Data Security
- Encrypt sensitive data in database
- Secure handling of API keys (environment variables only)
- Proper SQL injection prevention
- XSS prevention in frontend rendering

#### 11.3 API Security
- Input sanitization for all endpoints
- Proper error handling without information leakage
- Authentication for sensitive endpoints
- HTTPS enforcement in production

## Phase 2: Implementation Tasks

*Note: Detailed tasks will be generated in tasks.md using /sp.tasks command*

- [ ] Backend API development
- [ ] Database schema implementation
- [ ] Qdrant integration
- [ ] Gemini AI integration
- [ ] Frontend components
- [ ] ChatKit integration
- [ ] Voice service implementation
- [ ] Testing and validation
- [ ] Deployment configuration