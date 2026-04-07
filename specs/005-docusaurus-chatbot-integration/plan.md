# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This implementation plan addresses the integration of a Retrieval-Augmented Generation (RAG) chatbot for the Docusaurus-based book titled "Physical AI & Humanoid Robotics". The book content is already stored in a vector database. The primary requirement is to convert the existing React/Vite ChatKit frontend (`chatkit-frontend/src/App.tsx`) into a Docusaurus-compatible frontend while maintaining identical UI and functionality. The system will use the approved tech stack: @openai/chatkit-react for frontend, chatkit-python with FastAPI for backend, BetterAuth for authentication, Neon PostgreSQL for data storage, Qdrant for vector storage, FastEmbed for embeddings, and Google Gemini 1.5 Flash for responses.

The technical approach involves implementing a dual-mode RAG system that supports both Selected-Text RAG Mode (using highlighted text in chapters) and Standard RAG Mode (using Qdrant similarity search). The system will include authentication with user profiles displaying names at the top of the book interface, personalized features for saving preferences and highlighting sections, bilingual support (English/Urdu) for both content and chatbot responses, and search functionality that returns relevant content or "Content not found". The chatbot will appear as a floating widget on every page with responsive UI and accessibility features. All implementation details will strictly follow Context7 documentation.

## Technical Context

**Language/Version**: Python 3.10 (required for chatkit-python compatibility), TypeScript/JavaScript for frontend
**Primary Dependencies**: FastAPI, chatkit-python, @openai/chatkit-react, BetterAuth, Neon PostgreSQL driver, Qdrant client, Google Generative AI SDK, FastEmbed
**Storage**: Neon Serverless PostgreSQL for user data and personalization, Qdrant Cloud for vector storage containing book content embeddings
**Testing**: pytest for backend, Jest/React Testing Library for frontend
**Target Platform**: Web application with Docusaurus frontend and FastAPI backend
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <2 sec response time for chat queries, 95% uptime for FastAPI endpoints, 100 concurrent users support
**Constraints**: Must use only approved tech stack (no custom components), BetterAuth UI only (no custom auth UI), Context7 documentation compliance, Docusaurus compatibility
**Scale/Scope**: 1000 concurrent users, multilingual support (English/Urdu), personalization per user profile, search functionality across book content

### Context7 Documentation Compliance
**Mandatory Reference Requirement**: For every implementation task, developers must first locate and reference the relevant Context7 documentation that justifies or explains the use of specific technologies. This includes identifying the exact reference, section, or note from Context7 before beginning any coding. Implementation must strictly follow all patterns, configurations, and best practices as outlined in Context7 documentation.

**Technology Stack Verification**: Before implementation, each technology must be verified against Context7 documentation, including:
- ChatKit-Python for backend chat functionality
- @openai/chatkit-react for frontend components
- FastAPI for backend framework
- Qdrant for vector database operations
- FastEmbed for embedding generation
- Google Gemini 1.5 Flash for response generation
- Neon PostgreSQL for data storage
- BetterAuth for authentication

**Documentation Adherence**: If current knowledge of any tech stack component is outdated, developers must rely strictly on Context7 documentation rather than assumptions.

### Authentication & Personalization
**Authentication Architecture**: Hybrid JWT Pattern with BetterAuth as the Issuer and Python/FastAPI as the Verifier
**User Management**: BetterAuth for user registration/login with Neon PostgreSQL for extended user profiles
**JWT Configuration**: BetterAuth configured to issue Access Tokens (JWT) and Refresh Tokens (HttpOnly Cookies) for stateless verification by Python backend
**Email Verification**: Mandatory email verification workflow before account activation to prevent spam
**User Flow**: 3-step process: (1) Registration with email verification, (2) Email verification confirmation, (3) Onboarding with background data collection
**Personalization Features**: User preferences, content highlighting, and recommendation engine based on user profile
**User Interface**: Display user name at top of book interface with personalized experience
**Data Storage**: User profiles, preferences, and highlighted sections stored in Neon PostgreSQL with foreign key relationships
**Security**: JWT signature validation in Python backend via Authorization: Bearer header for RAG chat access. JWT Validation Middleware to verify signature/expiration instead of session checks.

### Dual-Mode RAG System
**Selected-Text Mode**: Real-time context extraction from highlighted text in book chapters
**Standard Mode**: Vector similarity search against book content in Qdrant database
**Response Generation**: Google Gemini 1.5 Flash for generating context-aware answers with explicit version pinning for reproducibility and safe updates, including rollback strategy for scenarios where response behavior regresses or produces undesired outputs
**Content Integration**: Direct integration with Docusaurus book content for seamless experience

### Bilingual Support
**Language Options**: English and Urdu for both content display and chatbot responses
**Language Detection**: Automatic detection of user input language
**Translation Services**: Gemini-based translation with formatting preservation
**UI Localization**: Complete localization of interface elements

### Search Functionality
**Full-Text Search**: Search across entire book content using Qdrant vector search
**Result Display**: Relevant content snippets with proper highlighting
**Fallback Handling**: "Content not found" response when no relevant results exist
**Performance**: Optimized search with response times under 1 second
**Search UI**: Docusaurus-compatible search bar with modal interface, animated transitions, and responsive design
**Search UX Enhancements**: Implementation of predictive typing and autocomplete functionality to improve user experience, with dynamic suggestions that adapt based on user behavior and search history

### Search Bar UI Components
**Search Bar**: Fixed-position search bar in top navigation bar with slate blue styling and animated hover effects
**Search Modal**: Animated search interface that appears when search icon is clicked with smooth transitions
**Search Input**: Input field with placeholder "Start typing to search documentation..." and proper focus management
**Search Results**: Display relevant book content based on user's search query with proper highlighting and citations
**No Results Handling**: Show "No results found" message with suggestion to "Try different keywords" when search returns no matches
**Search Loader**: Animated loader with slate blue border styling during search operations
**Theme Integration**: Search UI elements use slate blue and goldenrod colors to match website theme
**Responsive Design**: Ensure search functionality works across all device types (mobile, tablet, desktop)
**Accessibility**: Include proper ARIA labels and keyboard navigation (arrow keys, enter, escape) for search functionality

### Frontend Integration
**Docusaurus Compatibility**: Seamless integration with existing Docusaurus structure
**Floating Widget**: Persistent chat widget appearing on every page
**Responsive Design**: Mobile-first approach with adaptive layouts
**Accessibility**: WCAG 2.1 AA compliance with proper ARIA attributes
**User Experience**: Smooth animations and intuitive interactions
**Homepage Authentication UI**: Fully animated, high-performance "Sign In" / "Sign Up" buttons directly on Docusaurus Homepage matching "Physical AI" futuristic theme (gradients, hover effects)

### Chatbot UI Integration Requirements (User Story 2)
**Reference Implementation**: All UI elements must be implemented exactly as defined in the existing chatkit-frontend/src/App.tsx file, including:
- Floating chat button with consistent positioning across all pages
- Header controls with new chat and close buttons
- Start screen with suggested prompts
- Smooth animations and transitions
- Hover effects and interactive states for all buttons
- Feedback buttons (like/dislike) for each response
- Copy button functionality for generated answers
- Dark mode support that follows Docusaurus theme
- Multilingual toggles that work seamlessly with Docusaurus internationalization
- Skeleton loading states for content personalization and chat responses

**Styling Implementation**: All styling must be implemented using Tailwind CSS, matching the App.tsx Tailwind patterns, with Tailwind properly configured and added to the Docusaurus project to support consistent interactive styling.

**Technical Implementation**: The frontend implementation should be placed in docusaurus-frontend/src/components/ChatInterface/ChatInterface.jsx with any additional styles in ChatInterface.module.css or via Tailwind utilities. Developers must explicitly handle:
- Server-Side Rendering (SSR)/Client-Side Rendering (CSR) hydration
- Shadow DOM encapsulation without breaking Docusaurus styling
- React import strategies for Docusaurus compatibility without ReactDOM.createRoot conflicts
- Chat state maintenance across page navigation using localStorage with proper synchronization

### Frontend Styling & UI Components
**Framework**: Tailwind CSS for styling with Docusaurus theme integration
**Responsive Design**: Mobile-first approach with breakpoints at sm:640px, md:768px, lg:1024px, xl:1280px
**Dark Mode**: System preference detection with manual override option
**Accessibility**: WCAG 2.1 AA compliance with proper ARIA labels and keyboard navigation

**Component Styling Details**:
- **Message Bubbles**:
  - User messages: `bg-indigo-600 text-white rounded-lg p-3 max-w-[85%] ml-auto`
  - Assistant messages: `bg-gray-100 dark:bg-gray-700 text-gray-800 dark:text-gray-200 rounded-lg p-3 max-w-[85%] mr-auto`
  - Loading states: Skeleton with `animate-pulse bg-gray-200 dark:bg-gray-600 rounded`
- **Input Area**:
  - Container: `flex items-center p-3 border-t border-gray-200 dark:border-gray-600`
  - Textarea: `flex-1 border border-gray-300 dark:border-gray-600 rounded-lg p-2 bg-white dark:bg-gray-800 text-gray-900 dark:text-white`
  - Send button: `ml-2 bg-indigo-600 hover:bg-indigo-700 text-white rounded-lg px-4 py-2 disabled:opacity-50`
- **Mode Selector**:
  - Container: `flex space-x-2 p-2 bg-gray-100 dark:bg-gray-800 rounded-lg`
  - Buttons: `px-3 py-1 rounded-md text-sm font-medium transition-colors`
  - Active: `bg-indigo-600 text-white`
  - Inactive: `bg-white dark:bg-gray-700 text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600`
- **Floating Chat Widget**:
  - Container: `fixed bottom-6 right-6 z-50`
  - Button: `w-14 h-14 rounded-full bg-indigo-600 text-white flex items-center justify-center shadow-lg hover:bg-indigo-700`
  - Badge: `absolute -top-2 -right-2 bg-red-500 text-white text-xs rounded-full h-6 w-6 flex items-center justify-center`
- **Skeleton Loading States**:
  - Message: `h-4 bg-gray-200 dark:bg-gray-600 rounded animate-pulse mb-2`
  - Avatar: `h-8 w-8 rounded-full bg-gray-200 dark:bg-gray-600 animate-pulse`
- **Accessibility Features**:
  - ARIA labels for all interactive elements
  - Keyboard navigation support (Tab, Enter, Arrow keys)
  - Screen reader support for chat messages
  - Focus management for modal dialogs

### Enhanced ChatKit-Python Backend Requirements
**Session Lifecycle Management**: The ChatKit-Python backend must maintain per-session conversation context, automatically expire or clean up inactive sessions, and enforce strict request validation and access control for all API endpoints. This includes implementing robust session state management with automatic cleanup mechanisms to prevent memory leaks and ensure optimal performance.

**Streaming & Partial Response Handling**: In addition to core streaming functionality, ChatKit-Python should handle partial response streaming, manage retries, and ensure proper chunking of messages for a smooth real-time user experience. The system must implement intelligent buffering and message chunking to provide a seamless streaming experience.

**Error Handling & Fallback Mechanisms**: The backend should include built-in error handling and fallback mechanisms specifically at the ChatKit-Python layer to gracefully manage failures in streaming, vector search, or external LLM calls. This includes implementing retry logic with exponential backoff, graceful degradation when services are unavailable, and proper error propagation to the frontend.

### Streaming & Concurrency
**Backend Framework**: FastAPI with async/await patterns for high concurrency
**Streaming Protocol**: Server-Sent Events (SSE) for real-time token streaming from Gemini
**Concurrency Model**: AsyncIO with uvicorn workers for handling multiple connections
**Connection Limits**: 100 concurrent users with request queuing for overflow

**Streaming Implementation Details**:
- **FastAPI Streaming Response**: Using `StreamingResponse` with async generator for token-by-token delivery
- **ChatKit Integration**: Async adapter layer to convert Gemini streaming responses to ChatKit format
- **Client-Side Handling**: EventSource API to receive and render streaming tokens in real-time
- **Buffer Management**: Client-side token buffering to optimize rendering performance
- **Connection Management**: WebSocket fallback for environments where SSE is not optimal

**Concurrency Handling**:
- **Async Endpoints**: All API endpoints defined with `async def` for non-blocking I/O
- **Background Tasks**: For long-running operations like embedding generation and database writes
- **Request Queuing**: Using asyncio queues to manage requests during high load
- **Rate Limiting**: Per-user and per-IP rate limiting to prevent abuse
- **Connection Pooling**: Database and external API connection pooling for efficiency
- **Resource Management**: Proper cleanup of connections and resources using async context managers

**Performance Optimization**:
- **Token Streaming**: Direct streaming from Gemini to client without intermediate buffering
- **Async Processing**: Non-blocking operations throughout the request lifecycle
- **Caching**: Redis-based caching for frequently accessed embeddings and responses
- **Load Balancing**: Multiple uvicorn workers behind a load balancer for horizontal scaling

### Environment Variables & Secrets Management
**Configuration Management**: All configuration loaded from environment variables using Pydantic BaseSettings
**Secret Storage**: API keys and sensitive data stored in environment variables, never in code
**Frontend Security**: No backend secrets exposed to frontend; session tokens retrieved securely via authentication

**Required Backend Environment Variables**:
- `GEMINI_API_KEY`: Google Gemini API key for LLM access
- `QDRANT_URL`: Qdrant Cloud instance URL
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `QDRANT_COLLECTION`: Name of the collection for book content embeddings
- `NEON_DATABASE_URL`: Neon PostgreSQL connection string
- `BETTER_AUTH_SECRET`: BetterAuth secret key for session encryption
- `BETTER_AUTH_URL`: Base URL for BetterAuth service
- `API_HOST`: Host for the API server (default: 0.0.0.0)
- `API_PORT`: Port for the API server (default: 8000)
- `DEBUG`: Enable debug mode (default: false)
- `LOG_LEVEL`: Logging level (default: INFO)
- `MAX_WORKERS`: Number of uvicorn workers for concurrency
- `REQUEST_TIMEOUT`: Request timeout in seconds
- `CONNECTION_POOL_SIZE`: Database connection pool size
- `REDIS_URL`: Redis URL for caching (optional)

**Required Frontend Environment Variables**:
- `REACT_APP_API_BASE_URL`: Base URL for backend API
- `REACT_APP_BETTER_AUTH_URL`: BetterAuth base URL
- `REACT_APP_CHATKIT_API_URL`: ChatKit API URL

**Security Measures**:
- All API keys stored in environment variables only
- No secrets committed to version control
- Environment-specific configurations for dev/staging/prod
- Secure session management with BetterAuth
- CORS configured to restrict frontend origins
- Input validation and sanitization on all endpoints
- Rate limiting to prevent API key abuse
- Regular rotation of API keys and secrets

### Error Handling & Logging
**Backend Framework**: Structured logging with correlation IDs, centralized error handling, and user-friendly error messages
**Frontend Framework**: Error boundaries, error propagation from backend, and graceful error handling
**Logging Format**: JSON structured logging with timestamp, level, correlation ID, component, and message
**Storage Locations**:
  - Backend: Application logs to stdout/stderr, aggregated by platform (Railway, Vercel, Fly.io)
  - Frontend: Console logging in development, error reporting service in production (e.g., Sentry)
**Alerting Strategy**:
  - Backend: Platform-native alerting for errors, performance metrics, and uptime monitoring
  - Frontend: Error reporting service for client-side errors and user impact metrics

**Backend Error Handling**:
- **Centralized Error Handler**: FastAPI exception handlers for different error types (validation, database, external service)
- **Retry Logic**: Exponential backoff for external service calls (Qdrant, Gemini) with configurable attempts
- **User-Friendly Messages**: Generic error messages for users with detailed logs for developers
- **Correlation IDs**: Unique request IDs propagated through the entire request lifecycle
- **Error Types**:
  - Validation errors (HTTP 400) with field-specific details in logs
  - Authentication errors (HTTP 401/403) with appropriate messages
  - Resource not found (HTTP 404) with helpful suggestions
  - External service errors (HTTP 502/503) with graceful degradation
  - Server errors (HTTP 500) with generic user messages and detailed logs
- **Logging Details**: Request/response data (without sensitive info), execution time, user ID, session ID, error stack traces

**Frontend Error Handling**:
- **Error Boundaries**: React error boundaries for component-level error isolation
- **Error Propagation**: Proper handling of backend error responses with appropriate UI feedback
- **Network Error Handling**: Retry logic for failed API requests with exponential backoff
- **User Feedback**: Clear, actionable error messages in the UI
- **Error Tracking**: Integration with error reporting services (e.g., Sentry) for production
- **Graceful Degradation**: Fallback UI states when services are unavailable

**Logging Format Details**:
- **Timestamp**: ISO 8601 format with timezone
- **Level**: DEBUG, INFO, WARNING, ERROR, CRITICAL
- **Correlation ID**: UUID to trace requests across services
- **Component**: Module or service name
- **User ID**: Anonymized user identifier (when available)
- **Session ID**: Session identifier for tracking conversations
- **Request Data**: HTTP method, URL, execution time (without sensitive data)
- **Error Details**: Error type, message, stack trace (in error cases)

### Frontend-Backend Communication
**API Client**: Dedicated API utility module (`utils/api.js`) for all backend communication
**Communication Protocol**: RESTful API with JSON payloads over HTTPS
**Authentication**: JWT tokens issued by BetterAuth via Authorization: Bearer header for protected endpoints. Python backend validates JWT signature/expiration for access control.
**Error Handling**: Consistent error response format with error codes and messages

**Backend Endpoints Mapping**:

**Authentication Endpoints**:
- `POST /auth/register` - User registration (Step 1)
  - Request: `{name: string, email: string, password: string, password_confirmation: string}`
  - Response: `{success: boolean, message: string, user_id: string, step: string}`
  - Error Handling: Validation errors for invalid input, conflict for existing email
- `POST /auth/onboarding` - Complete onboarding (Step 2)
  - Request: `{user_id: string, software_background: string, hardware_os: string}`
  - Response: `{success: boolean, message: string, user_id: string, onboarding_completed: boolean}`
  - Error Handling: Validation errors, unauthorized if Step 1 not completed
- `POST /auth/login` - User login
  - Request: `{email: string, password: string}`
  - Response: `{success: boolean, token: string, user: object}`
  - Error Handling: Invalid credentials, account locked
- `GET /auth/me` - Get current user profile
  - Headers: `Authorization: Bearer {token}`
  - Response: `{user: object with profile data}`
  - Error Handling: Unauthorized for invalid/expired tokens

**RAG Chat Endpoints**:
- `POST /api/rag-chat` - Main RAG chat functionality
  - Headers: `Authorization: Bearer {token}` (optional)
  - Request: `{message: string, session_id: string, mode: string, selected_text: string, language_preference: string}`
  - Response: `{response: string, sources: array, session_id: string, language: string, mode_used: string}`
  - Error Handling: Validation errors, service unavailable for external API failures
- `GET /api/history` - Get conversation history
  - Headers: `Authorization: Bearer {token}`
  - Query Params: `session_id` (optional), `limit` (default 50), `offset` (default 0)
  - Response: `{history: array of conversations}`
  - Error Handling: Unauthorized, validation errors for params
- `POST /api/search` - Search book content
  - Request: `{query: string, limit: number (default 10), language: string (default 'en')}`
  - Response: `{results: array, total_results: number, query_time_ms: number}`
  - Error Handling: Validation errors for query parameters

**Translation Endpoints**:
- `POST /api/translate` - Translate chapter content
  - Headers: `Authorization: Bearer {token}`
  - Request: `{chapter_content: string, target_language: string, preserve_formatting: boolean}`
  - Response: `{translated_content: string, source_language: string, target_language: string}`
  - Error Handling: Validation errors, unsupported language, translation service failure
- `POST /api/chatbot-translate` - Translate chatbot responses
  - Headers: `Authorization: Bearer {token}` (optional)
  - Request: `{text: string, source_language: string, target_language: string}`
  - Response: `{translated_text: string, source_language: string, target_language: string}`
  - Error Handling: Validation errors, translation service failure

**Personalization Endpoints**:
- `POST /api/personalize` - Personalize chapter content
  - Headers: `Authorization: Bearer {token}`
  - Request: `{chapter_id: string, chapter_content: string, user_id: string}`
  - Response: `{personalized_content: string, modifications: array, variant_id: string}`
  - Error Handling: Unauthorized, validation errors, personalization service failure
- `GET /api/personalization-history` - Get personalization history
  - Headers: `Authorization: Bearer {token}`
  - Response: `{history: array of personalization records}`
  - Error Handling: Unauthorized

**Feedback Endpoints**:
- `POST /api/feedback` - Submit user feedback
  - Headers: `Authorization: Bearer {token}` (optional)
  - Request: `{conversation_id: string, message_id: string, rating: string, comment: string}`
  - Response: `{success: boolean, message: string}`
  - Error Handling: Validation errors, resource not found

**Search Endpoints**:
- `POST /api/search` - Search book content
  - Request: `{query: string, limit: number (default 10), language: string (default 'en')}`
  - Response: `{results: array, total_results: number, query_time_ms: number}`
  - Error Handling: Validation errors for query parameters
- `GET /api/search/suggestions` - Get search suggestions based on query
  - Request: `{query: string}`
  - Response: `{suggestions: array of strings}`
  - Error Handling: Validation errors for query parameter

**Health Check Endpoints**:
- `GET /api/health` - System health check
  - Response: `{status: string, timestamp: datetime, dependencies: object}`
  - Error Handling: N/A (health check endpoint)

**Frontend API Client Implementation** (`utils/api.js`):
- **Base Configuration**: Default headers, base URL, timeout settings
- **Authentication Interceptor**: Automatic token inclusion and refresh
- **Error Handler**: Centralized error processing with user-friendly messages
- **Request Methods**: Dedicated methods for each endpoint type (auth, chat, translate, etc.)
- **Type Safety**: TypeScript interfaces for all request/response types
- **Caching Layer**: Optional caching for read operations with TTL

### Continuous Integration / Deployment
**CI/CD Platform**: GitHub Actions for automated workflows
**Repository Structure**: Separate workflows for development, staging, and production
**Environment Management**: Environment-specific configurations with secure secret handling

**GitHub Actions Workflows**:

**Development Workflow** (`.github/workflows/development.yml`):
- **Trigger**: On pull request to `develop` branch
- **Steps**:
  - Checkout code
  - Setup Node.js and Python environments
  - Install dependencies for both frontend and backend
  - Run linters (ESLint, Pylint/Flake8)
  - Run code formatters (Prettier, Black)
  - Execute unit and integration tests with coverage reports
  - Security scanning (Snyk, Bandit for Python)
  - Build frontend and backend applications
  - Deploy to development environment
- **Artifacts**: Test reports, coverage reports, build artifacts

**Staging Workflow** (`.github/workflows/staging.yml`):
- **Trigger**: On push to `main` branch (after PR merge)
- **Steps**:
  - All development workflow steps
  - Performance testing with load simulation
  - End-to-end testing using Playwright/Cypress
  - Security testing and penetration testing
  - Deployment to staging environment
  - Automated health checks
- **Environment**: Staging environment with production-like data (anonymized)

**Production Workflow** (`.github/workflows/production.yml`):
- **Trigger**: Manual dispatch or release tag creation
- **Steps**:
  - All staging workflow steps
  - Additional manual approval step
  - Database migration execution
  - Blue-green deployment strategy
  - Post-deployment health checks
  - Notification to team and stakeholders
- **Environment**: Production environment with full security measures

**Security & Code Quality**:
- **Dependency Scanning**: Automated scanning for vulnerable dependencies
- **Code Quality**: SonarQube or similar for code quality metrics
- **Secret Detection**: Prevent secrets from being committed to repository
- **Branch Protection**: Require PR reviews, status checks, and up-to-date branches

**Environment Variable Management**:
- **GitHub Secrets**: Store sensitive data in GitHub repository secrets
- **Environment Files**: Non-sensitive configuration in environment-specific files
- **Secret Injection**: Inject secrets during build/deployment processes
- **Configuration Validation**: Validate configuration before deployment

### Deployment Platforms & Production Readiness
**Supported Platforms**: Railway, Vercel, and Fly.io for backend deployment; GitHub Pages or similar platforms for Docusaurus frontend
**Auto-scaling**: Enabled on all supported platforms to handle varying load
**Production Requirements**: Follow Context7 documentation for all deployment implementation details

**Backend Deployment Configuration**:

**Railway Deployment**:
- **Setup**: Create Railway project and link via `railway link <project-id>`
- **Environment Variables**: Set via Railway dashboard (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, etc.)
- **Configuration**:
  - Runtime: Python 3.10
  - Build command: `pip install -r requirements.txt`
  - Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
  - Instance type: Standard (adjust based on load)
- **Auto-scaling**: Configure via Railway dashboard with min/max instances
- **Monitoring**: Built-in metrics and logging via Railway dashboard
- **Health Checks**: Use `/api/health` endpoint for readiness checks

**Vercel Deployment**:
- **Setup**: Connect GitHub repository via Vercel dashboard
- **Environment Variables**: Configure via Vercel dashboard
- **Configuration**:
  - Framework preset: None (custom Python application)
  - Build command: `pip install -r requirements.txt`
  - Output directory: dist/
  - Install command: `npm install` (for frontend if needed)
- **Auto-scaling**: Native Vercel serverless auto-scaling
- **Monitoring**: Vercel Analytics for performance metrics
- **Custom Domain**: Configure via Vercel dashboard

**Fly.io Deployment**:
- **Setup**: Install Fly CLI (`flyctl`) and run `fly launch`
- **Configuration**: Define in `fly.toml` file:
  ```toml
  [app]
  name = "docusaurus-chatbot-backend"

  [build]
  builder = "heroku/buildpacks:20"

  [http_service]
  internal_port = 8000
  force_https = true
  auto_stop_machines = true
  auto_start_machines = true
  min_machines_running = 1
  processes = ["app"]

  [[vm]]
  size = "shared-cpu-1x"
  memory = "1gb"
  ```
- **Environment Variables**: Set via `fly secrets set KEY=value`
- **Auto-scaling**: Configure machine scaling in fly.toml
- **Monitoring**: Fly.io dashboard metrics and logs
- **Health Checks**: Configure via fly.toml `[http_service]` section

**Docusaurus Frontend Deployment**:
- **GitHub Pages**:
  - Build command: `npm run build`
  - Source: `/docs` folder or `gh-pages` branch
  - Custom domain: Configure via GitHub repository settings
  - GitHub Actions workflow for automated deployment
- **Vercel**:
  - Framework preset: Docusaurus
  - Build command: `npm run build`
  - Output directory: `build/`
  - Environment variables for API endpoints
- **Netlify**:
  - Build command: `npm run build`
  - Publish directory: `build/`
  - Functions directory: (if needed for server-side features)

**Platform-Specific Secret Management**:
- **Railway**: Use Railway's built-in secrets management via dashboard
- **Vercel**: Use Vercel's environment variables via dashboard or vercel.json
- **Fly.io**: Use `fly secrets` command or fly.toml for secret configuration
- **GitHub Actions**: Use repository secrets for CI/CD pipeline

**Connection Pooling Configuration**:
- **Neon Postgres**: Use connection pooling via Neon's built-in pooling or configure in application
- **Backend Configuration**: Set appropriate pool size based on platform limits (typically 5-20 connections)
- **Environment Variables**: Configure pool settings via environment variables per platform

**Health Checks & Monitoring**:
- **Endpoint**: `/api/health` returns system status and dependency health
- **Platform Integration**: Configure health checks in platform dashboards
- **Alerting**: Set up platform-native alerting for uptime and performance metrics
- **Logging**: Structured logging to platform-native logging systems

### Deployment Platforms & Production Readiness
**Supported Platforms**: Railway, Vercel, and Fly.io for backend deployment; GitHub Pages or similar platforms for Docusaurus frontend
**Auto-scaling**: Enabled on all supported platforms to handle varying load
**Production Requirements**: Follow Context7 documentation for all deployment implementation details

**Backend Deployment Configuration**:

**Railway Deployment**:
- **Setup**: Create Railway project and link via `railway link <project-id>`
- **Environment Variables**: Set via Railway dashboard (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, etc.)
- **Configuration**:
  - Runtime: Python 3.10
  - Build command: `pip install -r requirements.txt`
  - Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
  - Instance type: Standard (adjust based on load)
- **Auto-scaling**: Configure via Railway dashboard with min/max instances
- **Monitoring**: Built-in metrics and logging via Railway dashboard
- **Health Checks**: Use `/api/health` endpoint for readiness checks

**Vercel Deployment**:
- **Setup**: Connect GitHub repository via Vercel dashboard
- **Environment Variables**: Configure via Vercel dashboard
- **Configuration**:
  - Framework preset: None (custom Python application)
  - Build command: `pip install -r requirements.txt`
  - Output directory: dist/
  - Install command: `npm install` (for frontend if needed)
- **Auto-scaling**: Native Vercel serverless auto-scaling
- **Monitoring**: Vercel Analytics for performance metrics
- **Custom Domain**: Configure via Vercel dashboard

**Fly.io Deployment**:
- **Setup**: Install Fly CLI (`flyctl`) and run `fly launch`
- **Configuration**: Define in `fly.toml` file:
  ```toml
  [app]
  name = "docusaurus-chatbot-backend"

  [build]
  builder = "heroku/buildpacks:20"

  [http_service]
  internal_port = 8000
  force_https = true
  auto_stop_machines = true
  auto_start_machines = true
  min_machines_running = 1
  processes = ["app"]

  [[vm]]
  size = "shared-cpu-1x"
  memory = "1gb"
  ```
- **Environment Variables**: Set via `fly secrets set KEY=value`
- **Auto-scaling**: Configure machine scaling in fly.toml
- **Monitoring**: Fly.io dashboard metrics and logs
- **Health Checks**: Configure via fly.toml `[http_service]` section

**Docusaurus Frontend Deployment**:
- **GitHub Pages**:
  - Build command: `npm run build`
  - Source: `/docs` folder or `gh-pages` branch
  - Custom domain: Configure via GitHub repository settings
  - GitHub Actions workflow for automated deployment
- **Vercel**:
  - Framework preset: Docusaurus
  - Build command: `npm run build`
  - Output directory: `build/`
  - Environment variables for API endpoints
- **Netlify**:
  - Build command: `npm run build`
  - Publish directory: `build/`
  - Functions directory: (if needed for server-side features)

**Platform-Specific Secret Management**:
- **Railway**: Use Railway's built-in secrets management via dashboard
- **Vercel**: Use Vercel's environment variables via dashboard or vercel.json
- **Fly.io**: Use `fly secrets` command or fly.toml for secret configuration
- **GitHub Actions**: Use repository secrets for CI/CD pipeline

**Connection Pooling Configuration**:
- **Neon Postgres**: Use connection pooling via Neon's built-in pooling or configure in application
- **Backend Configuration**: Set appropriate pool size based on platform limits (typically 5-20 connections)
- **Environment Variables**: Configure pool settings via environment variables per platform

**Health Checks & Monitoring**:
- **Endpoint**: `/api/health` returns system status and dependency health
- **Platform Integration**: Configure health checks in platform dashboards
- **Alerting**: Set up platform-native alerting for uptime and performance metrics
- **Logging**: Structured logging to platform-native logging systems

### Caching & Optimization
**Caching Layer**: Redis for high-performance caching of frequently accessed data
**Cache Strategy**: Multi-level caching with appropriate TTL and invalidation policies
**Performance Goals**: Reduce response times by 60-80% and decrease load on external services

**Data Caching Strategy**:

**Embedding Caching**:
- **What**: Frequently used text embeddings to avoid redundant FastEmbed calls
- **Key Pattern**: `embedding:{text_hash}`
- **TTL**: 24 hours (refreshed on access)
- **Invalidation**: When source content changes
- **Performance Impact**: Reduces embedding generation time by up to 90%

**Qdrant Query Result Caching**:
- **What**: Results of common similarity searches to reduce Qdrant load
- **Key Pattern**: `qdrant:{query_hash}:{top_k}`
- **TTL**: 1 hour for common queries, 15 minutes for specific queries
- **Invalidation**: When document embeddings are updated
- **Performance Impact**: Reduces Qdrant API calls by 70-80%

**Gemini Response Caching**:
- **What**: Responses for frequently asked questions to reduce Gemini API calls
- **Key Pattern**: `gemini:{query_hash}:{context_hash}`
- **TTL**: 4 hours for common questions, 30 minutes for specific queries
- **Invalidation**: When underlying content changes
- **Performance Impact**: Reduces Gemini API usage by 50-60%

**User Session Caching**:
- **What**: Active user sessions and conversation context
- **Key Pattern**: `session:{user_id}:{session_id}`
- **TTL**: 24 hours with automatic extension
- **Invalidation**: On session timeout or explicit logout
- **Performance Impact**: Reduces database lookups for active sessions

**Personalization Result Caching**:
- **What**: Personalized content variants based on user profile
- **Key Pattern**: `personalization:{user_id}:{chapter_id}:{profile_hash}`
- **TTL**: 12 hours or until user profile changes
- **Invalidation**: When user profile or chapter content changes
- **Performance Impact**: Reduces personalization computation time

**Caching Implementation Details**:
- **Redis Configuration**: Cluster mode for high availability, with persistence disabled for cache tier
- **Connection Pooling**: Efficient connection management to Redis
- **Fallback Strategy**: Continue operation if cache is unavailable
- **Cache Warming**: Pre-populate cache with common queries during deployment
- **Monitoring**: Track cache hit rates, eviction rates, and memory usage

**Performance Optimization**:
- **Database Query Optimization**: Indexing strategy for frequent queries, query result caching
- **CDN Integration**: Static assets served through CDN for faster delivery
- **Compression**: Gzip/Brotli compression for API responses
- **Connection Pooling**: Database and external API connection pooling
- **Asynchronous Processing**: Background tasks for non-critical operations
- **Resource Bundling**: Optimize frontend asset loading and bundling

### Testing Strategy
**Testing Framework**: pytest for backend, Jest/React Testing Library for frontend
**Test Coverage Goal**: 80%+ for all critical components
**Test Types**: Unit, Integration, E2E, Performance, Security, Accessibility

**Backend Testing**:
- **Unit Tests**: Individual functions, services, and utility functions using pytest
- **Integration Tests**: API endpoints, database operations, external service integrations
- **E2E Tests**: Complete user workflows from API request to response
- **Mocking Strategy**: Use pytest-mock and unittest.mock for external dependencies
- **Database Tests**: Use test database with pytest fixtures for data setup/teardown
- **API Tests**: Test all endpoints with various input scenarios and error conditions
- **Concurrency Tests**: Simulate multiple concurrent users using pytest-asyncio
- **Security Tests**: Authentication, authorization, and input validation testing

**Frontend Testing**:
- **Unit Tests**: Individual React components using Jest and React Testing Library
- **Integration Tests**: Component interactions and state management
- **E2E Tests**: Complete user workflows using Playwright or Cypress
- **Accessibility Tests**: Automated a11y testing using axe-core integrated into CI/CD pipelines, ensuring all pages and interactive components undergo WCAG 2.1 AA compliance checks with failures blocking deployments until resolved
- **Visual Regression**: Component appearance across different states and screen sizes
- **Performance Tests**: Component rendering and state update performance

**RAG-Specific Testing**:
- **Embedding Tests**: Verify FastEmbed generates consistent vectors
- **Retrieval Tests**: Test Qdrant similarity search accuracy
- **Response Quality**: Evaluate Gemini response relevance and accuracy
- **Dual-Mode Tests**: Verify both Selected-Text and Standard RAG modes
- **Citation Tests**: Ensure source citations are properly included

**Performance Testing**:
- **Load Testing**: Simulate 100+ concurrent users using tools like Locust
- **Response Time Testing**: Ensure <2s response times for 95% of requests
- **Database Performance**: Test query performance under load
- **API Bottleneck Testing**: Identify and resolve performance bottlenecks
- **Memory Usage**: Monitor memory consumption under various loads

**Continuous Testing**:
- **CI/CD Pipeline**: Automated testing on every commit and PR
- **Test Parallelization**: Run tests in parallel to reduce execution time
- **Code Coverage Reports**: Track coverage metrics and enforce minimum thresholds
- **Automated Deployment Tests**: Test deployments to staging environment

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Approved Tech Stack Compliance
✅ **ChatKit SDK**: Using @openai/chatkit-react for frontend and chatkit-python for backend as mandated
✅ **Backend Framework**: Using FastAPI with ChatKit Python as mandated
✅ **LLM**: Using Google Gemini (specifically Gemini 1.5 Flash) as mandated
✅ **Authentication**: Using BetterAuth only (no custom authentication systems) as mandated
✅ **Database**: Using Neon PostgreSQL only (no other databases) as mandated
✅ **Vector Database**: Using Qdrant Cloud as mandated
✅ **Embedding Engine**: Using FastEmbed as mandated
✅ **Frontend Framework**: Integrating with Docusaurus as mandated
✅ **No Custom Components**: Using only official components (no custom UI, auth, DB systems) as mandated
✅ **Context7 Compliance**: All implementation details will follow Context7 documentation exactly as mandated

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

Based on the requirements for Docusaurus integration with a separate backend, we'll implement a multi-project structure with distinct backend and frontend applications:

```text
chatkit-backend/
├── src/
│   ├── models/
│   │   ├── user.py                 # User model with onboarding data
│   │   ├── conversation.py         # Conversation and message models
│   │   └── personalization.py      # Personalization model
│   ├── services/
│   │   ├── auth_service.py         # BetterAuth integration
│   │   ├── rag_service.py          # RAG implementation with Qdrant/FastEmbed
│   │   ├── gemini_service.py       # Google Gemini integration
│   │   ├── translation_service.py  # Urdu/English translation
│   │   ├── personalization_service.py # Content personalization
│   │   └── chat_service.py         # Chat session management, lifecycle management, and error handling with ChatKit-Python
│   ├── api/
│   │   ├── v1/
│   │   │   ├── auth.py             # Authentication endpoints
│   │   │   ├── rag_chat.py         # RAG chat endpoints
│   │   │   ├── history.py          # Conversation history
│   │   │   ├── feedback.py         # Feedback endpoints
│   │   │   ├── personalization.py  # Personalization endpoints
│   │   │   ├── translation.py      # Translation endpoints
│   │   │   ├── search.py           # Search endpoints
│   │   │   └── suggestions.py      # Search suggestions endpoints
│   │   └── health.py               # Health check endpoint
│   ├── core/
│   │   ├── config.py               # Configuration and settings
│   │   ├── database.py             # Database connection
│   │   ├── qdrant_client.py        # Vector database client
│   │   └── security.py             # Security utilities
│   └── main.py                     # Application entry point
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── alembic/
│   └── versions/                   # Database migrations
├── requirements.txt
├── Dockerfile
└── pyproject.toml

docusaurus-frontend/
├── src/
│   ├── components/
│   │   ├── ChatInterface/          # Main chat UI component
│   │   │   ├── ChatWindow.jsx      # Chat window implementation
│   │   │   ├── Message.jsx         # Individual message component
│   │   │   ├── InputArea.jsx       # Message input area
│   │   │   └── ModeSelector.jsx    # Selected-text vs standard RAG selector
│   │   ├── Search/
│   │   │   ├── SearchBar.jsx       # Docusaurus-compatible search bar UI
│   │   │   ├── SearchModal.jsx     # Search results modal
│   │   │   └── SearchResults.jsx   # Display search results with highlighting
│   │   ├── Auth/
│   │   │   ├── Register.jsx        # Registration component
│   │   │   ├── Login.jsx           # Login component
│   │   │   └── Onboarding.jsx      # Two-step onboarding flow
│   │   ├── Personalization/
│   │   │   └── ProfileSettings.jsx # User profile and preferences
│   │   ├── Translation/
│   │   │   └── LanguageToggle.jsx  # Language selection
│   │   └── Layout/
│   │       └── DocusaurusLayout.jsx # Integration with Docusaurus
│   ├── pages/
│   │   └── chat.jsx                # Chat page integrated with Docusaurus
│   ├── css/
│   │   └── chat.css                # Custom chat styles
│   ├── utils/
│   │   ├── api.js                  # API client utilities
│   │   ├── auth.js                 # Authentication helpers
│   │   └── constants.js            # Shared constants
│   └── theme/
│       └── ChatWidget.jsx          # Chat widget for Docusaurus
├── static/
│   └── img/                        # Static images
├── docusaurus.config.js            # Docusaurus configuration
├── babel.config.js
├── package.json
├── tailwind.config.js
└── Dockerfile

chatkit-frontend/                    # Existing React/Vite ChatKit frontend (to be referenced during conversion)
├── src/
│   ├── App.tsx
│   ├── components/
│   └── services/
├── package.json
├── vite.config.ts
└── tsconfig.json
```

**Structure Decision**: We chose a multi-project architecture with separate backend (FastAPI) and frontend (Docusaurus) applications to maintain clear separation of concerns. The backend handles all business logic, authentication, RAG operations, and API endpoints, while the Docusaurus frontend integrates the chat functionality seamlessly into the documentation site. The existing chatkit-frontend serves as a reference for the component conversion process.

### System Enhancements for Robustness and Observability

To further improve the robustness, usability, and observability of the RAG chatbot system, the following enhancements are defined:

**Analytics & Telemetry**: Explicit user analytics and engagement tracking will be implemented for all chatbot interactions. This includes logging click events, session durations, query frequency, and other interaction metrics. Telemetry data will be stored securely in Neon PostgreSQL or an appropriate analytics platform and visualized through dashboards for actionable insights.

**Fallback for Translation/LLM Failures**: In addition to backend retry and error handling, the frontend will include explicit UI fallbacks. If Gemini API or translation services fail, users will see a clear offline message or prompt to retry, ensuring continuity of user experience while backend logging captures detailed failure information.

**Automated Accessibility Testing**: Automated accessibility testing using axe-core (or equivalent) will be integrated into the CI/CD pipeline. All pages and interactive components, including chat, search, and personalization features, will undergo WCAG 2.1 AA compliance checks with failures blocking deployments until resolved.

**Versioning & LLM Response Management**: Gemini 1.5 Flash model versioning will be strictly maintained, with explicit version pinning in production. A rollback strategy will be defined for scenarios where response behavior regresses or produces undesired outputs, ensuring reproducibility and consistency across deployments.

**Observability & Metrics Dashboards**: System observability will be expanded to include latency tracking, cache hit/miss metrics, API request throughput, and error rates. These metrics will be aggregated into real-time dashboards and tied to alerts to proactively monitor system health and performance.

**Search UI/UX Enhancements**: The search interface will be further enhanced with predictive autocomplete, keyboard navigation shortcuts, and dynamic suggestions. Users navigating via keyboard will experience seamless interactions, and suggestions will adapt based on user behavior and search history.

## Complexity Tracking

The implementation follows all constitutional requirements with no violations. All components use the approved tech stack as mandated:

| Component | Technology Used | Constitutional Requirement |
|-----------|----------------|---------------------------|
| Frontend UI | @openai/chatkit-react official components only | No custom UI components allowed |
| Backend Framework | FastAPI with chatkit-python | FastAPI mandated |
| Authentication | BetterAuth with official UI components only | BetterAuth mandated, no custom auth UI |
| Database | Neon PostgreSQL only | Neon PostgreSQL mandated |
| Vector Database | Qdrant Cloud | Qdrant mandated |
| Embeddings | FastEmbed | FastEmbed mandated |
| LLM | Google Gemini 1.5 Flash | Gemini mandated |
| Frontend Framework | Docusaurus integration | Docusaurus compatibility mandated |
| Documentation | Context7 compliance | Context7 documentation compliance mandated |
