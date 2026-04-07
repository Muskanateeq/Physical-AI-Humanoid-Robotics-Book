# Research: Docusaurus Chatbot Integration

## Decision: Docusaurus-Compatible ChatKit Implementation
**Rationale**: Need to integrate the existing ChatKit React frontend into Docusaurus without using ReactDOM.createRoot which is incompatible with Docusaurus. The solution is to create a Docusaurus-compatible React component that wraps the ChatKit functionality.

## Technical Approach
- Create a React component in the Docusaurus src/components directory
- Use the existing ChatKit configuration from the chatkit-frontend App.tsx
- Adapt the floating chat interface to work within Docusaurus pages
- Maintain all functionality: chat toggle, new chat button, conversation history, etc.

## Key Considerations
1. **Docusaurus Integration**: Use React components that can be imported into MDX pages or used as theme components
2. **State Management**: Use localStorage for thread persistence as in the original implementation
3. **Styling**: Maintain the existing dark theme and purple/gold color scheme
4. **API Connection**: Keep the same API endpoint configuration ('http://localhost:8000/chatkit')

## Implementation Strategy
1. Create a Docusaurus-compatible Chatbot component that doesn't require ReactDOM.createRoot
2. Adapt the floating UI to work within Docusaurus page structure
3. Ensure the component can be easily added to any Docusaurus page
4. Maintain all existing functionality from the original chatkit-frontend

## Alternatives Considered
- **Iframe approach**: Would isolate the chatbot but create integration and styling issues
- **Server-side rendering**: Would require significant changes to the existing ChatKit implementation
- **Custom chat implementation**: Would lose the existing UI and functionality
- **Docusaurus plugin**: Would be overkill for this use case and add unnecessary complexity

## Final Decision
Create a standalone React component that can be imported into Docusaurus pages, maintaining all existing ChatKit functionality while being compatible with Docusaurus' React rendering system.

## Authentication System Research

### Decision: Two-Step Account Creation with Better-Auth
**Rationale**: The system requires a two-step account creation process with mandatory onboarding to collect user background information before granting access to the RAG chatbot. Better-Auth provides a secure, well-documented solution that integrates well with FastAPI backends and supports the required functionality.

**Alternatives considered**:
- Custom authentication system: More complex to implement securely
- Third-party providers (Auth0, Firebase): Less control over onboarding flow
- Simple username/password: Doesn't meet the requirement for detailed user background collection

### Decision: Neon Serverless PostgreSQL for User Data
**Rationale**: Neon Serverless PostgreSQL provides the scalability, security, and serverless capabilities needed for the application. It integrates well with FastAPI and provides the transactional integrity required for the two-step signup process.

**Alternatives considered**:
- Traditional PostgreSQL: Requires more management overhead
- MongoDB: Less suitable for transactional operations needed in signup flow
- SQLite: Not appropriate for production web application with concurrent users

### Decision: Bilingual Functionality (English/Urdu)
**Rationale**: Supporting both English and Urdu languages meets the constitution requirement for user-centric design and makes the educational content accessible to a wider audience.

**Alternatives considered**:
- English only: Would exclude Urdu-speaking users
- Multiple language support: Would add unnecessary complexity initially

## Docusaurus Integration Research

### Decision: React Component Approach for Chatbot
**Rationale**: Creating a standalone React component for the chatbot ensures compatibility with Docusaurus while maintaining the UI/UX from the original chatkit-frontend implementation. This approach avoids conflicts with Docusaurus rendering system.

**Alternatives considered**:
- Direct ReactDOM.createRoot: Would conflict with Docusaurus rendering
- iframe embedding: Would create isolation issues and poor user experience
- Server-side rendering only: Would limit interactivity

### Decision: FastAPI Backend with Python 3.10
**Rationale**: FastAPI provides excellent performance, automatic API documentation, and async support. Python 3.10 is required for compatibility with chatkit-python.

**Alternatives considered**:
- Node.js/Express: Would not meet Python 3.10 requirement for chatkit-python
- Django: More complex than needed for API backend
- Flask: Less performant than FastAPI

## RAG System Research

### Decision: Dual-Mode RAG Operation
**Rationale**: Implementing both Selected-Text RAG Mode and Standard RAG Mode provides flexibility for users to get answers based on specific text selections or broader context, which is essential for an educational platform.

**Alternatives considered**:
- Standard RAG only: Would limit user flexibility
- Selected-Text RAG only: Would not allow general questions about book content

### Decision: FastEmbed for Query Embeddings
**Rationale**: FastEmbed provides local, API-key-free embeddings which is more secure and cost-effective than cloud-based embedding services.

**Alternatives considered**:
- OpenAI embeddings: Would require API keys and ongoing costs
- Hugging Face embeddings: Would require more setup and management
- Cohere embeddings: Would require API keys and ongoing costs

### Decision: Qdrant Cloud for Vector Storage
**Rationale**: Qdrant Cloud provides managed vector database service with good performance and scalability for the book content embeddings.

**Alternatives considered**:
- Self-hosted Qdrant: Requires more management
- Pinecone: Alternative vector database but Qdrant was already established in the codebase
- Weaviate: Alternative vector database but Qdrant was already established in the codebase

### Decision: Google Gemini 1.5 Flash for LLM
**Rationale**: Gemini 1.5 Flash provides good balance of performance, cost, and capabilities for the RAG system.

**Alternatives considered**:
- OpenAI GPT: Would require different API integration
- Anthropic Claude: Would require different API integration
- Open-source models: Would require more infrastructure management

## Frontend Technology Research

### Decision: @openai/chatkit-react for Frontend UI
**Rationale**: As required by the constitution, @openai/chatkit-react provides the necessary components to recreate the UI from the original chatkit-frontend while ensuring compatibility with the backend.

**Alternatives considered**:
- Custom UI components: Would not meet constitution requirements
- Alternative chat libraries: Would not meet constitution requirements

## Personalization System Research

### Decision: Chapter-Level Personalization
**Rationale**: Allowing users to personalize chapter content based on their technical background enhances learning effectiveness by tailoring explanations and examples to their skill level.

**Alternatives considered**:
- Global personalization: Would be less targeted
- No personalization: Would not meet feature requirements

## Deployment Research

### Decision: Multi-Platform Deployment Support
**Rationale**: Supporting deployment on Railway, Vercel, and other platforms provides flexibility for different hosting needs and requirements.

**Alternatives considered**:
- Single platform deployment: Would limit hosting options
- Self-hosting only: Would require more infrastructure management

## Analytics and Observability Research

### Decision: Comprehensive Event Logging
**Rationale**: Tracking user interactions, personalization usage, and system performance is essential for continuous improvement and monitoring of the educational platform.

**Alternatives considered**:
- Minimal logging: Would limit insights into user behavior
- Third-party analytics: Would add complexity and potential privacy concerns