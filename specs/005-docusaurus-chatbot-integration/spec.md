# Feature Specification: Docusaurus Chatbot Integration

**Feature Branch**: `005-docusaurus-chatbot-integration`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "my frontend website is ready which is made by docusaurus react library my docusaurus is frontend is about physical ai and humanoid robotics book that teach about physical ai & humanoid robotics and i want to integrate a rag chatbot that can give answer of users questions realetd to book content chatbot frontend is ready at this folder location /chatkit-frontend but the main cause is that this chatbot is build in react and vite.js frontend but i want to build chatbot with docusaursu compatible thaat can fully work with docusaurus but i want fully fledge this ui for chatbot that have implement in this chatbot folder location /chatkit-frontend add this rag chatbot frontend in my docusaurus website with docusaursu compatible my frontend docusuarus webite location is here /docusaurus-frontend now i want to create a rag chatbot frontend in my docusaurus website that actually match with this chatbot that are build in this fi". The existing chatbot frontend is located at `chatkit-frontend/src/App.tsx` and needs to be converted into Docusaurus style while keeping the UI identical and keeping all ChatKit functionality. The current chatbot frontend (React + ViteJS) must be converted into a Docusaurus-compatible frontend without changing any UI or features. The new frontend must keep all the same features exactly, no customization, nothing new — only the same ChatKit React components. Every part of the system must follow Context7 for rules, APIs, session handling, streaming, and ChatKit integration. I must only use the approved tech stack — no custom libraries, no alternate frameworks, nothing outside the listed tech.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Book Content Q&A via Chatbot (Priority: P1)

A visitor to the Physical AI and Humanoid Robotics book website wants to ask questions about the book content and receive accurate, context-aware answers. They should be able to interact with a chatbot interface that is seamlessly integrated into the Docusaurus website, providing instant responses based on the book's content.

**Why this priority**: This is the core value proposition - allowing users to get immediate answers to their questions about the book content, improving engagement and learning experience.

**Independent Test**: Can be fully tested by asking questions about book content and verifying that the chatbot provides relevant, accurate answers based on the book's material.

**Acceptance Scenarios**:

1. **Given** a user is on the Docusaurus website with the integrated chatbot, **When** they type a question related to the book content, **Then** the chatbot returns a relevant answer based on the book's content with proper citations.

2. **Given** a user asks a question about a specific topic in the book, **When** they submit the query, **Then** the chatbot provides a contextual response that references the relevant sections of the book.

---
### User Story 2 - Chatbot UI Integration (Priority: P2)

A visitor to the website should experience a seamless integration where the chatbot UI feels like a natural part of the Docusaurus website design, matching the existing theme and styling while maintaining the functionality from the original chatkit-frontend implementation.

**Why this priority**: Ensures the chatbot doesn't feel like an external addition but rather a native feature of the website, maintaining the professional appearance and user experience.

**Independent Test**: The chatbot interface appears consistent with the Docusaurus theme, maintains all UI functionality from the original implementation, and doesn't disrupt the user's navigation of the website.

**Acceptance Scenarios**:

1. **Given** a user is browsing the Docusaurus website, **When** they interact with the chatbot interface, **Then** the UI elements match the website's color scheme, typography, and styling conventions.

2. **Given** a user opens the chatbot interface, **When** they interact with its components (input field, send button, message history), **Then** all functionality works as expected with the same UI/UX as the original chatkit implementation.

---
### User Story 3 - Authentication and User Onboarding (Priority: P2)

A new user must complete a two-step account creation process before accessing the chatbot functionality. The system should guide users through account creation and onboarding to personalize their experience based on their technical background. The authentication UI MUST use BetterAuth's official UI components only, with no custom authentication UI elements.

**Why this priority**: Ensures proper user management and enables personalized content delivery based on user's software and hardware background, which is essential for an educational platform.

**Independent Test**: New users can successfully create an account, complete the onboarding process, and gain access to personalized chatbot functionality based on their background information.

**Acceptance Scenarios**:

1. **Given** a new user visits the website, **When** they initiate account creation, **Then** they are guided through Step 1 (name, email, password) followed by Step 2 (onboarding questionnaire) before gaining full access.

2. **Given** a user has completed Step 1 of registration, **When** they attempt to access main features, **Then** they are redirected to complete the onboarding process.

3. **Given** a user has completed the onboarding process, **When** they interact with the chatbot, **Then** the responses are personalized based on their software_background and hardware_os preferences.

### User Story 4 - Bilingual Functionality (Priority: P2)

Users should be able to switch between English and Urdu languages seamlessly across the platform. The system should provide a global language toggle and per-chapter translation capabilities, with the RAG chatbot responding in the appropriate language based on user preference or input.

**Why this priority**: Enables accessibility for Urdu-speaking users and provides a localized learning experience, which is essential for a diverse educational platform.

**Independent Test**: Users can switch languages globally, translate individual chapters to Urdu, and receive chatbot responses in their preferred language while maintaining content integrity.

**Acceptance Scenarios**:

1. **Given** a user accesses the website, **When** they click the global language toggle, **Then** all UI elements, navigation labels, and system messages switch between English and Urdu instantly.

2. **Given** a logged-in user is reading a chapter, **When** they click the "Translate to Urdu" button, **Then** the chapter content is accurately translated to Urdu while preserving formatting and structure.

3. **Given** a user asks a question in Urdu to the chatbot, **When** the language detection processes the input, **Then** the chatbot responds in Urdu with appropriate context-aware answers.

### User Story 5 - Dual-Mode RAG Functionality (Priority: P1)

Users should be able to interact with the RAG chatbot in two distinct modes: (1) Selected-Text RAG Mode where answers are generated exclusively from highlighted text in chapters, and (2) Standard RAG Mode where answers are generated from the entire book content through similarity search. The system should provide accurate, context-aware responses in both modes.

**Why this priority**: Enables flexible learning experiences where users can get answers based on specific text selections or broader context, which is essential for an educational platform.

**Independent Test**: Users can switch between RAG modes seamlessly and receive accurate responses that are properly grounded in the selected text or broader content.

**Acceptance Scenarios**:

1. **Given** a user highlights text in a chapter and asks a question, **When** the Selected-Text RAG Mode is activated, **Then** the response is generated exclusively from the selected text.

2. **Given** a user asks a general question without highlighting text, **When** the Standard RAG Mode is activated, **Then** the response is generated from relevant book content retrieved through Qdrant similarity search.

### User Story 6 - Chapter Personalization (Priority: P2)

Users should be able to personalize chapter content based on their technical background and preferences. The system should adapt explanations, examples, and instructions to match the user's skill level and hardware setup.

**Why this priority**: Enhances learning effectiveness by tailoring content to individual user needs and technical background, which is crucial for an educational platform.

**Independent Test**: Users can click "Personalize Chapter" and receive content adapted to their profile with improved comprehension and engagement.

**Acceptance Scenarios**:

1. **Given** a logged-in user is reading a chapter, **When** they click the "Personalize Chapter" button, **Then** the content adapts based on their onboarding data (software_background, hardware_os).

2. **Given** personalized content is displayed, **When** the user returns to the same chapter later, **Then** the personalized version is automatically restored.

### User Story 7 - Frontend & Docusaurus Compatibility (Priority: P1)

Users should experience seamless functionality across all Docusaurus pages with consistent UI elements, responsive design, and preserved chat state during navigation. The existing chatbot frontend located at `chatkit-frontend/src/App.tsx` must be converted into Docusaurus style while keeping the UI identical and keeping all ChatKit functionality. The current chatbot frontend (React + ViteJS) must be converted into a Docusaurus-compatible frontend without changing any UI or features. The new frontend must keep all the same features exactly, no customization, nothing new — only the same ChatKit React components. Every part of the system must follow Context7 for rules, APIs, session handling, streaming, and ChatKit integration. I must only use the approved tech stack — no custom libraries, no alternate frameworks, nothing outside the listed tech.

**Why this priority**: Ensures the integrated chatbot works flawlessly within the Docusaurus environment, maintaining the professional appearance and user experience.

**Independent Test**: The chatbot functions properly on all page types without conflicts, with responsive UI and preserved conversation state across navigation.

**Acceptance Scenarios**:

1. **Given** a user is browsing Docusaurus pages, **When** they navigate between different sections, **Then** the chat state is preserved and accessible via the floating button.

2. **Given** a user accesses the site on a mobile device, **When** they interact with the chatbot, **Then** all UI elements are responsive and usable.

### User Story 8 - Deployment & Production Readiness (Priority: P1)

The system should be deployable across multiple platforms with proper security, scalability, and monitoring capabilities.

**Why this priority**: Ensures the application can be reliably deployed and maintained in production environments with appropriate scaling and security measures.

**Independent Test**: The system can be deployed to different platforms (Railway, Vercel and Fly.io) with proper configuration management and monitoring.

**Acceptance Scenarios**:

1. **Given** proper environment variables are configured, **When** the deployment pipeline runs, **Then** the application deploys successfully with all services connected.

2. **Given** the system is under load, **When** traffic increases significantly, **Then** it scales appropriately and maintains performance.

### User Story 9 - Analytics & Observability (Priority: P3)

The system should capture user interactions and system performance metrics to enable continuous improvement and monitoring.

**Why this priority**: Provides insights into user behavior and system performance to guide future enhancements and identify issues proactively.

**Independent Test**: Analytics data is captured accurately without impacting user experience or system performance.

**Acceptance Scenarios**:

1. **Given** a user interacts with the chatbot, **When** they submit queries and receive responses, **Then** these interactions are logged for analysis.

2. **Given** the system is running, **When** performance metrics are monitored, **Then** response times and error rates are tracked and alertable.

### User Story 10 - Search Documentation Feature (Priority: P2)

Users should be able to search through the book content using a search documentation button in the top right corner of the navbar. When clicked, users can type to search for documentation, and relevant content from the book should be displayed. If no results are found, appropriate feedback should be shown with suggestions to try different keywords. The search UI should be fully animated with button hover effects, and the search loader should have a slate blue border. The search functionality should be fully integrated with the website's theme and provide a responsive experience across all device types.

**Why this priority**: Provides users with a quick way to find specific content in the book without having to navigate through chapters, improving the overall user experience and accessibility of the content.

**Independent Test**: Users can click the search button, enter search terms, and receive relevant results from the book content, with appropriate feedback when no results are found.

**Acceptance Scenarios**:

1. **Given** a user clicks the search documentation button in the navbar, **When** they type a search query, **Then** relevant book content is displayed based on the search terms.

2. **Given** a user enters search terms that match content in the book, **When** the search is executed, **Then** the results are displayed with proper highlighting and citations to the source material.

3. **Given** a user enters search terms that do not match any book content, **When** the search is executed, **Then** a "No results found" message is displayed with suggestions to try different keywords.

4. **Given** a user is on any device type (mobile, tablet, desktop), **When** they interact with the search feature, **Then** the interface remains responsive and fully functional.

---

### User Story 11 - RAG System Integration (Priority: P3)

The system should leverage RAG technology to provide accurate answers to user questions by retrieving relevant information from the book's content and generating responses based on that information, ensuring answers are grounded in the source material.

**Why this priority**: Ensures the chatbot provides accurate, factual responses based on the book content rather than generating potentially incorrect information, which is crucial for an educational resource.

**Independent Test**: Questions about book content result in responses that are factually accurate and can be traced back to specific parts of the book content.

**Acceptance Scenarios**:

1. **Given** a user asks a technical question about physical AI concepts from the book, **When** the RAG system processes the query, **Then** the response contains information that is directly sourced from the book's content.

2. **Given** the chatbot generates a response to a user query, **When** the response is examined, **Then** it should include references or citations to the relevant sections of the book content.

### User Story 12 - BetterAuth UI Requirements (Priority: P1)

All authentication UI elements MUST use BetterAuth's official UI components only, with no custom authentication interface elements. This includes login, signup, password reset, and user profile management interfaces.

**Why this priority**: Ensures consistency with BetterAuth's design and functionality while maintaining the constraint that no custom-built components are used outside the approved tech stack.

**Independent Test**: All authentication interfaces use BetterAuth's native components without any custom styling or replacement components.

**Acceptance Scenarios**:

1. **Given** a user needs to authenticate, **When** they access the login/signup interface, **Then** they see only BetterAuth's official UI components with no custom elements.

2. **Given** a user needs to reset their password, **When** they access the password reset interface, **Then** they see only BetterAuth's official UI components with no custom elements.

---
### Edge Cases

- What happens when the user asks a question that cannot be answered from the book content?
- How does the system handle multiple concurrent users asking questions simultaneously?
- What occurs when the backend API is temporarily unavailable?
- How does the system handle inappropriate or irrelevant questions?
- What happens when a user submits extremely long or malformed queries?
- What happens when a user attempts to bypass the onboarding step?
- How does the system handle expired sessions during the onboarding process?
- What occurs when a user provides invalid input during signup or onboarding?
- How does the system handle attempts to access main features without completing onboarding?
- What happens when a user abandons the onboarding process?
- What occurs when translation API is unavailable during chapter translation?
- How does the system handle mixed-language input in the chatbot?
- What happens when language detection fails to identify the input language?
- How does the system handle malformed Urdu text in translations?
- What occurs when a user rapidly switches between languages?
- What happens when Qdrant Cloud is unavailable during Standard RAG Mode?
- How does the system handle very long selected text in Selected-Text RAG Mode?
- What occurs when Gemini API is rate-limited or unavailable?
- How does the system handle embedding failures in FastEmbed?
- What happens when environment variables are missing or misconfigured?
- What occurs when personalization API fails during content adaptation?
- How does the system handle conflicts between user preferences and content availability?
- What happens when chat state storage exceeds browser limits?
- How does the system handle deployment failures or service unavailability?
- What occurs when analytics services are temporarily unavailable?
- How does the system handle excessive load beyond scaling capabilities?
- What happens when structured logging fails or disk space is exhausted?
- What happens when the search query is extremely long or malformed?
- How does the system handle search requests when the search index is temporarily unavailable?
- What occurs when multiple users perform searches simultaneously?
- How does the system handle search queries in different languages (English/Urdu)?
- What happens when the search functionality conflicts with other UI elements?
- How does the system handle very broad search queries that would return too many results?
- What occurs when the search index is being updated during a search request?
- How does the system handle search queries with special characters or symbols?
- What happens when the search UI is accessed on a device with limited processing power?

## Requirements *(mandatory)*

### Tech Stack Adherence *(mandatory)*

Every part of the system MUST use only these approved technologies. Nothing should be custom-built outside this tech stack:

- **ChatKit SDK**: @openai/chatkit-react for frontend and chatkit-python for backend
- **Backend Framework**: FastAPI with ChatKit Python integration
- **Language Model**: Google Gemini (specifically Gemini 1.5 Flash)
- **Authentication**: BetterAuth only (no custom authentication systems)
- **Database**: Neon PostgreSQL only (no other databases or custom DB systems)
- **Frontend Framework**: Docusaurus with ChatKit React components
- **Vector Database**: Qdrant Cloud for semantic search
- **Embedding Engine**: FastEmbed for query embeddings

**No custom-built components outside this tech stack are permitted.** This includes:
- No custom UI components (must use official ChatKit components)
- No custom authentication systems (must use BetterAuth only)
- No custom authentication UI (must use BetterAuth's official UI components only)
- No alternative databases (must use Neon PostgreSQL only)
- No custom session management (must use ChatKit/BackendAuth systems)
- No alternative LLMs (must use Gemini only)

### Context7 Documentation Compliance *(mandatory)*

All implementation must strictly follow Context7 documentation for all tech stack components:

- **ChatKit SDK Implementation**: All @openai/chatkit-react and chatkit-python implementations MUST follow Context7 documentation exactly
- **Backend Implementation**: FastAPI with ChatKit Python integration MUST follow Context7 documentation exactly
- **Authentication**: BetterAuth integration MUST follow Context7 documentation exactly
- **Database**: Neon PostgreSQL usage MUST follow Context7 documentation exactly
- **Frontend**: ChatKit React components usage MUST follow Context7 documentation exactly
- **RAG Architecture**: All RAG implementation details MUST follow Context7 documentation exactly
- **Session Management**: All session handling via ChatKit MUST follow Context7 documentation exactly
- **API Integration**: All API endpoints and communication patterns MUST follow Context7 documentation exactly

### Functional Requirements

- **FR-001**: System MUST integrate the chatbot UI seamlessly into the Docusaurus website structure and styling
- **FR-002**: System MUST provide RAG (Retrieval-Augmented Generation) functionality to answer user questions based on book content
- **FR-003**: Users MUST be able to ask questions in natural language and receive relevant responses based on the book content
- **FR-004**: System MUST maintain the UI/UX functionality from the existing chatkit-frontend implementation
- **FR-005**: System MUST handle user sessions and conversation history appropriately
- **FR-006**: System MUST provide proper error handling for failed API requests or unavailable services
- **FR-007**: System MUST ensure the chatbot interface is responsive and works across different device sizes
- **FR-008**: System MUST preserve the visual design and styling consistency with the Docusaurus theme
- **FR-009**: System MUST provide proper loading states and user feedback during question processing
- **FR-010**: System MUST handle different types of queries including technical questions, conceptual questions, and content summaries
- **FR-011**: System MUST implement a two-step account creation process with Better Auth. Implementation MUST use BetterAuth's official UI components only, with no custom authentication UI.
- **FR-012**: System MUST validate user input (full name, email, password, password confirmation) during Step 1 of account creation
- **FR-013**: System MUST redirect users to Step 2 onboarding after successful Step 1 completion
- **FR-014**: System MUST collect mandatory onboarding data (software_background, hardware_os) in Step 2
- **FR-015**: System MUST store user profile data in Neon Serverless Postgres linked to Better Auth user ID
- **FR-016**: System MUST enforce access control to restrict main features until onboarding is completed
- **FR-017**: System MUST securely manage user sessions using Better Auth
- **FR-018**: System MUST protect user secrets and not expose sensitive data to frontend
- **FR-019**: System MUST provide API endpoints for signup, onboarding, login, logout, and session verification
- **FR-020**: System MUST support personalization based on user's onboarding data (software_background, hardware_os)
- **FR-021**: System MUST store all authentication data in Neon Serverless PostgreSQL with proper foreign key relationships to Better-Auth user IDs
- **FR-022**: System MUST create temporary user records in Neon during Step 1 of account creation
- **FR-023**: System MUST update user records with onboarding data and set onboarding_completed flag in Neon during Step 2
- **FR-024**: System MUST log personalization events in Neon with chapter ID, timestamp, user background, and content variant
- **FR-025**: System MUST ensure transactional integrity during two-step signup process in Neon
- **FR-026**: System MUST support rapid retrieval of user metadata from Neon for authentication and personalization workflows
- **FR-027**: System MUST encrypt all data at rest and in transit for Neon database
- **FR-028**: System MUST restrict Neon access to authorized backend services only, prohibiting frontend access
- **FR-029**: System MUST provide a global language toggle in the top navigation bar for switching between English and Urdu
- **FR-030**: System MUST update all UI elements, navigation labels, and system messages instantly when language is switched
- **FR-031**: System MUST store user's language preference in Neon Serverless PostgreSQL for logged-in users
- **FR-032**: System MUST persist language preferences across page reloads and re-logins
- **FR-033**: System MUST provide a "Translate to Urdu" button for each chapter for logged-in users
- **FR-034**: System MUST generate accurate Urdu translations that preserve formatting, headings, and code blocks
- **FR-035**: System MUST allow toggling between English and Urdu at the chapter level
- **FR-036**: System MUST store translation history and language preferences per user in Neon PostgreSQL
- **FR-037**: System MUST automatically restore personalized or translated content on return visits
- **FR-038**: System MUST dynamically respond in English or Urdu based on global toggle or user input language
- **FR-039**: System MUST implement language detection to identify input language automatically
- **FR-040**: System MUST ensure proper text rendering and layout for both English and Urdu languages
- **FR-041**: System MUST use ONLY official @openai/chatkit-react components without ANY custom UI replacements; this is constitutionally mandated and overrides any other design requirements. Implementation MUST follow Context7 documentation exactly.
- **FR-042**: System MUST use chatkit-python with FastAPI for backend session management and message streaming. Implementation MUST follow Context7 documentation exactly.
- **FR-043**: System MUST integrate Gemini 1.5 Flash as the core language model for generating responses. Implementation MUST follow Context7 documentation exactly.
- **FR-044**: System MUST store all API credentials securely in .env file (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, etc.). Implementation MUST follow Context7 documentation exactly.
- **FR-045**: System MUST use Neon Serverless Postgres to store user profiles, personalization settings, and conversation history. Implementation MUST follow Context7 documentation exactly.
- **FR-046**: System MUST load all configuration from environment variables. Implementation MUST follow Context7 documentation exactly.
- **FR-047**: System MUST support Selected-Text RAG Mode when user highlights text in Docusaurus chapters. Implementation MUST follow Context7 documentation exactly.
- **FR-048**: System MUST support Standard RAG Mode using FastEmbed and Qdrant Cloud when no text is selected. Implementation MUST follow Context7 documentation exactly.
- **FR-049**: System MUST expose FastAPI endpoints (/api/rag-chat, /api/history, /api/personalize, /api/feedback, /api/translate) that are RAG-aware. Implementation MUST follow Context7 documentation exactly.
- **FR-050**: System MUST maintain bilingual consistency across all RAG components. Implementation MUST follow Context7 documentation exactly.
- **FR-051**: System MUST provide "Personalize Chapter" buttons at the beginning of each chapter for logged-in users
- **FR-052**: System MUST customize chapter content based on user's onboarding data (software_background, hardware_os)
- **FR-053**: System MUST save personalized content variants in Neon Serverless PostgreSQL with proper metadata
- **FR-054**: System MUST automatically restore personalized content on return visits based on user profile
- **FR-055**: System MUST consider personalized content when processing RAG queries related to personalized chapters
- **FR-056**: System MUST maintain both original and personalized versions of content with proper versioning
- **FR-057**: System MUST allow users to reset personalization or switch between personalized and original content
- **FR-058**: System MUST ensure Docusaurus compatibility without ReactDOM.createRoot conflicts; this MUST be achieved through proper React component integration techniques without replacing ChatKit components
- **FR-059**: System MUST support both SSR and CSR for optimal performance
- **FR-060**: System MUST maintain all styling consistency with Docusaurus theme through CSS customization of ChatKit components, NOT through custom component replacements
- **FR-061**: System MUST maintain chat state across page navigation using localStorage or session storage
- **FR-062**: System MUST implement a floating chatbot button that appears consistently across all pages
- **FR-063**: System MUST ensure all UI components are fully responsive and usable on mobile devices
- **FR-064**: System MUST implement dark mode that follows Docusaurus theme and adapts to system preferences
- **FR-065**: System MUST provide skeleton loading states for content personalization and chat responses
- **FR-066**: System MUST include proper ARIA labels and accessibility attributes for all interactive components
- **FR-067**: System MUST implement environment variable management for all API keys and service endpoints
- **FR-068**: System MUST implement secure secret management using platform-specific solutions
- **FR-069**: System MUST support deployment on Railway, Vercel, and Fly.io with auto-scaling capabilities
- **FR-070**: System MUST implement Neon Postgres connection pooling for optimal database performance
- **FR-071**: System MUST implement multi-level caching (CDN, API responses, database queries) for performance
- **FR-072**: System MUST implement structured logging with request IDs, user IDs, and component tracing
- **FR-073**: System MUST implement comprehensive error handling with graceful degradation and user-friendly messages
- **FR-074**: System MUST provide health check endpoints for monitoring and auto-healing capabilities
- **FR-075**: System MUST log user interactions including chat queries, personalization usage, and content engagement
- **FR-076**: System MUST track performance metrics such as response times, error rates, and API usage
- **FR-077**: System MUST provide a search documentation button in the top right corner of the navbar with proper styling and animations
- **FR-078**: System MUST implement search functionality that allows users to type and search for book content
- **FR-079**: System MUST display search results that match the user's query from the book content
- **FR-080**: System MUST show "No results found" message when search query does not match any book content
- **FR-081**: System MUST provide suggestions to users to "Try different keywords" when no search results are found
- **FR-082**: System MUST implement animated search UI with button hover effects matching the website theme
- **FR-083**: System MUST style the search loader with a slate blue border to match the website's color scheme
- **FR-084**: System MUST ensure the search functionality is fully responsive across all device types (mobile, tablet, desktop)
- **FR-085**: System MUST integrate the search UI with the website's theme using slate blue and other theme colors
- **FR-086**: System MUST provide proper search result highlighting and citations to source material
- **FR-087**: System MUST implement search loading states with appropriate visual feedback
- **FR-088**: System MUST ensure search functionality works seamlessly with existing chatbot and personalization features
- **FR-089**: System MUST preserve search state across page navigation where appropriate
- **FR-090**: System MUST implement proper keyboard navigation for search functionality
- **FR-091**: System MUST include accessibility features (ARIA labels, screen reader support) for the search functionality

### Key Entities *(include if feature involves data)*

- **User Query**: The question or input provided by the website visitor that needs to be answered based on book content
- **Book Content**: The source material from the Physical AI and Humanoid Robotics book that serves as the knowledge base for the RAG system
- **Generated Response**: The answer provided by the chatbot based on the book content and the user's query
- **Conversation History**: The sequence of interactions between the user and the chatbot during a session
- **User Account**: The user's account information managed by Better Auth including name, email, and authentication status
- **User Profile**: The user's profile data stored in Neon Serverless Postgres including software_background, hardware_os, and onboarding_completed status
- **Session**: The authenticated session managed by Better Auth to control access to application features
- **Onboarding Data**: The user's background information (software_background, hardware_os) collected during the mandatory onboarding process
- **Neon Database**: The Neon Serverless PostgreSQL database storing authentication, onboarding, and personalization data
- **User Profile Record**: The user's profile data stored in Neon with software_background, hardware_os, and onboarding_completed status
- **Personalization Log**: Records of user-specific chapter customizations, content variants, timestamps, and device information
- **Session Token**: Authentication tokens stored in Neon for session management
- **Language Preference**: User's selected language (English/Urdu) stored in Neon database
- **Translated Content**: Chapter content translated to Urdu while preserving original English version
- **Translation History**: Record of user's translation activities and preferences stored in Neon
- **Bilingual Chat Response**: Chatbot responses generated in the appropriate language based on user preference or input
- **RAG Chat Session**: Chat session managed by chatkit-python with FastAPI for handling user interactions
- **Selected Text**: User-highlighted text from Docusaurus chapters used for Selected-Text RAG Mode
- **Standard RAG Query**: User query processed through FastEmbed and Qdrant Cloud in Standard RAG Mode
- **Gemini Response**: AI-generated responses from Gemini 1.5 Flash model
- **Environment Configuration**: System configuration loaded from .env file including API keys and service endpoints
- **Conversation History**: Complete record of user-chatbot interactions stored in Neon database
- **Personalized Content**: Chapter content customized based on user's onboarding data and preferences
- **User Preference Profile**: User's stored preferences including theme settings, language, and personalization choices
- **Chat State**: Persistent chat session data maintained across page navigation
- **Floating Chat Button**: Persistent UI element providing access to chatbot functionality across all pages
- **Environment Configuration**: System configuration loaded from environment variables including API keys and service endpoints
- **Deployment Configuration**: Platform-specific deployment settings for Railway, Vercel and Fly.io.
- **Analytics Event**: Tracked user interactions and system metrics for observability and improvement
- **Performance Metric**: Measured system performance indicators including response times and error rates
- **Error Log**: Structured logging data for debugging and monitoring purposes
- **Search Query**: The text input provided by the user in the search documentation interface to find relevant book content
- **Search Results**: The collection of relevant book content returned based on the user's search query
- **Search Documentation Button**: The UI element in the top right corner of the navbar that triggers the search functionality
- **Search Loader**: The visual indicator showing the search process with slate blue border styling
- **Search State**: The current state of the search functionality including query input, results, and loading status
- **Search Configuration**: Settings for search behavior, styling, and integration with the website theme

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can ask questions about book content and receive relevant answers within 2 seconds of submitting their query
- **SC-002**: The integrated chatbot maintains at least 95% of the functionality present in the original chatkit-frontend implementation
- **SC-003**: 95% of user questions about book content receive factually accurate responses based on the book material
- **SC-004**: The chatbot interface seamlessly integrates with the Docusaurus website theme without visual inconsistencies
- **SC-005**: The system handles up to 100 concurrent users without performance degradation
- **SC-006**: At least 80% of new users successfully complete the two-step account creation process
- **SC-007**: User onboarding process completes within 2 minutes on average
- **SC-008**: System enforces access control with 100% accuracy, preventing unonboarded users from accessing main features
- **SC-009**: Personalization based on user's background data improves user engagement by at least 25%
- **SC-010**: Authentication system handles up to 1000 concurrent registration attempts without performance degradation
- **SC-011**: Global language toggle switches between English and Urdu with less than 500ms response time
- **SC-012**: At least 90% of users can successfully translate chapters to Urdu and toggle between languages
- **SC-013**: Urdu translations maintain at least 95% accuracy in preserving original formatting and structure
- **SC-014**: Language detection correctly identifies input language with 95% accuracy for chatbot responses
- **SC-015**: Bilingual chatbot responses are delivered in the correct language 98% of the time
- **SC-016**: Selected-Text RAG Mode delivers responses based solely on highlighted text with 98% accuracy
- **SC-017**: Standard RAG Mode provides relevant responses using Qdrant retrieval and FastEmbed with 95% accuracy
- **SC-018**: Gemini 1.5 Flash generates responses within 5 seconds for 95% of queries
- **SC-019**: System maintains 99% uptime for all FastAPI endpoints
- **SC-020**: Environment configuration loads correctly from .env file with 100% success rate
- **SC-021**: At least 85% of users engage with chapter personalization features
- **SC-022**: Personalized content loads within 1 second of user request
- **SC-023**: Docusaurus compatibility maintains 99% uptime across all page types
- **SC-024**: Mobile responsiveness scores 95+ on mobile usability tests
- **SC-025**: Floating chat button appears consistently across 100% of website pages
- **SC-026**: Dark mode adapts correctly to system preferences in 98% of user sessions
- **SC-027**: Deployment pipeline completes successfully in 99% of attempts
- **SC-028**: System maintains sub-500ms response time under 95th percentile load
- **SC-029**: Error handling provides user-friendly messages for 100% of handled error cases
- **SC-030**: Analytics capture 95% of user interactions without impacting performance
- **SC-031**: Search functionality loads within 500ms of user request
- **SC-032**: Search returns relevant results for 90% of valid queries within 1 second
- **SC-033**: Search documentation button is visible and accessible in the top right corner of the navbar on all pages
- **SC-034**: Search UI animations and hover effects function properly across 95% of user sessions
- **SC-035**: Search loader displays with correct slate blue border styling in 100% of search operations
- **SC-036**: Search functionality remains responsive across all device types (mobile, tablet, desktop) with 95% success rate
- **SC-037**: Search results include proper highlighting and citations to source material in 98% of cases
- **SC-038**: "No results found" message with keyword suggestions displays correctly in 100% of empty search cases
- **SC-039**: Search accessibility features (ARIA labels, keyboard navigation) function properly for 98% of users

## Clarifications

### Session 2025-12-09

- Q: Should users need to authenticate to use the chatbot? → A: Authentication required with two-step account creation process
- Q: How should the system store conversation history? → A: Store in backend database with user session tracking
- Q: How should the system handle questions that cannot be answered from the book content? → A: Provide a best-guess answer based on general knowledge
- Q: Where should the chatbot component be available on the website? → A: Floating button on all pages
- Q: What is the expected maximum length for user queries? → A: Limit to 500 characters

## Additional System Requirements

### Chapter Personalization Engine

The system must provide a chapter personalization engine that allows users to customize content based on their background and preferences:

- **Personalization UI**: Provide "Personalize Chapter" buttons at the beginning of each chapter for logged-in users
- **User Profile Integration**: Use onboarding data (software_background, hardware_os) to customize chapter content
- **Content Adaptation**: Modify explanations, examples, and hardware-specific instructions based on user's technical background
- **Backend Storage**: Save personalized content variants in Neon Serverless PostgreSQL with chapter ID, user ID, timestamp, and device platform
- **Content Restoration**: Automatically restore personalized content on return visits based on user profile and chapter history
- **RAG Integration**: Ensure personalized content is considered when processing RAG queries related to personalized chapters
- **Version Management**: Maintain both original and personalized versions of content with proper versioning
- **User Control**: Allow users to reset personalization or switch between personalized and original content

### Frontend & Docusaurus Compatibility

The frontend must be fully compatible with Docusaurus and provide an optimal user experience:

All frontend implementation details MUST follow Context7 documentation exactly.

- **React Integration**: Use proper React import strategies for Docusaurus compatibility without ReactDOM.createRoot conflicts
- **Shadow DOM Handling**: Implement proper Shadow DOM strategies for component encapsulation without breaking Docusaurus styling
- **SSR/CSR Handling**: Support both Server-Side Rendering (SSR) and Client-Side Rendering (CSR) for optimal performance
- **Cross-Page State Persistence**: Maintain chat state across page navigation using localStorage or session storage
- **Floating Chat Button**: Implement a floating chatbot button that appears consistently across all pages
- **Mobile Responsiveness**: Ensure all UI components are fully responsive and usable on mobile devices
- **Dark Mode Support**: Implement dark mode that follows Docusaurus theme and adapts to system preferences
- **Skeleton Loading States**: Provide skeleton loading states for content personalization and chat responses
- **Accessibility (ARIA)**: Include proper ARIA labels and accessibility attributes for all interactive components
- **Multilingual Toggles**: Implement language toggles that work seamlessly with Docusaurus internationalization
- **Component Reusability**: Design components to be reusable across different Docusaurus pages and layouts

### Deployment & Production Readiness

The system must be production-ready with proper deployment and scaling considerations:

All deployment implementation details MUST follow Context7 documentation exactly.

- **Environment Variables**: Document all required environment variables including API keys, database URLs, and service endpoints
- **Secret Management**: Implement secure secret management using platform-specific solutions (Railway, Vercel, and Fly.io etc.)
- **Backend Deployment**: Support deployment on Railway, Vercel, and Fly.io with auto-scaling capabilities
- **Docusaurus Deployment**: Support deployment pipelines with GitHub Actions to GitHub Pages or similar platforms
- **Connection Pooling**: Implement Neon Postgres connection pooling for optimal database performance
- **Caching Strategy**: Implement multi-level caching (CDN, API responses, database queries) for performance
- **Logging System**: Implement structured logging with request IDs, user IDs, and component tracing
- **Error Handling**: Implement comprehensive error handling with graceful degradation and user-friendly messages
- **Health Checks**: Provide health check endpoints for monitoring and auto-healing capabilities
- **Scaling Considerations**: Design for horizontal scaling with stateless services where possible
- **Security Headers**: Implement proper security headers and CSP policies for web security

### Analytics & Observability

The system must include comprehensive analytics and observability features:

All analytics and observability implementation details MUST follow Context7 documentation exactly.

- **Event Logging**: Log user interactions including chat queries, personalization usage, language preferences, and content engagement
- **Performance Metrics**: Track response times, error rates, API usage, and database query performance
- **User Analytics**: Monitor user behavior patterns, chapter engagement, and feature adoption rates
- **RAG Analytics**: Track retrieval quality, answer relevance, and user satisfaction with responses
- **Error Tracking**: Implement error tracking with detailed context and automatic alerting
- **A/B Testing**: Support A/B testing for UI components, personalization algorithms, and chatbot responses
- **Dashboard Integration**: Provide analytics dashboards for monitoring system health and user engagement
- **Data Privacy**: Ensure all analytics comply with privacy regulations and user consent requirements
- **Retention Analysis**: Track user retention and feature usage over time
- **Feedback Integration**: Collect and analyze user feedback for continuous improvement

### Search Documentation Feature Requirements

The system must provide a comprehensive search documentation feature that allows users to search through book content with a UI that matches the website's theme:

All search documentation feature implementation details MUST follow Context7 documentation exactly.

- **Search Button UI**: Provide a search documentation button in the top right corner of the navbar with slate blue styling and animated hover effects
- **Search Interface**: Implement a fully animated search interface that appears when the search button is clicked
- **Search Input**: Provide a search input field with placeholder text "Start typing to search documentation..."
- **Search Results**: Display relevant book content based on user's search query with proper highlighting
- **No Results Handling**: Show "No results found" message with suggestion to "Try different keywords" when search returns no matches
- **Search Loader**: Implement a search loader with slate blue border styling during search operations
- **Theme Integration**: Ensure search UI elements use slate blue and goldenrod colors to match the website theme
- **Button Animations**: Implement smooth button animations and hover effects for the search button
- **Responsive Design**: Ensure search functionality works across all device types (mobile, tablet, desktop)
- **Accessibility**: Include proper ARIA labels and keyboard navigation for search functionality
- **Search Performance**: Optimize search to return results within 1 second for most queries
- **Search State Management**: Maintain search state across page navigation where appropriate

### Deliverables

- Complete API specifications for personalization endpoints
- Deployment configuration templates for different platforms
- Analytics schema and dashboard specifications
- Frontend component documentation for Docusaurus integration
- Security and compliance guidelines
- Performance benchmarks and scaling recommendations
- Search documentation UI/UX specifications
- Search API specifications and integration points
- Search accessibility and keyboard navigation guidelines

## RAG Chatbot System Architecture

### Frontend Implementation

The frontend MUST use the official @openai/chatkit-react components exactly as documented in Context7, with absolutely NO custom UI replacements of any kind. This is a constitutionally mandated requirement that overrides any other design considerations. The following components are the ONLY acceptable UI elements for the chatbot interface:
- ChatKitProvider (MUST wrap the entire chat interface)
- Chat (MUST be the main chat container)
- ChatSession (MUST manage session state)
- MessageList (MUST display conversation history)
- Message (MUST render individual messages)
- MessageInput (MUST handle user input)
- TypingIndicator (MUST show when responses are being generated)
- Streamable (MUST handle streaming responses)
- useChatSession hook (MUST manage session logic)

All styling must be achieved through CSS customization of these official components, NOT through replacement with custom components. The Docusaurus-compatible implementation MUST maintain all functionality from the original chatkit-frontend App.tsx file while ensuring compatibility with Docusaurus (avoiding issues with `main.tsx` and `ReactDOM.createRoot`). All implementation details MUST follow Context7 documentation exactly.

### Backend Implementation

The backend must use chatkit-python with FastAPI to:
- Manage chatbot sessions
- Stream messages
- Control typing indicators
- Coordinate all inference logic
- Handle both operating modes (Selected-Text RAG and Standard RAG)

All backend implementation details MUST follow Context7 documentation exactly.

### LLM and Services Integration

- Use Gemini 1.5 Flash (via Google AI Studio API) as the core language model
- Store GEMINI_API_KEY securely in .env file
- Configure Qdrant Cloud with URL, API key, and collection name in .env
- Use selected FastEmbed model for embeddings
- Store Neon Serverless Postgres connection string (NEON_DATABASE_URL) in .env
- Save user profiles, personalization settings, onboarding metadata, feedback, and full conversation history in Neon

All LLM and services integration details MUST follow Context7 documentation exactly.

### Configuration Management

All services must load configuration strictly from environment variables:
- Gemini API credentials
- Qdrant Cloud configuration
- FastEmbed model selection
- Docusaurus project paths
- Neon Postgres connection details

All configuration management details MUST follow Context7 documentation exactly.

### Dual-Mode Operation

The chatbot must support two operating modes:

All dual-mode operation implementation details MUST follow Context7 documentation exactly.

**(1) Selected-Text RAG Mode:**
- User highlights text inside a Docusaurus chapter
- Backend bypasses Qdrant retrieval entirely
- Gemini generates answers only from the provided selected_text
- Forces strictly localized reasoning

**(2) Standard RAG Mode:**
- When no text is selected, user questions are embedded using FastEmbed
- Perform similarity search on Qdrant Cloud
- Gemini generates answers from top-k retrieved passages plus conversation memory

### API Endpoints

Backend must expose FastAPI endpoints that are RAG-aware and load all secrets and config from .env:
- /api/rag-chat
- /api/history
- /api/personalize
- /api/feedback
- /api/translate

All API endpoint implementation details MUST follow Context7 documentation exactly.

### Bilingual Support

The system must detect user input language (English or Urdu), reply in the same language, and maintain bilingual consistency across all components.

All bilingual support implementation details MUST follow Context7 documentation exactly.

### Deliverables

- Complete API specification for all endpoints
- Configuration schema for environment variables
- Integration patterns for dual-mode operation
- Security guidelines for credential management
- Performance benchmarks for both RAG modes

## Multilingual Requirements (Bilingual: English and Urdu)

All multilingual implementation details MUST follow Context7 documentation exactly.

### Global Language Functionality

The system must include a global language toggle prominently in the top navigation bar that allows users to switch instantly between English and Urdu. Switching the language must immediately update all UI elements, navigation labels, system messages, and downstream services such as the RAG chatbot, translation agent, and personalization engine.

### Language Preference Persistence

- For logged-in users, the selected language preference must be stored securely in the Neon Serverless PostgreSQL database under the user profile
- Language preferences must persist across page reloads or re-logins
- System must automatically restore user's preferred language on subsequent visits

### Per-Chapter Translation Functionality

- Support per-chapter Urdu translation for logged-in users
- Provide a "Translate to Urdu" button at the beginning of each chapter
- Translation Agent must fetch the chapter content—including any personalized modifications—and generate an accurate, natural Urdu translation
- Preserve headings, formatting, code blocks, and structural layout during translation
- Keep the English version intact while allowing toggle between English and Urdu at the chapter level
- Store translation history and language preferences per user in Neon PostgreSQL
- Automatically restore personalized or translated content on return visits

### RAG Chatbot Bilingual Support

- RAG chatbot must dynamically respond in English or Urdu according to:
  - Global language toggle
  - Per-chapter language selection
  - Language of the user's input
- Implement language detection to identify the input language automatically
- Ensure responses match the appropriate language based on context and preferences

### Integration Requirements

- All bilingual functionality must integrate seamlessly with:
  - Authentication system
  - Onboarding process
  - Personalization engine
  - Chapter-interaction logging
  - Agent workflows
- Maintain consistency across all components for a unified bilingual experience

### Technical Specifications

- Support both English and Urdu languages throughout the platform
- Ensure proper text rendering and layout for both languages
- Handle right-to-left (RTL) text direction for Urdu if needed
- Maintain content integrity during language switching
- Optimize performance for language switching operations

### Deliverables

- UI/UX specifications for global language toggle placement and functionality
- Database schema updates for language preference storage
- Translation API specifications and integration points
- Language detection and response logic specifications
- Testing scenarios for bilingual functionality

## Database Requirements (Neon Serverless PostgreSQL)

All database implementation details MUST follow Context7 documentation exactly.

### Core Database Functionality

Neon Serverless PostgreSQL must serve as the primary and secure backend database for authentication, onboarding, and personalization in the web application that uses Better-Auth for two-step signup. Neon must operate fully serverless to enable autoscaling, branching, low-latency queries, zero-maintenance storage, and point-in-time backups.

### Authentication Data Storage

- Store all authentication-related data, including user credentials, hashed passwords provided by Better-Auth, session tokens, and both partial and completed account states
- Maintain user records linked to Better-Auth user IDs with proper foreign key relationships
- Support rapid retrieval of user metadata for authentication workflows
- Handle high-concurrency scenarios for multiple simultaneous users

### Onboarding Metadata Storage

- Securely store onboarding metadata collected during the two-step signup flow:
  - software_background (Beginner, Intermediate, Advanced)
  - hardware_os (Windows, Mac, Linux, Chromebook/Web)
  - onboarding_completed boolean flag
- Ensure transactional integrity when updating user records during the two-step process

### Personalization Data Management

- Manage personalization preferences for Docusaurus chapters by tracking:
  - User-specific chapter customizations
  - Content variants
  - Timestamps
  - Device platform information
  - Background information
- Ensure a consistent and adaptive experience across sessions
- Log chapter ID, timestamp, user background, device platform, and chosen content variants

### Two-Step Signup Integration

- After Step 1 (name, email, password), create a temporary user record in Neon
- Once Step 2 onboarding is completed, update the same record with onboarding fields and set onboarding_completed flag to true
- Enforce transactional integrity for all updates during the signup process

### Chapter Personalization Workflow

- When a logged-in user clicks "Personalize Chapter":
  - Fetch onboarding data and past personalization history from Neon
  - Store or update personalized chapter content in Neon
  - Log chapter ID, timestamp, user background, device platform, and content variant
- Guarantee consistency across sessions through proper data storage and retrieval

### Data Models and Schema Requirements

- Define tables for: users, user_profiles, personalization_logs, session_tokens, and auxiliary tables
- Ensure proper foreign key relationships between Better-Auth user IDs and Neon records
- Include fields for versioned content, timestamps, and device/platform information
- Support fast lookups with appropriate indexing strategies

### Security and Access Control

- Encrypt all data at rest and in transit
- Restrict access to authorized backend services only
- Prohibit direct frontend access to Neon
- Implement secure connection protocols and authentication for backend services

### Transactional Operations

- Ensure all multi-step signup operations are transactionally safe
- Maintain consistency during personalization logging
- Handle concurrent updates safely
- Implement proper error handling for failed database operations

### Performance and Optimization

- Support rapid retrieval of user metadata for authentication and personalization workflows
- Implement caching, indexing, or query optimization recommendations
- Optimize for low-latency queries
- Handle high-concurrency scenarios efficiently

### Integration Considerations

- Address how Neon interacts with Better-Auth for partial and complete user accounts
- Ensure consistency between onboarding completion flags, session states, and personalized content
- Define clear API contracts between backend services and Neon
- Maintain synchronization between Better-Auth and Neon records

### Deliverables

- ER diagrams for all database tables and relationships
- Endpoint data access patterns for signup, onboarding, and chapter personalization
- Detailed data schema definitions
- Transactional flow diagrams for multi-step signup and personalization logging
- Security enforcement and access control logic
- Error handling strategies for failed writes, concurrent updates, or unavailable database services

## Authentication Requirements

All authentication implementation details MUST follow Context7 documentation exactly.

### Two-Step Account Creation System

The system must implement a two-step account creation flow using BetterAuth's official UI components only, with no custom authentication UI:

### Authentication UI Requirements

All authentication UI elements MUST use BetterAuth's official components and styling, with no custom authentication interface elements.

**Step 1 - Basic Account Creation:**
- User provides their full name, email, password, and password confirmation
- Backend must validate the input for proper format and password strength
- Create a partially registered user in Better Auth
- Immediately redirect the user to Step 2 without granting full access until onboarding is completed

**Step 2 - Mandatory Onboarding:**
- System presents a mandatory onboarding questionnaire with fields for:
  - software_background (Beginner, Intermediate, Advanced)
  - hardware_os (Windows, Mac, Linux, or Chromebook/Web)
- Onboarding data must be securely stored in a user_profile data structure linked to the Better Auth user ID
- Fields must include: software_background, hardware_os, and onboarding_completed (boolean)
- Only after onboarding is completed should the system mark the account as fully activated and grant access to main application features

### Session and Security Management

- Only partially registered users from Step 1 may access Step 2
- Any attempt to access main application features without completing onboarding must redirect the user to Step 2
- Better Auth must manage secure session cookies, password hashing, sign-in, sign-out, and authenticated route protection
- No secrets or sensitive data should ever be exposed to the frontend
- Backend must define API endpoints for Step 1 (/signup), Step 2 (/onboarding), login, logout, and session verification, including proper input validation
- The user_profile schema must be clearly defined and linked to the Better Auth user ID, tracking the onboarding completion status

### Personalization Data Usage

- Onboarding data, including software_background and hardware_os, must be leveraged to personalize chapter content and RAG chatbot responses
- Specification must include how this data can be safely accessed by backend services for personalization purposes

### Error Handling and Edge Cases

- System must handle error conditions and edge cases, including:
  - Invalid or malformed input during signup or onboarding
  - Expired sessions
  - Attempts to bypass the onboarding step
- Clear HTTP response codes must be defined for all failure scenarios

### Deliverables

- Flow diagrams for Step 1 and Step 2
- Detailed endpoint descriptions with request and response formats
- Data models for Better Auth users and user_profile
- Session and access control logic
- Security enforcement rules
- State transitions for partial to complete accounts
- Assumptions or dependencies

## Backend Implementation Plan

All backend implementation details MUST follow Context7 documentation exactly.

### Environment and Dependencies
- **Python Version**: MUST run on Python 3.10 to ensure full compatibility with chatkit-python and all dependencies
- **Virtual Environment**: Implementation MUST begin with creating a Python 3.10 virtual environment and installing necessary dependencies
- **Dependencies**: MUST include FastAPI, chatkit-python, qdrant-client, google-generativeai, fastembed, better-auth, and related packages

### Authentication and User Management
- **Better Auth Integration**: MUST implement Better Auth for user authentication and session management
- **BetterAuth UI Components**: MUST use BetterAuth's official UI components only, with no custom authentication UI
- **Two-Step Flow**: MUST implement Step 1 (signup) and Step 2 (onboarding) account creation flow
- **User Validation**: MUST validate user input (name, email, password strength) during Step 1
- **Profile Storage**: MUST store user profile data in Neon Serverless Postgres with fields: software_background, hardware_os, onboarding_completed
- **Access Control**: MUST enforce access control to restrict features until onboarding completion
- **Session Management**: MUST securely manage user sessions using Better Auth
- **API Endpoints**: MUST provide endpoints for signup, onboarding, login, logout, and session verification

### Database Integration (Neon Serverless PostgreSQL)
- **Neon Configuration**: MUST configure Neon Serverless PostgreSQL for autoscaling, branching, and zero-maintenance operation
- **Data Encryption**: MUST ensure all data is encrypted at rest and in transit
- **Connection Security**: MUST implement secure connection protocols and authentication for backend services
- **Schema Management**: MUST define and maintain database schema for users, user_profiles, personalization_logs, and session_tokens
- **Foreign Key Relations**: MUST establish proper foreign key relationships between Better-Auth user IDs and Neon records
- **Transactional Integrity**: MUST enforce transactional integrity for all multi-step operations (signup, onboarding, personalization)
- **Indexing Strategy**: MUST implement appropriate indexing for fast lookups of user metadata
- **Personalization Logging**: MUST log chapter personalization events with chapter ID, timestamp, user background, and content variant
- **Query Optimization**: MUST optimize queries for rapid retrieval of user metadata for authentication and personalization workflows

### Multilingual Support
- **Language Toggle**: MUST implement global language toggle functionality in the top navigation bar
- **Language Persistence**: MUST store user language preferences in Neon database and restore on subsequent visits
- **Translation API**: MUST integrate with translation API for generating Urdu chapter translations
- **Language Detection**: MUST implement language detection to identify input language automatically
- **Bilingual Chatbot**: MUST ensure chatbot responds in the appropriate language based on context and user preference
- **Content Preservation**: MUST preserve formatting, headings, and code blocks during Urdu translation
- **Chapter Toggle**: MUST allow toggling between English and Urdu at the chapter level
- **Translation History**: MUST store translation history and language preferences per user in Neon PostgreSQL

### RAG System Integration
- **Frontend Components**: MUST use official @openai/chatkit-react components without custom UI replacements
- **Backend Services**: MUST use chatkit-python with FastAPI for session management and message streaming
- **LLM Integration**: MUST integrate Gemini 1.5 Flash as the core language model for generating responses
- **Environment Configuration**: MUST load all API credentials and service endpoints from .env file
- **Dual-Mode Operation**: MUST support both Selected-Text RAG Mode and Standard RAG Mode
- **Qdrant Integration**: MUST perform similarity search on Qdrant Cloud for Standard RAG Mode
- **FastEmbed Integration**: MUST use FastEmbed for query embeddings in Standard RAG Mode
- **API Endpoints**: MUST expose RAG-aware endpoints (/api/rag-chat, /api/history, /api/personalize, /api/feedback, /api/translate)
- **Conversation Storage**: MUST store complete conversation history in Neon database
- **Bilingual Consistency**: MUST maintain language consistency across all RAG components

### Chapter Personalization Engine
- **Personalization UI**: MUST provide "Personalize Chapter" buttons at the beginning of each chapter for logged-in users
- **User Profile Integration**: MUST use onboarding data (software_background, hardware_os) to customize chapter content
- **Content Adaptation**: MUST modify explanations, examples, and hardware-specific instructions based on user's technical background
- **Backend Storage**: MUST save personalized content variants in Neon Serverless PostgreSQL with chapter ID, user ID, timestamp, and device platform
- **Content Restoration**: MUST automatically restore personalized content on return visits based on user profile and chapter history
- **RAG Integration**: MUST ensure personalized content is considered when processing RAG queries related to personalized chapters
- **Version Management**: MUST maintain both original and personalized versions of content with proper versioning
- **User Control**: MUST allow users to reset personalization or switch between personalized and original content

### Frontend & Docusaurus Compatibility
- **React Integration**: MUST use proper React import strategies for Docusaurus compatibility without ReactDOM.createRoot conflicts
- **SSR/CSR Handling**: MUST support both Server-Side Rendering (SSR) and Client-Side Rendering (CSR) for optimal performance
- **Cross-Page State Persistence**: MUST maintain chat state across page navigation using localStorage or session storage
- **Floating Chat Button**: MUST implement a floating chatbot button that appears consistently across all pages
- **Mobile Responsiveness**: MUST ensure all UI components are fully responsive and usable on mobile devices
- **Dark Mode Support**: MUST implement dark mode that follows Docusaurus theme and adapts to system preferences
- **Skeleton Loading States**: MUST provide skeleton loading states for content personalization and chat responses
- **Accessibility (ARIA)**: MUST include proper ARIA labels and accessibility attributes for all interactive components

### Deployment & Production Readiness
- **Environment Variables**: MUST document all required environment variables including API keys, database URLs, and service endpoints
- **Secret Management**: MUST implement secure secret management using platform-specific solutions (Railway, Vercel, and Fly.io etc.)
- **Backend Deployment**: MUST support deployment on Railway, Vercel, and Fly.io with auto-scaling capabilities
- **Connection Pooling**: MUST implement Neon Postgres connection pooling for optimal database performance
- **Caching Strategy**: MUST implement multi-level caching (CDN, API responses, database queries) for performance
- **Health Checks**: MUST provide health check endpoints for monitoring and auto-healing capabilities
- **Security Headers**: MUST implement proper security headers and CSP policies for web security

### Analytics & Observability
- **Event Logging**: MUST log user interactions including chat queries, personalization usage, language preferences, and content engagement
- **Performance Metrics**: MUST track response times, error rates, API usage, and database query performance
- **User Analytics**: MUST monitor user behavior patterns, chapter engagement, and feature adoption rates
- **RAG Analytics**: MUST track retrieval quality, answer relevance, and user satisfaction with responses
- **Error Tracking**: MUST implement error tracking with detailed context and automatic alerting
- **Dashboard Integration**: MUST provide analytics dashboards for monitoring system health and user engagement

### Directory Structure and Module Responsibilities
- **main.py**: MUST initialize the FastAPI app, register routers, and enable CORS restricted to frontend origins
- **clients.py**: MUST handle initialization of singleton clients for FastEmbed, Qdrant Cloud (using QDRANT_URL and QDRANT_API_KEY), and Gemini client (using GOOGLE_API_KEY) with secure connections, appropriate timeouts, and retries
- **rag_service.py**: MUST orchestrate the full RAG workflow - taking input from frontend, generating embeddings, querying Qdrant, constructing prompts, calling Gemini, and assembling responses
- **chatkit_adapter.py**: MUST format messages for chatkit-python adapter and stream tokens to frontend when streaming mode is used
- **schemas.py**: MUST define Pydantic request and response models
- **config.py**: MUST manage environment variables, logging, and secret configuration
- **exceptions.py**: MUST define custom exceptions for input validation errors, upstream service failures, and timeouts
- **utils.py**: MUST provide helper functions for building prompts, sanitizing text, and implementing rate-limiting
- **tests/**: MUST contain unit and integration test stubs for verifying each module and workflow step

### RAG Pipeline Implementation
- **Input Validation**: MUST validate and sanitize user input upon receiving POST /api/rag-chat requests, optionally handling session information
- **Embedding Generation**: MUST generate embeddings using FastEmbed for sanitized text, producing vector representations for querying
- **Vector Search**: MUST query Qdrant Cloud for top matching chunks with metadata (source IDs, chunk offsets, similarity scores)
- **Prompt Construction**: MUST assemble retrieved chunks into RAG prompt according to relevance/recency/priority while respecting Gemini's token limits and including system instructions for grounding, citations, and safe fallbacks
- **LLM Response**: MUST send prompt to Gemini in either non-streaming mode (complete response) or streaming mode (incremental tokens via chatkit-python)
- **Response Processing**: MUST clean LLM output, attach citations, compute metadata (latency, retrieval scores), and return JSON response to frontend
- **Optional Endpoints**: MUST include GET /api/history for conversation history, POST /api/feedback for user feedback, and GET /api/health for health checks

All RAG pipeline implementation details MUST follow Context7 documentation exactly.

### Error Handling and Resilience
- **Input Validation Errors**: MUST return HTTP 400 with descriptive messages
- **Upstream Service Errors**: MUST return HTTP 502/503 for Qdrant/Gemini failures with retry guidance
- **Timeouts**: MUST return HTTP 504 with partial results if applicable
- **Circuit Breaker**: MUST implement circuit-breaker policies and exponential backoff for resilience
- **Authentication Errors**: MUST return appropriate HTTP codes (401, 403) for authentication and authorization failures
- **Onboarding Validation**: MUST return HTTP 400 for invalid onboarding data with descriptive error messages

All error handling and resilience implementation details MUST follow Context7 documentation exactly.

### Observability and Security
- **Structured Logging**: MUST implement structured logging with request IDs, session IDs, and external call timings
- **Metrics Collection**: MUST monitor request rate, error rate, latency per component, retrieval quality, and token usage
- **Security Measures**: MUST store secrets in environment variables, enforce HTTPS for external calls, implement CORS restricted to frontend origins, and apply rate limiting per IP/session
- **Authentication**: MUST implement Better Auth for user authentication with two-step account creation process; sessions must be securely managed with proper validation

All observability and security implementation details MUST follow Context7 documentation exactly.

### Performance and Scalability
- **Resource Profiles**: MUST consider CPU and memory profiles for FastEmbed operations
- **Qdrant Tuning**: MUST tune search parameters (nprobes, indexing) for optimal performance
- **Streaming Decisions**: MUST determine when to use streaming vs non-streaming Gemini calls based on use case
- **Horizontal Scaling**: MUST support horizontal scaling of FastAPI workers behind load balancer
- **Caching Strategy**: MUST implement caching for frequently requested embeddings or top results
- **Database Scaling**: MUST leverage Neon's autoscaling capabilities to handle high-concurrency scenarios for multiple simultaneous users
- **Query Optimization**: MUST optimize database queries for rapid retrieval of user metadata and personalization data
- **Connection Pooling**: MUST implement efficient connection pooling for Neon database access

All performance and scalability implementation details MUST follow Context7 documentation exactly.

### Testing Strategy
- **Unit Tests**: MUST include tests for RAG logic with mocked embedding, Qdrant, and Gemini clients
- **Integration Tests**: MUST test against staging services to verify end-to-end functionality
- **E2E Tests**: MUST simulate full frontend-to-backend flow including failure scenarios (empty context, long queries, API errors)
- **Database Tests**: MUST include tests for Neon database operations, transactional integrity during signup/onboarding, and personalization logging
- **Concurrency Tests**: MUST test high-concurrency scenarios for multiple simultaneous users accessing Neon
- **Security Tests**: MUST verify data encryption, access controls, and proper authentication for Neon connections
- **Multilingual Tests**: MUST include tests for language toggle functionality, translation accuracy, language detection, and bilingual chatbot responses
- **Translation Tests**: MUST verify Urdu translations preserve formatting, headings, and code blocks correctly
- **Language Switching Tests**: MUST test rapid language switching and UI element updates
- **RAG Mode Tests**: MUST include tests for both Selected-Text RAG Mode and Standard RAG Mode
- **Qdrant Integration Tests**: MUST verify similarity search functionality and result accuracy
- **FastEmbed Tests**: MUST validate query embedding generation and quality
- **Gemini Response Tests**: MUST verify response quality and relevance for both RAG modes
- **Environment Configuration Tests**: MUST test loading of all API credentials from .env file
- **Dual-Mode Switching Tests**: MUST verify seamless switching between RAG modes
- **Personalization Tests**: MUST include tests for chapter personalization based on user profile data
- **Content Adaptation Tests**: MUST verify content modifications based on software_background and hardware_os
- **Personalization Storage Tests**: MUST verify saving and restoring personalized content in Neon database
- **Docusaurus Compatibility Tests**: MUST verify frontend compatibility with Docusaurus SSR/CSR patterns
- **State Persistence Tests**: MUST verify chat state maintenance across page navigation
- **Mobile Responsiveness Tests**: MUST verify UI functionality on various mobile devices and screen sizes
- **Dark Mode Tests**: MUST verify theme switching and system preference adaptation
- **Accessibility Tests**: MUST verify ARIA labels and accessibility compliance
- **Deployment Tests**: MUST verify successful deployment across different platforms (Railway, Vercel, and Fly.io)
- **Environment Configuration Tests**: MUST verify proper loading of all environment variables
- **Analytics Tests**: MUST verify accurate logging of user interactions and system metrics
- **Performance Monitoring Tests**: MUST verify tracking of response times and error rates
- **Error Tracking Tests**: MUST verify comprehensive error logging and alerting

All testing strategy implementation details MUST follow Context7 documentation exactly.

### Extensibility
- **Model Swapping**: MUST support adding new embedding models as alternatives to FastEmbed
- **Search Filters**: MUST allow additional search filters (by chapter, page number, content type)
- **Analytics**: MUST support analytics endpoints for user feedback and retraining signals

All extensibility implementation details MUST follow Context7 documentation exactly.
