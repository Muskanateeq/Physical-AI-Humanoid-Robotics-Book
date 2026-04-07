# Feature Specification: Docusaurus Integrated RAG Chatbot

**Feature Branch**: `004-docusaurus-rag-chatbot`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "You are a senior AI architect and full-stack engineer.

Building an “Integrated RAG Chatbot” inside my **Docusaurus book website**.

IMPORTANT:
My book is **already embedded and stored in Qdrant Cloud Vector Database**.
You DO NOT need to write ingestion or embedding code.

Tech Stack:

* ChatKit → Frontend UI (inside Docusaurus, JavaScript SDK)
* ChatKit → Backend integration (Python SDK)
* FastAPI → Backend
* Gemini Free → Main LLM (instead of OpenAI)
* OpenAI-Agents-style architecture → For reasoning, memory, and tool calling (but implemented using Gemini)
* Qdrant Cloud → Book vector database (already populated)
* Neon Serverless Postgres → Chat history storage

You must clearly separate:

1. ChatKit = Frontend (Docusaurus UI only, JavaScript SDK)
2. Agent Logic = Backend (FastAPI + Gemini + ChatKit Python SDK)

SYSTEM RULES:

1. The chatbot MUST answer ONLY from my book content stored in Qdrant.
2. If the answer is not f
1. When a user selects (highlights) ANY text from the book page and opens the chatbot:

   * The selected text must be automatically captured
   * The question must be answered ONLY based on the selected text
   * No external knowledge
   * No Qdrant query in this mode
2. If answer is not contained in selected text, reply exactly:
   "This information is not available in the book."

ChatKit integration:

* **Frontend**: ChatKit JavaScript SDK
  - Renders chat UI, handles floating icon, History toggle, Clear Chat, Like/Dislike/Copy, Voice Mode button
  - Captures selected text for Selected-Text Mode
  - Sends chat requests to backend Python SDK
* **Backend**: ChatKit Python SDK
  - Receives frontend ChatKit requests
  - Handles /chat (RAG), /select-and-chat (selected-text), /history, /feedback
  - Integrates with Gemini agent
  - Stores chat messages, feedback, and voice mode preference in Neon Postgres
  - Generates voice output for answers when requested

WHAT YOU MUST GENERATE (FULL WORKING CODE, NOT PSEUDO):

1. Final system architecture (text diagram)
2. Complete folder structure (frontend + backend)
3. FastAPI backend including:

   * /chat endpoint (RAG with Qdrant)
   * /select-and-chat endpoint (ONLY selected text)
   * /history endpoint (from Neon Postgres)
   * /feedback endpoint (like/dislike)
   * /voice endpoint (generate robotic voice from answer)

4. Gemini Free API integration for Agent logic
5. Qdrant Cloud retrieval & filtering logic (NO insertion)
6. SQL schema for Neon Postgres (chat history + feedback + timestamp + voice mode)
7. “OpenAI-style” Agent System Prompt rewritten for Gemini
8. ChatKit frontend code for Docusaurus including:

   * Floating robotic icon on all pages
   * Chat window with History toggle
   * Selected text capture logic
   * Like / Dislike / Copy functionality
   * Voice Mode toggle button + voice playback
   * FastAPI API connection (chat + feedback + voice)

9. .env file example for:

   * Gemini
   * Qdrant
   * Neon

10. Deployment guide:

* Local setup
* Production (Vercel / VPS / Railway / Render)

CONSTRAINTS:

* DO NOT use LangChain
* DO NOT generate embedding scripts
* USE ONLY:
  ChatKit JS frontend + ChatKit Python backend + FastAPI + Gemini + Qdrant + Neon
* Must follow best security practices
* Must scale for many users

FINAL GOAL:
A fully-functional **book-only** RAG chatbot with TWO MODES:

1. Normal Mode → Uses Qdrant to answer from book
2. Selected Text Mode → Uses ONLY user-highlighted text

Additionally:

* User can enable **Voice Mode** for robotic audio output
* Voice Mode plays answer simultaneously with text
* UI shows voice icon for playback and toggle
* Fully integrated with ChatKit JS frontend and ChatKit Python backend

Now generate EVERYTHING in full code and full steps.
Start with architecture, then give full code section by section."

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Basic RAG Chatbot Interaction (Priority: P1)

As a reader browsing the Docusaurus book website, I want to interact with an AI chatbot that answers questions based solely on the book content stored in Qdrant, so that I can get accurate information directly from the book without needing to search through pages.

**Why this priority**: This is the core functionality of the RAG chatbot - providing book-specific answers using Qdrant vector database. Without this, the feature has no value.

**Independent Test**: Can be fully tested by opening the chatbot, asking a question about book content, and receiving an answer based on the book stored in Qdrant. This delivers the core value of having an AI assistant that knows the book content.

**Acceptance Scenarios**:

1. **Given** I am viewing any page of the Docusaurus book, **When** I click the floating chatbot icon and ask a question about book content, **Then** the chatbot returns an answer based on information retrieved from the Qdrant vector database.

2. **Given** I ask a question not covered in the book content, **When** I submit the question, **Then** the chatbot responds with "This information is not available in the book."

---

### User Story 2 - Selected Text Mode Interaction (Priority: P2)

As a reader who has selected/highlighted specific text on a book page, I want to ask questions about only that selected text so that I can get context-specific answers without the chatbot referencing other parts of the book.

**Why this priority**: This provides an enhanced user experience by allowing focused queries on content the user is currently reading, which is a key differentiator of the system.

**Independent Test**: Can be fully tested by selecting text on a book page, opening the chatbot, asking a question, and receiving an answer based only on the selected text without querying the broader book content in Qdrant.

**Acceptance Scenarios**:

1. **Given** I have selected/highlighted text on a book page, **When** I open the chatbot, the selected text is automatically captured and sent as context, **Then** the chatbot responds using only the selected text and does not query Qdrant for broader book content.

2. **Given** I have selected text that does not contain the answer to my question, **When** I ask the question about the selected text, **Then** I receive the response "This information is not available in the book."

---

### User Story 3 - Chat History and Feedback (Priority: P3)

As a user, I want to maintain my conversation history and provide feedback on answers so that I can have continuous conversations and help improve the chatbot's responses.

**Why this priority**: This enhances user engagement and provides valuable data for improving the system, while also allowing for better contextual conversations.

**Independent Test**: Can be fully tested by having a multi-turn conversation, checking history is preserved, and providing feedback on responses that gets recorded in the database.

**Acceptance Scenarios**:

1. **Given** I am in an active chat session, **When** I ask multiple questions, **Then** all messages are stored in the Neon Postgres database and can be retrieved for history viewing.

2. **Given** I have received an answer from the chatbot, **When** I click the like/dislike button, **Then** my feedback is stored in the database and acknowledged to me.

---

### User Story 4 - Voice Mode Functionality (Priority: P3)

As a user, I want to enable voice mode to hear answers audibly so that I can consume content in multiple formats, especially when reading is not convenient.

**Why this priority**: This adds accessibility and multi-modal interaction capabilities, enhancing the user experience for different use cases.

**Independent Test**: Can be fully tested by enabling voice mode, asking a question, and hearing the response audibly while it's displayed in text.

**Acceptance Scenarios**:

1. **Given** I have enabled voice mode, **When** I receive an answer from the chatbot, **Then** I hear the answer played back audibly while it's displayed in text.

2. **Given** I have voice mode enabled, **When** I toggle it off, **Then** subsequent answers are not played audibly.

---

### Edge Cases

- What happens when the Qdrant database is temporarily unavailable during a normal mode query?
- How does the system handle very long selected text (e.g., multiple paragraphs) in selected-text mode?
- What happens when the Gemini API is rate-limited or unavailable?
- How does the system handle concurrent users asking questions simultaneously?
- What happens when a user submits an empty or malformed query?
- How does the system handle extremely long responses that might overwhelm the user?
- What happens if the Neon database is temporarily unavailable for storing chat history?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST provide a floating chatbot icon on all Docusaurus book pages that opens a chat interface when clicked
- **FR-002**: System MUST allow users to ask questions in a chat interface and receive answers based only on book content stored in Qdrant
- **FR-003**: System MUST capture selected/highlighted text from the current book page when the chatbot is opened in selected-text mode
- **FR-004**: System MUST provide two distinct modes: Normal Mode (queries Qdrant) and Selected Text Mode (uses only selected text)
- **FR-005**: System MUST return "This information is not available in the book" when the answer is not found in the specified source (Qdrant in normal mode or selected text in selected-text mode)
- **FR-006**: System MUST store all chat history in Neon Postgres database with timestamps, user session information, and message metadata
- **FR-007**: System MUST provide voice output capability that plays answers audibly when voice mode is enabled
- **FR-008**: System MUST allow users to provide feedback (like/dislike) on answers which is stored in the database
- **FR-009**: System MUST integrate with Gemini Free API to generate intelligent, contextually relevant responses based on provided context
- **FR-010**: System MUST provide a history toggle feature to view previous conversations and maintain conversation context
- **FR-011**: System MUST include copy functionality to allow users to copy answers to clipboard with one click
- **FR-012**: System MUST maintain voice mode preference across user sessions using browser storage
- **FR-013**: System MUST implement proper error handling and display user-friendly messages when services are unavailable
- **FR-014**: System MUST securely manage API keys and database credentials using environment variables
- **FR-015**: System MUST implement rate limiting to handle concurrent users without performance degradation
- **FR-016**: System MUST integrate ChatKit JavaScript SDK for frontend UI and ChatKit Python SDK for backend processing
- **FR-017**: System MUST provide FastAPI endpoints for /chat (RAG), /select-and-chat (selected-text), /history, /feedback, and /voice
- **FR-018**: System MUST retrieve and filter relevant content from Qdrant Cloud vector database based on user queries
- **FR-019**: System MUST use OpenAI-Agent-style architecture implemented with Gemini for reasoning, memory, and tool calling
- **FR-020**: System MUST provide a system prompt optimized for book content Q&A that works with Gemini

### Key Entities *(include if feature involves data)*

- **ChatSession**: Represents a user's conversation with the chatbot, containing metadata like session ID, start time, user identifier (if available), and active status
- **ChatMessage**: Represents individual messages in a conversation, including content, sender (user/assistant), timestamp, associated session ID, and message type (query/response)
- **UserFeedback**: Represents user feedback on specific messages, including feedback type (like/dislike), optional comment, timestamp, message ID reference, and user identifier
- **VoicePreference**: Represents user's preference for voice mode (enabled/disabled), stored per session/user and persisted in browser storage or database

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can initiate a chat conversation and receive a relevant answer within 5 seconds for 90% of queries
- **SC-002**: At least 85% of user queries receive answers based on book content rather than the "not available" response
- **SC-003**: Users can successfully use both Normal Mode and Selected Text Mode with 95% accuracy in mode switching
- **SC-004**: The system can handle at least 100 concurrent users without significant performance degradation
- **SC-005**: 90% of users successfully complete their first chat interaction (send a message and receive a response)
- **SC-006**: Users provide feedback on at least 15% of chatbot responses when the feedback option is available
- **SC-007**: Voice mode functionality works correctly for 98% of user requests when enabled
- **SC-008**: The system maintains a 99% uptime during regular business hours
- **SC-009**: Selected text capture works correctly for 95% of user selections across different browsers and devices
- **SC-010**: The system correctly responds with "This information is not available in the book" for 100% of queries with no relevant book content