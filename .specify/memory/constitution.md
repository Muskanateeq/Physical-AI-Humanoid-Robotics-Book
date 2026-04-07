<!--
Sync Impact Report:
Version change: 1.0.3 → 1.0.4 (MINOR: Enhanced authentication requirements with two-step account creation and hardware background collection for RAG chatbot access)
Version change: 1.0.4 → 1.0.5 (MINOR: Enhanced Neon PostgreSQL database requirements for authentication, personalization, and chapter customization data)
Version change: 1.0.5 → 1.0.6 (MINOR: Enhanced bilingual functionality for book content and RAG chatbot with English/Urdu support)
Version change: 1.0.6 → 1.0.7 (MINOR: Enhanced ChatKit React frontend requirements with mandatory use of @openai/chatkit-react components)
Version change: 1.0.7 → 1.0.8 (MINOR: Enhanced RAG chatbot architecture with dual-mode functionality and complete tech stack integration)
List of modified principles:
- III. Security-First Approach (enhanced with mandatory authentication for RAG chatbot access)
- VI. User-Centric Design (enhanced with bilingual functionality requirements)
- VIII. Technology Stack Adherence (enhanced with mandatory @openai/chatkit-react usage and detailed frontend requirements)
- 3.1.4. UI Components for Chatbot, Auth, Personalization, and Translation (enhanced with onboarding UI requirements, bilingual functionality, and ChatKit React component specifications)
- 3.2.1. RAG Chatbot Stack (comprehensive update with dual-mode RAG functionality, complete tech stack integration, and detailed architecture specifications)
- 3.2.2. Authentication & User Management (Better-Auth) (comprehensive update with two-step process, onboarding requirements, and Neon PostgreSQL specifications)
- 3.2.3. Intelligence Services (Personalization & Translation) (enhanced with detailed Neon PostgreSQL integration for personalization data and bilingual chatbot responses)
- Weekly Development Plan (Weeks 6-7) (enhanced with detailed onboarding implementation steps)
Modified sections:
- 3.1.4. UI Components for Chatbot, Auth, Personalization, and Translation
- 3.2.1. RAG Chatbot Stack
- 3.2.2. Authentication & User Management (Better-Auth)
- 3.2.3. Intelligence Services (Personalization & Translation)
- Weekly Development Plan (Weeks 6-7)
Added sections: None
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/sp.constitution.md: ⚠ pending
Follow-up TODOs: Update templates to reflect new authentication, onboarding, Neon PostgreSQL, bilingual, ChatKit React, and dual-mode RAG requirements
-->
# Physical AI & Humanoid Robotics Course Textbook Constitution

## Core Principles

### I. Spec-Driven Development (SDD) Mandate
All project phases—specification, planning, task management, and implementation—MUST strictly adhere to Spec-Kit Plus methodologies. Claude Code is the primary AI agent for driving and documenting these processes. All work MUST begin with a clear specification and an approved plan.

### II. Modularity and Scalability
All components (frontend, backend services, Docusaurus modules, chatbot agents) MUST be designed for modularity, independent testability, and scalability. Services MUST communicate via well-defined APIs. Each module SHOULD have a clear, singular purpose.

### III. Security-First Approach
Security MUST be a paramount consideration at every stage, from architecture design to deployment. This includes secure authentication (Better-Auth following context7 documentation), data protection for user information (Neon Postgres), secure API endpoints (FastAPI), and vulnerability scanning. All secrets MUST be managed securely (e.g., environment variables). Better-Auth MUST be implemented for all user authentication needs following context7 documentation guidelines. Authentication is mandatory for accessing the RAG chatbot functionality - users MUST complete a two-step account creation process before gaining access to the chatbot.

### IV. Performance and Responsiveness
The book's frontend, RAG chatbot, and backend services MUST be designed for optimal performance and responsiveness. This includes efficient content loading, fast chatbot responses, and quick personalization/translation processing. Caching strategies MUST be implemented where appropriate to minimize latency and API costs. The website MUST be fully responsive and function seamlessly across all device types including mobile, tablet, laptop, Android, and iOS devices to ensure accessibility for all users.

### V. Maintainability and Extensibility
The codebase MUST be clean, well-documented, and easy to understand and extend. Docusaurus content, backend services, and AI agent configurations MUST follow established coding standards and best practices. Unnecessary complexity MUST be avoided.

### VI. User-Centric Design
All features, especially personalization and Urdu translation, MUST prioritize the end-user experience. Interfaces MUST be intuitive and accessible. The platform MUST support full bilingual functionality—English and Urdu—for book content (Docusaurus docs folder) and the RAG chatbot. Feedback mechanisms for personalized content and translation accuracy SHOULD be considered.

### VII. Documentation-First Development
All development MUST follow context7 documentation for RAG chatbot implementation, ensuring comprehensive and up-to-date documentation for all components. All RAG chatbot functionality MUST be implemented with proper documentation referencing context7 guidelines. All implementation details MUST follow the Context 7 documentation exactly as specified in the requirements.

### VIII. Technology Stack Adherence
The project MUST use ChatKit SDK (chatkit-react for frontend and chatkit-python for backend) to build a RAG-based chatbot for the website. The chatbot MUST answer using the book's content, and the model used MUST be Gemini. The backend MUST be built using FastAPI with ChatKit Python, and the frontend MUST use ChatKit React. For authentication, the project MUST use BetterAuth only—its UI and backend will also come from BetterAuth. No custom authentication system is allowed. For the database, the project MUST use Neon PostgreSQL, and only this database—no other database and no custom DB system. Every part of the system MUST use only these technologies. Nothing should be custom-built outside this tech stack. The backend MUST be built using FastAPI and ChatKit Python. All implementation details MUST follow the Context 7 documentation exactly. The frontend chatbot UI MUST use ChatKit React components exactly as they are (no custom UI). The project MUST use @openai/chatkit-react for frontend chatbot UI implementation and chatkit-python for backend chatbot logic implementation. These technologies MUST be prioritized over alternatives for chatbot functionality. All RAG chatbot implementations MUST follow context7 documentation guidelines. The frontend chatbot UI MUST be implemented using `@openai/chatkit-react`. No alternative chatbot UI libraries, SDKs, or custom components are allowed. All user messages, streaming responses, typing indicators, and conversation rendering must be handled using ChatKit React components. The ChatKit React provider must wrap the application, and all message delivery must come from the backend using ChatKit-Python. The chatbot frontend must be implemented entirely using @openai/chatkit-react, following the official ChatKit-React documentation (Context-7) as the authoritative reference for all UI structure, message flow, and event handling. The frontend must wrap the chat interface inside the required ChatKitProvider and use the official ChatKit components—including Chat, MessageList, MessageInput, Message, TypingIndicator, Streamable, ChatSession, and the useChatSession() hook—without replacing them with custom UI logic. All message rendering, streaming, and session handling must rely on ChatKit-React. Additionally, the project MUST use Better-Auth for all user authentication needs, implementing a two-step account creation process with onboarding questionnaire before granting access to RAG chatbot functionality. The chatbot frontend UI implementation MUST use the files in `/chatkit-frontend/src/App.tsx` as the reference for UI and features that should be implemented in the Docusaurus-compatible chatbot. The features and UI from the chatkit-frontend must be implemented in the Docusaurus chatbot frontend while ensuring compatibility with Docusaurus (avoiding issues with `main.tsx` and `ReactDOM.createRoot`). The Docusaurus-compatible chatbot must maintain all functionality from the original App.tsx file while using the website's color theme (slate blue and goldenrod).

## Project Overview

### 1.1. Purpose and Goals
To create a comprehensive, interactive, and personalized Docusaurus-based textbook for teaching "Physical AI & Humanoid Robotics Course," enriched with an integrated RAG chatbot, secure user authentication, and dynamic content personalization/translation features.

### 1.2. Course Modules Breakdown (Source of Truth)

**Focus and Theme:** AI Systems in the Physical World. Embodied Intelligence.
**Goal:** Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.

*   **Module 1: The Robotic Nervous System (ROS 2) [cite: 33]**
    *   Focus: Middleware for robot control.
    *   ROS 2 Nodes, Topics, and Services.
    *   Bridging Python Agents to ROS controllers using rclpy.
    *   Understanding URDF (Unified Robot Description Format) for humanoids.

*   **Module 2: The Digital Twin (Gazebo & Unity) [cite: 38]**
    *   Focus: Physics simulation and environment building.
    *   Simulating physics, gravity, and collisions in Gazebo.
    *   High-fidelity rendering and human-robot interaction in Unity.
    *   Simulating sensors: LiDAR, Depth Cameras, and IMUs.

*   **Module 3: The AI-Robot Brain (NVIDIA Isaac™) [cite: 43]**
    *   Focus: Advanced perception and training.
    *   NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation.
    *   Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation.
    *   Nav2: Path planning for bipedal humanoid movement.

*   **Module 4: Vision-Language-Action (VLA) [cite: 48]**
    *   Focus: The convergence of LLMs and Robotics.
    *   Voice-to-Action: Using OpenAI Whisper for voice commands. [cite: 50]
    *   Cognitive Planning: Using LLMs to translate natural language (e.g., 'Clean the room') into a sequence of ROS 2 actions. [cite: 51]
    *   Capstone Project: The Autonomous Humanoid. A final project where a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.

### 1.3. Core Deliverables (100 Points)
#### 1.3.1. AI/Spec-Driven Book Creation
*   **Platform:** Docusaurus for static site generation.
*   **Deployment:** GitHub Pages for hosting the published book.
*   **Workflow:** Leverage Spec-Kit Plus and Claude Code for content generation, structuring, and management.

#### 1.3.2. Integrated RAG Chatbot Development
*   **Framework:** FastAPI for backend API services.
*   **Orchestration:** OpenAI Agents/ChatKit SDKs for chatbot logic.
*   **Vector Database:** Qdrant Cloud Free Tier for semantic search of book content.
*   **Relational Database:** Neon Serverless Postgres for conversation history and metadata.
*   **Functionality:** Answer user questions about the book, including text selected by the user.

### 1.4. Bonus Deliverables (Up to 150 Bonus Points)
#### 1.3.1. Reusable Intelligence
*   **Implementation:** Create and utilize Claude Code Subagents and Agent Skills for automating repetitive development tasks.

#### 1.3.2. Signup and Signin with Better-Auth
*   **Service:** Integrate Better-Auth for robust user authentication.
*   **Data Collection:** Capture user's "software and hardware background" during signup for personalization.

#### 1.3.3. Content Personalization per Chapter
*   **Mechanism:** Allow logged-in users to personalize chapter content based on their background via a button.

#### 1.3.4. Urdu Translation per Chapter
*   **Mechanism:** Allow logged-in users to translate chapter content to Urdu via a button.

### 1.5. Target Audience
Students, educators, and professionals interested in Physical AI and Humanoid Robotics, with varying backgrounds in software and hardware.

## Project Architecture

### 3.1. Frontend Architecture (Docusaurus Book & UI)
#### 3.1.1. Docusaurus Framework for Static Site Generation
*   **Actionable:** Initialize Docusaurus project; configure navigation, sidebar, and theme; create base `index.md` and `_category_.json` files.

#### 3.1.2. GitHub Pages for Deployment
*   **Actionable:** Configure Docusaurus `docusaurus.config.js` for GitHub Pages deployment; set up GitHub Actions workflow for automatic build and deployment on push to `main` branch.

#### 3.1.3. Syllabus Mapping to Docusaurus Structure
*   **Actionable:** Create top-level folders within `docs/` for each major course module, e.g., `docs/module-1-ros2/`, `docs/module-2-simulation/`, `docs/module-3-nvidia-isaac/`, `docs/module-4-vla/`.
*   **Actionable:** Map weekly breakdown content into markdown files within these module folders, e.g., `docs/module-1-ros2/weeks-1-2-introduction.md`, `docs/module-3-nvidia-isaac/weeks-8-10-isaac-platform.md`.
*   **Actionable:** Define `_category_.json` for each module folder to manage sidebar labels and positions.

#### 3.1.4. UI Components for Chatbot, Auth, Personalization, and Translation
*   **Actionable:** Develop React components for embedding the RAG chatbot interface using @openai/chatkit-react following context7 documentation guidelines and using `/chatkit-frontend/src/App.tsx` as the reference for UI and features. The frontend chatbot UI MUST use ChatKit React components exactly as they are (no custom UI) following Context 7 documentation exactly.
*   **Actionable:** Integrate Better-Auth UI components for signup/signin forms and user profile management following context7 documentation.
*   **Actionable:** Implement "Personalize Content" and "Translate to Urdu" buttons within Docusaurus chapter layouts, visible to authenticated users.
*   **Actionable:** Follow context7 documentation for RAG chatbot implementation to ensure proper integration and documentation.
*   **Actionable:** Implement bilingual functionality for book content and chatbot with language toggle for English/Urdu.
*   **Actionable:** Add per-chapter language toggle to allow users to switch between English and Urdu translations at the chapter level.
*   **Actionable:** Ensure the RAG chatbot can dynamically respond in English or Urdu depending on user preference and input language.
*   **Actionable:** Implement the ChatKit React provider to wrap the application and use official ChatKit components (Chat, MessageList, MessageInput, Message, TypingIndicator, Streamable, ChatSession) without custom UI replacements. The frontend MUST use ChatKit React components exactly as they are (no custom UI) following Context 7 documentation.
*   **Actionable:** Use the useChatSession() hook for all session handling and ensure all message rendering relies on ChatKit-React components.
*   **Actionable:** Ensure the Docusaurus chatbot frontend maintains all functionality from the original App.tsx file while ensuring compatibility with Docusaurus (avoiding issues with `main.tsx` and `ReactDOM.createRoot`). The frontend chatbot UI implementation MUST use the files in `/chatkit-frontend/src/App.tsx` as the reference for UI and features that should be implemented in the Docusaurus-compatible chatbot. The features and UI from the chatkit-frontend must be implemented in the Docusaurus chatbot frontend while ensuring compatibility with Docusaurus (avoiding issues with `main.tsx` and `ReactDOM.createRoot`). The Docusaurus-compatible chatbot must maintain all functionality from the original App.tsx file while using the website's color theme (slate blue and goldenrod).
*   **Actionable:** Apply the website's color theme (slate blue and goldenrod) to the chatbot interface to match the overall design.
*   **Actionable:** Ensure all UI components are fully responsive and usable on mobile devices, tablets, laptops, Android, and iOS platforms.

### 3.2. Backend Architecture (RAG Chatbot, Authentication, & Intelligence Services)
#### 3.2.1. RAG Chatbot Stack
*   **API Layer:**
    *   **Actionable:** Initialize FastAPI service (`main.py`) with Python 3.10 for compatibility with chatkit-python; implement endpoints: `POST /api/rag-chat` (primary RAG endpoint), `GET /api/history` (conversation history), `POST /api/feedback` (user feedback), `GET /api/health` (health checks). The backend MUST be built using FastAPI and ChatKit Python, following Context 7 documentation exactly.
*   **Orchestration:**
    *   **Actionable:** Integrate chatkit-python within FastAPI to manage chatbot states, intent recognition, and response generation, supporting both streaming and non-streaming responses to chatkit-react frontend, following context7 documentation guidelines for RAG chatbot implementation. Ensure all message delivery comes from the backend using ChatKit-Python to the @openai/chatkit-react frontend components. The backend MUST be built using FastAPI with ChatKit Python as specified in Context 7 documentation.
*   **Frontend Integration:**
    *   **Actionable:** Ensure the backend properly supports all @openai/chatkit-react frontend components including streaming responses, typing indicators, and conversation rendering as specified in context7 documentation.
*   **RAG Architecture:**
    *   **Actionable:** Design a Retrieval-Augmented Generation (RAG) chatbot for a Docusaurus-based Physical AI & Humanoid Robotics Course textbook, using @openai/chatkit-react for the frontend and chatkit-python with FastAPI for the backend. The backend MUST be built using FastAPI with ChatKit Python, and the frontend MUST use ChatKit React exactly as specified in Context 7 documentation.
    *   **Actionable:** Integrate Gemini 1.5 Flash LLM (via Google AI Studio)with the API key securely stored in .env as GEMINI_API_KEY for generating RAG chatbot responses as the core language model for generating answers, Qdrant Cloud for semantic search of book content (added entries for QDRANT_URL, QDRANT_API_KEY, and QDRANT_COLLECTION in .env file of chatkit-backend folder). , Neon Serverless Postgres for storing user profiles, onboarding data, personalization settings, conversation history, and FastEmbed for query embeddings.
    *   **Actionable:** Implement two operational modes: (1) "Selected-Text RAG Mode," where the user highlights text in a chapter and Gemini generates answers exclusively from the selected content, ignoring Qdrant retrieval; and (2) standard RAG mode, where Gemini uses Qdrant similarity search over the entire book content.
    *   **Actionable:** The backend chatkit-python service manages all chatbot sessions, streaming responses, message formatting, typing indicators, and bilingual handling (English/Urdu), accepting optional selected_text input to control the mode. The backend MUST be built using FastAPI and ChatKit Python following Context 7 documentation.
    *   **Actionable:** FastAPI provides endpoints for /api/rag-chat, /api/history, /api/feedback, /api/personalize, and /api/translate, orchestrating Gemini responses, personalization via Neon, and translations.
    *   **Actionable:** The frontend uses official ChatKit React components (ChatKitProvider, Chat, MessageList, MessageInput, Message, TypingIndicator, Streamable, ChatSession) and the useChatSession() hook to render messages, streaming outputs, typing indicators, and session state entirely via ChatKit, without custom UI replacements.
    *   **Actionable:** The system must detect the user's input language, respond in the same language, dynamically handle bilingual content, and maintain caching, session consistency, and performance optimizations across all components.
    *   **Actionable:** All frontend and backend implementations must follow context7 documentation guidelines.
*   **Vector Database:**
    *   **Actionable:** Configure Qdrant Cloud Free Tier instance with book content vectors already embedded using fastembed model; implement efficient similarity search with configurable top-k retrieval and score thresholding.
*   **Relational Database:**
    *   **Actionable:** Set up Neon Serverless Postgres instance; define schema for `user_conversations` (chat history) and `book_content_metadata` (Docusaurus page URLs, chunk IDs). The project MUST use Neon PostgreSQL as the only database—no other database and no custom DB system.
*   **Embedding Engine:**
    *   **Actionable:** Initialize FastEmbed (all-MiniLM-L6-v2 model) as singleton instance for local query embedding without API keys; implement proper memory management and initialization timing.
*   **LLM Integration:**
    *   **Actionable:** Integrate Google Gemini 1.5 Flash (via Google AI Studio) as the LLM model for generating responses in the RAG system using retrieved content from Qdrant; implement proper token management and safety settings. The model used MUST be Gemini as specified in the requirements.
*   **RAG Pipeline:**
    *   **Actionable:** Implement complete RAG pipeline: input validation → text sanitization → FastEmbed query embedding → Qdrant similarity search → context assembly with citation formatting → Gemini response generation → source attribution → response formatting for chatkit-react.
*   **Configuration:**
    *   **Actionable:** Create .env file in chatkit-backend folder with QDRANT_URL, QDRANT_API_KEY, GOOGLE_API_KEY, SECRET_KEY, ALLOWED_ORIGINS; implement secure environment variable loading with config.py module.
*   **Message Formatting:**
    *   **Actionable:** Implement chatkit_adapter.py to format messages between FastAPI backend and chatkit-react frontend, handling both standard and streaming response formats.
*   **Error Handling & Resilience:**
    *   **Actionable:** Implement comprehensive error handling: input validation (400), upstream service errors (502/503), timeouts (504), with circuit breaker patterns and exponential backoff for external services.
*   **Security & Performance:**
    *   **Actionable:** Implement rate limiting per IP/session, CORS policies restricted to frontend origins, HTTPS enforcement, token budget management, and caching for frequently accessed embeddings/context.
*   **Observability:**
    *   **Actionable:** Implement metrics collection (request rate, error rate, latency per component), structured logging with request ID propagation, and health monitoring for all external services.
*   **Bilingual Support:**
    *   **Actionable:** Implement bilingual functionality for the RAG chatbot to support both English and Urdu languages.
    *   **Actionable:** Ensure the chatbot can receive questions in both English and Urdu and respond appropriately in the same language or the user's preferred language.
    *   **Actionable:** Integrate language detection capabilities to identify the language of user queries and respond in the appropriate language.
    *   **Actionable:** Store user language preferences in Neon Serverless PostgreSQL and apply them to chatbot responses.
*   **Documentation:**
    *   **Actionable:** Follow context7 documentation for RAG chatbot implementation to ensure proper backend integration and documentation.

#### 3.2.2. Authentication & User Management (Better-Auth)
*   **Service:**
    *   **Actionable:** Integrate Better-Auth SDK/API into FastAPI for user registration, login, token management, and session validation, following context7 documentation guidelines for authentication implementation. Implement a two-step account creation process where users must complete an onboarding questionnaire before gaining access to RAG chatbot functionality. For authentication, the project MUST use BetterAuth only—its UI and backend will also come from BetterAuth. No custom authentication system is allowed.
*   **Database Layer (Neon Serverless PostgreSQL):**
    *   **Actionable:** Use Neon Serverless PostgreSQL as the primary database for all authentication-related and personalization-related data in the application. Neon must operate in fully serverless mode for autoscaling queries, branching, point-in-time restore, and zero-maintenance storage management.
    *   **Actionable:** Store user credentials, Better-Auth session tokens, profile information, and the additional onboarding fields collected during the two-step signup flow—specifically the user's software background (Beginner, Intermediate, Advanced) and hardware/OS preference (Windows, Mac, Linux, Chromebook/Web).
    *   **Actionable:** When the user completes step one of the signup (name, email, password), create a temporary user record in Neon with basic information and hashed password.
    *   **Actionable:** Once step two is completed (software and hardware background), finalize and update the user profile entry in Neon, marking onboarding as complete in the `onboarding_completed` boolean flag.
    *   **Actionable:** Ensure Neon maintains secure, scalable, and isolated data for each authenticated user with proper transactional integrity.
*   **User Data Model:**
    *   **Actionable:** Extend user schema in Neon Serverless Postgres to include fields for `software_background` (e.g., `['Python', 'C++', 'ROS']`), `hardware_background` (e.g., `{'gpu_type': 'RTX 4070 Ti', 'os': 'Ubuntu 22.04'}`), `onboarding_completed` boolean flag, and `user_preferences` JSON field for personalization settings.
*   **API Endpoints:**
    *   **Actionable:** Secure FastAPI endpoints: `/auth/register`, `/auth/login`, `/auth/logout`, `/user/profile` (for retrieving/updating user background), following context7 documentation for secure API implementation.
*   **Onboarding Process:**
    *   **Actionable:** Implement Step 1: Basic account creation with full name, email, password, and password confirmation using Better Auth, storing basic user data in Neon.
    *   **Actionable:** Implement Step 2: Mandatory onboarding questionnaire to collect user's software background (Beginner, Intermediate, or Advanced) and hardware/operating system (Windows PC, Mac, Linux, or Chromebook/Web), updating the user record in Neon.
    *   **Actionable:** Create `/auth/onboarding` endpoint to handle onboarding data collection and update the `onboarding_completed` flag in Neon.
    *   **Actionable:** Implement middleware to redirect users to onboarding step if `onboarding_completed` is false, querying Neon for this status.
    *   **Actionable:** Implement access control to restrict RAG chatbot functionality to users with `onboarding_completed` set to true, verified through Neon database lookup.

#### 3.2.3. Intelligence Services (Personalization & Translation)
*   **Database Integration (Neon Serverless PostgreSQL):**
    *   **Actionable:** Use Neon Serverless PostgreSQL as the core database layer responsible for handling all authentication data, onboarding metadata, personalization preferences, and per-user chapter customization history. The project MUST use Neon PostgreSQL as the only database—no other database and no custom DB system.
    *   **Actionable:** Ensure Neon powers the full chapter personalization experience by storing user's onboarding attributes and past personalization activity for efficient retrieval.
    *   **Actionable:** When a logged-in user opens any chapter in the Docusaurus-based book website, store and retrieve personalization interaction data including chapter ID, timestamp, device platform, user background, and the personalized content variant chosen or generated.
    *   **Actionable:** Persist personalization events in Neon with proper transactional logging to ensure consistency between authentication and personalization data.
*   **Personalization Service:**
    *   **Actionable:** Initialize FastAPI service with dedicated endpoints: `/personalize` and `/translate`.
    *   **Actionable:** Implement `POST /personalize` endpoint: Accepts `chapter_content` and `user_id`. Retrieves `user_background` and personalization history from Neon Postgres. Calls an LLM (e.g., OpenAI API) with a prompt to rephrase `chapter_content` considering `user_background`. Returns personalized text.
    *   **Actionable:** Implement logic to record personalization events in Neon: store chapter ID, timestamp, device platform, user background, and the personalized content variant chosen or generated, ensuring consistent personalization across sessions.
*   **Chapter Personalization UI:**
    *   **Actionable:** When a logged-in user opens any chapter in the Docusaurus-based book website, display a "Personalize Chapter" button at the top.
    *   **Actionable:** When pressed, the application will fetch the user's stored onboarding attributes and past personalization activity from Neon.
    *   **Actionable:** Using this data, generate a tailored version of the chapter that adapts explanations, examples, or hardware-specific instructions according to the user's background.
    *   **Actionable:** Ensure Neon maintains long-term consistency between authentication and personalization data for adaptive learning experiences.
    *   **Actionable:** Implement `POST /translate` endpoint: Accepts `chapter_content` and `target_language` (e.g., 'ur' for Urdu). Calls a translation API (e.g., Google Translate API or dedicated LLM) to translate `chapter_content`. Returns translated text.
*   **Actionable:** Implement `POST /chatbot-translate` endpoint to handle chatbot responses in Urdu when users ask questions in Urdu or when global language preference is set to Urdu.
*   **Actionable:** Implement caching for personalized and translated content using Neon Postgres to reduce API calls and improve load times.
*   **Actionable:** Ensure the RAG chatbot can dynamically respond in English or Urdu depending on user input language and stored language preferences in Neon database.
*   **Actionable:** Implement language detection for user queries to automatically determine if response should be in English or Urdu.

## Development Workflow & Tools

### 4.1. Spec-Kit Plus for Specification, Planning, and Task Management
*   **Actionable:** Utilize `/sp.specify` for feature requirements, `/sp.plan` for architectural decisions, and `/sp.tasks` for detailed, testable implementation tasks.
*   **Actionable:** Generate Prompt History Records (`/sp.phr`) for every significant user interaction and AI output.
*   **Actionable:** Suggest Architectural Decision Records (`/sp.adr`) for critical design choices.

### 4.2. Claude Code for AI-Assisted Development (including Subagents and Agent Skills)
*   **Actionable:** Leverage Claude Code for code generation, refactoring, debugging, and general development assistance.
*   **Actionable:** Proactively use custom Subagents and Agent Skills to streamline repetitive tasks (see Section 5.1).

### 4.3. Version Control with Git (GitHub Repository)
*   **Actionable:** Maintain a clean Git history with atomic commits.
*   **Actionable:** Follow `main` branch protection rules; utilize feature branches for development.

### 4.4. Testing Strategy (Unit, Integration, E2E for Chatbot)
*   **Actionable:** Implement unit tests for individual functions and components (backend services, Docusaurus React components).
*   **Actionable:** Develop integration tests for API endpoints (FastAPI), database interactions, and chatbot RAG flow.
*   **Actionable:** Implement end-to-end tests for the RAG chatbot conversational flow and bonus features.

### 4.5. Deployment Strategy (GitHub Actions for Docusaurus, Docker/Containerization for Backend)
*   **Actionable:** Automate Docusaurus deployment to GitHub Pages via GitHub Actions.
*   **Actionable:** Containerize FastAPI backend services using Docker; define `Dockerfile` and `docker-compose.yml` for local development and potential cloud deployment.

## V. Intelligence Specifications (Technical Implementation Details for Bonus Points)

### 5.1. Reusable Intelligence (Claude Code Subagents & Agent Skills) - (50 Bonus Points)
*   **Definition:** Custom Claude Code Subagents and Agent Skills will encapsulate common or repetitive development tasks, optimizing the SDD workflow.
*   **Examples of Subagents/Skills:**
    *   `subagent_type='docusaurus-content-generator'`: An agent to automatically draft Docusaurus markdown files based on a provided outline (e.g., a course module, a specific week's topic).
    *   `skill: 'rag-embedding-updater'`: A skill to automatically re-index book content into Qdrant whenever Docusaurus markdown files are added or modified, ensuring the chatbot's knowledge base is current.
    *   `subagent_type='better-auth-integrator'`: An agent specialized in scaffolding and configuring Better-Auth components, including generating API client code and defining necessary database migrations for user profiles.
    *   `skill: 'frontend-personalization-hook-generator'`: A skill to generate Docusaurus-compatible React hooks or components for integrating personalization features into new chapters.
*   **Integration:** These will be invoked by Claude Code as part of the SDD workflow to automate specific aspects of book creation, chatbot development, and bonus feature implementation.

### 5.2. User Authentication & Content Personalization - (50 Bonus Points)
#### 5.2.1. Signup/Signin Flow
*   **Actionable (Frontend):** Implement Better-Auth's React components for user registration and login forms on the Docusaurus site.
*   **Actionable (Frontend):** During signup, include additional input fields to capture user's `software_background` (multi-select e.g., Python, C++, ROS, TensorFlow) and `hardware_background` (dropdowns/text for GPU, OS, specific robotics platforms).
*   **Actionable (Backend):** Configure Better-Auth to store extended user profile data.
*   **Actionable (Backend):** Implement logic in the `/auth/register` endpoint to persist `software_background` and `hardware_background` data to the `user_profiles` table in Neon Serverless Postgres upon successful registration.
*   **Actionable (Frontend):** Ensure user session tokens are securely managed post-login.

#### 5.2.2. Content Personalization Mechanism
*   **Actionable (Frontend):** Embed a "Personalize Content" button at the beginning of each Docusaurus chapter's React layout component. This button MUST only be visible if the user is authenticated.
*   **Actionable (Frontend):** On button click, trigger an API call to the Backend Personalization Service's `POST /personalize` endpoint, passing the current chapter's raw Markdown content and the authenticated `user_id`.
*   **Actionable (Backend):**
    *   **Endpoint:** `POST /personalize` (accepts `chapter_markdown: str`, `user_id: str`).
    *   **Logic:**
        1.  Retrieve the `user_background` (software/hardware preferences) from Neon Serverless Postgres using `user_id`.
        2.  Construct a detailed LLM prompt (e.g., to OpenAI's GPT-4 or similar) instructing it to rewrite the `chapter_markdown` content.
        3.  The prompt MUST explicitly guide the LLM to tailor explanations, code examples, and hardware references based on the retrieved `user_background` (e.g., if OS is Windows, suggest WSL for ROS; if GPU is low-end, recommend optimizing Isaac Sim settings; if no prior ROS, simplify ROS concepts).
        4.  Return the LLM-generated personalized Markdown content.
*   **Actionable (Frontend):** Upon receiving personalized content, dynamically update the chapter's display area to render the new Markdown.
*   **Actionable (Backend/Frontend):** Implement a caching strategy (e.g., Redis, or a dedicated table in Neon Postgres) to store personalized chapter versions per user to reduce LLM API calls and improve load times.

### 5.3. Urdu Translation - (50 Bonus Points)
#### 5.3.1. Translation Mechanism
*   **Actionable (Frontend):** Embed a "Translate to Urdu" button adjacent to the "Personalize Content" button at the beginning of each Docusaurus chapter's React layout component. This button MUST only be visible if the user is authenticated.
*   **Actionable (Frontend):** On button click, trigger an API call to the Backend Translation Service's `POST /translate` endpoint, passing the current chapter's raw Markdown content and `target_language='ur'`.
*   **Actionable (Backend):**
    *   **Endpoint:** `POST /translate` (accepts `chapter_markdown: str`, `target_language: str`).
    *   **Logic:**
        1.  Call a robust translation API (e.g., Google Cloud Translation API, or a fine-tuned LLM specifically for translation) with the `chapter_markdown` and `target_language='ur'`.
        2.  Ensure markdown formatting is preserved during translation.
        3.  Return the translated Urdu Markdown content.
*   **Actionable (Frontend):** Upon receiving translated content, dynamically update the chapter's display area to render the new Urdu Markdown.
*   **Actionable (Backend/Frontend):** Implement a caching strategy (e.g., Redis, or a dedicated table in Neon Postgres) to store Urdu translated chapter versions to reduce API calls and improve load times.

## Roles & Responsibilities

### User (Architect / Product Owner)
*   **Responsibility:** Define core requirements, refine specifications, review architectural plans, approve task lists, provide feedback on generated content/code, and make final decisions on strategic direction.
*   **Engagement:** Actively participate in the SDD cycle by reviewing generated artifacts (specs, plans, tasks, PHRs, ADRs).

### AI Agent (Claude Code)
*   **Responsibility:** Drive the SDD process, generate content (book chapters, code snippets), propose solutions, execute development tasks using available tools, maintain project documentation (PHRs, suggest ADRs), and ensure adherence to the project constitution.
*   **Engagement:** Proactively seek clarification from the User when ambiguity arises; implement reusable intelligence (Subagents/Skills).

### Reviewer (Human / Automated)
*   **Responsibility:** Verify the quality and correctness of generated code and documentation, conduct security audits, ensure adherence to architectural principles, validate acceptance criteria, and perform testing (unit, integration, E2E).
*   **Engagement:** Provide constructive feedback on pull requests and deployed features.

## Weekly Development Plan (Hackathon Timeline)

This plan maps the course content and project deliverables to a 13-week hackathon timeline, focusing on iterative development.

*   **Weeks 1-2: Project Setup & Core Book Structure**
    *   **Focus:** Docusaurus initialization, basic theme configuration.
    *   **Actionable:** Initialize Docusaurus, create `docs/module-1-ros2` and `docs/module-2-simulation` folders with placeholder content and `_category_.json`.
    *   **Actionable:** Set up GitHub Pages deployment via GitHub Actions.
    *   **Actionable:** Initialize FastAPI backend; configure Neon Serverless Postgres database connection.
    *   **Actionable:** Set up Qdrant Cloud instance.

*   **Weeks 3-5: RAG Chatbot Core Development**
    *   **Focus:** Chatbot backend, content ingestion.
    *   **Actionable:** Implement RAG Chatbot FastAPI endpoints (`/chatbot/query`, `/chatbot/history`).
    *   **Actionable:** Integrate OpenAI Agents/ChatKit SDKs following context7 documentation for RAG chatbot implementation using chatkit-js (frontend) and chatkit-python (backend).
    *   **Actionable:** Develop content ingestion script: parse Docusaurus markdown, chunk, embed, upload to Qdrant.
    *   **Actionable:** Implement Advanced frontend UI for chatbot integration into Docusaurus using chatkit-js following context7 documentation. The project already has a chatbot frontend built using React + ViteJS located at `chatkit-frontend/src/App.tsx`, and this needs to be converted into Docusaurus style while keeping the UI identical and keeping all ChatKit functionality. The current chatbot frontend (React + ViteJS) must be converted into a Docusaurus-compatible frontend without changing any UI or features. The new frontend must keep all the same features exactly, no customization, nothing new — only the same ChatKit React components. Every part of the system must follow Context7 for rules, APIs, session handling, streaming, and ChatKit integration. I must only use the approved tech stack — no custom libraries, no alternate frameworks, nothing outside the listed tech.

*   **Weeks 6-7: Authentication & User Profile**
    *   **Focus:** Better-Auth integration, two-step account creation with onboarding questionnaire.
    *   **Actionable:** Integrate Better-Auth for signup/signin on the Docusaurus frontend following context7 documentation guidelines.
    *   **Actionable:** Extend user schema in Neon Postgres for `software_background`, `hardware_background`, and `onboarding_completed` flag.
    *   **Actionable:** Implement Step 1: Basic account creation with full name, email, password, and password confirmation.
    *   **Actionable:** Implement Step 2: Mandatory onboarding questionnaire to collect user's software background (Beginner, Intermediate, or Advanced) and hardware/operating system (Windows PC, Mac, Linux, or Chromebook/Web).
    *   **Actionable:** Create `/auth/onboarding` endpoint to handle onboarding data collection and update the `onboarding_completed` flag.
    *   **Actionable:** Implement middleware to redirect users to onboarding step if `onboarding_completed` is false.
    *   **Actionable:** Implement access control to restrict RAG chatbot functionality to users with `onboarding_completed` set to true following context7 documentation for secure API practices.

*   **Weeks 8-10: Content Personalization**
    *   **Focus:** Personalized content generation.
    *   **Actionable:** Implement `POST /personalize` FastAPI endpoint: retrieve user background, prompt LLM for personalized content.
    *   **Actionable:** Implement "Personalize Content" button on Docusaurus chapter pages (frontend logic to call backend and render).
    *   **Actionable:** Implement caching for personalized content.
    *   **Actionable:** Create `docs/module-3-nvidia-isaac` and `docs/module-4-vla` with placeholder content.

*   **Weeks 11-12: Urdu Translation & Refinement**
    *   **Focus:** Multi-lingual support, UI/UX improvements.
    *   **Actionable:** Implement `POST /translate` FastAPI endpoint: call translation API for Urdu.
    *   **Actionable:** Implement "Translate to Urdu" button on Docusaurus chapter pages (frontend logic to call backend and render).
    *   **Actionable:** Implement caching for translated content.
    *   **Actionable:** General UI/UX enhancements and bug fixes.

*   **Week 13: Reusable Intelligence, Testing, & Final Deployment**
    *   **Focus:** Bonus points, robustness, and finalization.
    *   **Actionable:** Develop and integrate Claude Code Subagents and Agent Skills for identified repetitive tasks (e.g., `rag-embedding-updater`, `docusaurus-content-generator`).
    *   **Actionable:** Conduct comprehensive unit, integration, and E2E testing.
    *   **Actionable:** Perform security review.
    *   **Actionable:** Final deployment to GitHub Pages and verification of all features.

## Course Hardware Requirements (Reference Only)

This section details the hardware considerations for students as outlined in the course description. The project architecture will assume these environments, but the book content MUST provide guidance for diverse setups.

*   **Digital Twin Workstation:** NVIDIA RTX 4070 Ti (12GB VRAM) or higher, Intel Core i7 (13th Gen+) or AMD Ryzen 9, 64 GB DDR5 RAM, Ubuntu 22.04 LTS.
*   **Physical AI Edge Kit:** NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB), Intel RealSense D435i/D455, USB Microphone/Speaker array.
*   **Robot Lab Options:** Unitree Go2 Edu (Proxy), Unitree G1 (Miniature/Premium).
*   **Cloud-Native Lab Option:** AWS g5.2xlarge or g6e.xlarge instance for Isaac Sim on Omniverse Cloud.

## Assessment Guidelines (Reference Only)

The project will be assessed based on the following:

*   ROS 2 package development project.
*   Gazebo simulation implementation.
*   Isaac-based perception pipeline.
*   Capstone: Simulated humanoid robot with conversational AI.
*   Successful implementation of all core and bonus project deliverables.

## Governance
This Constitution supersedes all other project practices and documentation. Amendments to this Constitution require a formal proposal, review, and explicit approval by the Project Architect, followed by documentation in an Architectural Decision Record (ADR) if architecturally significant. Versioning MUST follow semantic rules (MAJOR.MINOR.PATCH). All project artifacts and code reviews MUST ensure compliance with these stated principles.

**Version**: 1.0.8 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-12-09
