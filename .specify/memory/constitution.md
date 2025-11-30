<!--
Sync Impact Report:
Version change: 0.0.0 → 1.0.0 (MAJOR: Initial creation of the constitution)
List of modified principles: (All new)
Added sections: Project Overview, Architectural Principles, Project Architecture, Development Workflow & Tools, Intelligence Specifications, Roles & Responsibilities, Weekly Development Plan, Course Hardware Requirements (Reference Only), Assessment Guidelines (Reference Only), Glossary / Definitions
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/sp.constitution.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Course Textbook Constitution

## Core Principles

### I. Spec-Driven Development (SDD) Mandate
All project phases—specification, planning, task management, and implementation—MUST strictly adhere to Spec-Kit Plus methodologies. Claude Code is the primary AI agent for driving and documenting these processes. All work MUST begin with a clear specification and an approved plan.

### II. Modularity and Scalability
All components (frontend, backend services, Docusaurus modules, chatbot agents) MUST be designed for modularity, independent testability, and scalability. Services MUST communicate via well-defined APIs. Each module SHOULD have a clear, singular purpose.

### III. Security-First Approach
Security MUST be a paramount consideration at every stage, from architecture design to deployment. This includes secure authentication (Better-Auth), data protection for user information (Neon Postgres), secure API endpoints (FastAPI), and vulnerability scanning. All secrets MUST be managed securely (e.g., environment variables).

### IV. Performance and Responsiveness
The book's frontend, RAG chatbot, and backend services MUST be designed for optimal performance and responsiveness. This includes efficient content loading, fast chatbot responses, and quick personalization/translation processing. Caching strategies MUST be implemented where appropriate to minimize latency and API costs.

### V. Maintainability and Extensibility
The codebase MUST be clean, well-documented, and easy to understand and extend. Docusaurus content, backend services, and AI agent configurations MUST follow established coding standards and best practices. Unnecessary complexity MUST be avoided.

### VI. User-Centric Design
All features, especially personalization and Urdu translation, MUST prioritize the end-user experience. Interfaces MUST be intuitive and accessible. Feedback mechanisms for personalized content and translation accuracy SHOULD be considered.

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
*   **Actionable:** Develop React components for embedding the RAG chatbot interface.
*   **Actionable:** Integrate Better-Auth UI components for signup/signin forms and user profile management.
*   **Actionable:** Implement "Personalize Content" and "Translate to Urdu" buttons within Docusaurus chapter layouts, visible to authenticated users.

### 3.2. Backend Architecture (RAG Chatbot, Authentication, & Intelligence Services)
#### 3.2.1. RAG Chatbot Stack
*   **API Layer:**
    *   **Actionable:** Initialize FastAPI service (`main.py`) with endpoints: `/chatbot/query` (for RAG), `/chatbot/history` (for conversation history).
*   **Orchestration:**
    *   **Actionable:** Integrate OpenAI Agents/ChatKit SDKs within FastAPI to manage chatbot states, intent recognition, and response generation.
*   **Vector Database:**
    *   **Actionable:** Configure Qdrant Cloud Free Tier instance; implement vector embedding generation (e.g., using OpenAI embeddings); create collection for book content vectors.
*   **Relational Database:**
    *   **Actionable:** Set up Neon Serverless Postgres instance; define schema for `user_conversations` (chat history) and `book_content_metadata` (Docusaurus page URLs, chunk IDs).
*   **Content Ingestion:**
    *   **Actionable:** Develop a script to parse Docusaurus markdown files, split into logical chunks, generate embeddings using an LLM, and upload vectors to Qdrant.

#### 3.2.2. Authentication & User Management (Better-Auth)
*   **Service:**
    *   **Actionable:** Integrate Better-Auth SDK/API into FastAPI for user registration, login, token management, and session validation.
*   **User Data Model:**
    *   **Actionable:** Extend user schema in Neon Serverless Postgres to include fields for `software_background` (e.g., `['Python', 'C++', 'ROS']`), `hardware_background` (e.g., `{'gpu_type': 'RTX 4070 Ti', 'os': 'Ubuntu 22.04'}`).
*   **API Endpoints:**
    *   **Actionable:** Secure FastAPI endpoints: `/auth/register`, `/auth/login`, `/auth/logout`, `/user/profile` (for retrieving/updating user background).

#### 3.2.3. Intelligence Services (Personalization & Translation)
*   **Actionable:** Initialize FastAPI service with dedicated endpoints: `/personalize` and `/translate`.
*   **Actionable:** Implement `POST /personalize` endpoint: Accepts `chapter_content` and `user_id`. Retrieves `user_background` from Neon Postgres. Calls an LLM (e.g., OpenAI API) with a prompt to rephrase `chapter_content` considering `user_background`. Returns personalized text.
*   **Actionable:** Implement `POST /translate` endpoint: Accepts `chapter_content` and `target_language` (e.g., 'ur' for Urdu). Calls a translation API (e.g., Google Translate API or dedicated LLM) to translate `chapter_content`. Returns translated text.

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
    *   **Actionable:** Integrate OpenAI Agents/ChatKit SDKs.
    *   **Actionable:** Develop content ingestion script: parse Docusaurus markdown, chunk, embed, upload to Qdrant.
    *   **Actionable:** Implement basic frontend UI for chatbot integration into Docusaurus.

*   **Weeks 6-7: Authentication & User Profile**
    *   **Focus:** Better-Auth integration, user background collection.
    *   **Actionable:** Integrate Better-Auth for signup/signin on the Docusaurus frontend.
    *   **Actionable:** Extend user schema in Neon Postgres for `software_background` and `hardware_background`.
    *   **Actionable:** Modify signup form to capture user background data.
    *   **Actionable:** Implement secure user profile API endpoint.

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

**Version**: 1.0.1 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29
