---
id: 1
title: Create RAG Chatbot Spec
stage: spec
date: 2025-12-06
surface: agent
model: claude-opus-4-5-20251101
feature: 004-docusaurus-rag-chatbot
branch: 004-docusaurus-rag-chatbot
user:
command: /sp.specify You are a senior AI architect and full-stack engineer. I am building an "Integrated RAG Chatbot" inside my **Docusaurus book website**.
labels: [specification, rag, chatbot, docusaurus, qdrant, gemini]
links:
  spec: specs/004-docusaurus-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-docusaurus-rag-chatbot/spec.md
tests:
 - none
---

## Prompt

You are a senior AI architect and full-stack engineer.

I am building an "Integrated RAG Chatbot" inside my **Docusaurus book website**.

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
7. "OpenAI-style" Agent System Prompt rewritten for Gemini
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
Start with architecture, then give full code section by section.

## Response snapshot

Created comprehensive specification for Docusaurus Integrated RAG Chatbot feature with:
- 4 prioritized user stories covering core functionality
- Detailed functional requirements (20 specific requirements)
- Key entities definition
- 10 measurable success criteria
- Edge cases consideration

## Outcome

- ✅ Impact: Complete spec ready for planning phase
- 🧪 Tests: none
- 📁 Files: Created specs/004-docusaurus-rag-chatbot/spec.md
- 🔁 Next prompts: /sp.plan to create implementation plan
- 🧠 Reflection: Spec captures all requirements from user prompt with clear acceptance criteria

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL):
- Prompt variant (if applicable):
- Next experiment (smallest change to try):