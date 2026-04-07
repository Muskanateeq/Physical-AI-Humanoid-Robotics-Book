# Data Model: Docusaurus Chatbot Integration

## Core RAG Entities

### User Query
- **Description**: The question or input provided by the website visitor that needs to be answered based on book content
- **Fields**:
  - text: string (the actual query text)
  - timestamp: datetime (when the query was made)
  - userId: string (optional, for logged-in users)
  - threadId: string (conversation identifier)
  - language: string (enum: 'English', 'Urdu', default: 'English')

### Book Content
- **Description**: The source material from the Physical AI and Humanoid Robotics book that serves as the knowledge base for the RAG system
- **Fields**:
  - contentId: string (unique identifier for the content chunk)
  - text: string (the actual book content)
  - sourceUrl: string (URL of the original Docusaurus page)
  - metadata: object (additional information like chapter, section, etc.)
  - embedding: array of numbers (vector representation for similarity search)

### Generated Response
- **Description**: The answer provided by the chatbot based on the book content and the user's query
- **Fields**:
  - responseId: string (unique identifier for the response)
  - queryId: string (reference to the original query)
  - text: string (the response text)
  - sources: array of strings (references to book content used)
  - timestamp: datetime (when the response was generated)
  - threadId: string (conversation identifier)
  - language: string (enum: 'English', 'Urdu', default: 'English')
  - relevance_score: number (0-1, how relevant the response is)

### Conversation History
- **Description**: The sequence of interactions between the user and the chatbot during a session
- **Fields**:
  - threadId: string (unique identifier for the conversation thread)
  - messages: array of message objects (each with role, content, timestamp)
  - createdAt: datetime (when the conversation started)
  - lastActive: datetime (when the last message was sent)
  - userId: string (optional, for logged-in users)
  - title: string (auto-generated from first message or user input)

## User Authentication Entities

### User (Better-Auth)
- **Description**: User account information managed by Better Auth
- **Fields**:
  - user_id: string (primary key, from Better-Auth)
  - name: string
  - email: string (unique)
  - password_hash: string (managed by Better-Auth)
  - created_at: datetime
  - updated_at: datetime
  - onboarding_completed: boolean (default: false)

### UserProfile (Neon PostgreSQL)
- **Description**: Extended user profile data stored in Neon Serverless PostgreSQL
- **Fields**:
  - user_id: string (foreign key to Better-Auth user)
  - software_background: string (enum: 'Beginner', 'Intermediate', 'Advanced')
  - hardware_os: string (enum: 'Windows', 'Mac', 'Linux', 'Chromebook/Web')
  - language_preference: string (enum: 'English', 'Urdu', default: 'English')
  - onboarding_completed: boolean (default: false)
  - created_at: datetime
  - updated_at: datetime

## Content Personalization Entities

### PersonalizedChapter
- **Description**: Personalized version of a chapter based on user's background
- **Fields**:
  - personalization_id: string (primary key, UUID)
  - user_id: string (foreign key to user)
  - chapter_id: string (identifier for the original chapter)
  - original_content_hash: string (to track if original changed)
  - personalized_content: text
  - personalization_settings: json (user's background used for personalization)
  - created_at: datetime
  - updated_at: datetime

### ChapterPersonalizationLog
- **Description**: Log of personalization activities for analytics
- **Fields**:
  - log_id: string (primary key, UUID)
  - user_id: string (foreign key to user)
  - chapter_id: string
  - personalization_id: string (foreign key to PersonalizedChapter)
  - action: string (enum: 'view', 'reset', 'customize')
  - device_info: json (browser, OS, screen size)
  - timestamp: datetime

## Translation Entities

### TranslatedChapter
- **Description**: Translated version of a chapter (e.g., from English to Urdu)
- **Fields**:
  - translation_id: string (primary key, UUID)
  - user_id: string (foreign key to user, nullable for system translations)
  - chapter_id: string
  - source_language: string (default: 'English')
  - target_language: string (enum: 'Urdu')
  - original_content_hash: string
  - translated_content: text
  - translation_quality_score: number (nullable)
  - created_at: datetime
  - updated_at: datetime

### TranslationLog
- **Description**: Log of translation activities for analytics
- **Fields**:
  - log_id: string (primary key, UUID)
  - user_id: string (foreign key to user)
  - translation_id: string (foreign key to TranslatedChapter)
  - source_language: string
  - target_language: string
  - action: string (enum: 'translate', 'toggle', 'view')
  - timestamp: datetime

## RAG System Entities

### RAGQuery
- **Description**: Record of a RAG query made by a user
- **Fields**:
  - query_id: string (primary key, UUID)
  - user_id: string (foreign key to user)
  - thread_id: string (foreign key to conversation thread)
  - query_text: text
  - query_language: string (enum: 'English', 'Urdu')
  - selected_text: text (nullable, for Selected-Text RAG Mode)
  - retrieved_chunks_count: integer
  - retrieval_time_ms: integer
  - timestamp: datetime

### RAGResponse
- **Description**: Record of a RAG response generated by the system
- **Fields**:
  - response_id: string (primary key, UUID)
  - query_id: string (foreign key to RAGQuery)
  - response_text: text
  - response_language: string (enum: 'English', 'Urdu')
  - source_citations: json (array of source references with page/chapter info)
  - llm_model_used: string (default: 'gemini-1.5-flash')
  - response_time_ms: integer
  - relevance_score: number (0-1)
  - timestamp: datetime

## Feedback Entities

### UserFeedback
- **Description**: User feedback on responses (like/dislike/report)
- **Fields**:
  - feedback_id: string (primary key, UUID)
  - user_id: string (foreign key to user, nullable for anonymous)
  - response_id: string (foreign key to RAGResponse)
  - feedback_type: string (enum: 'like', 'dislike', 'report_inaccurate')
  - feedback_comment: text (nullable)
  - timestamp: datetime

### FeedbackAggregation
- **Description**: Aggregated feedback for content quality assessment
- **Fields**:
  - aggregation_id: string (primary key, UUID)
  - content_hash: string (hash of the content being rated)
  - like_count: integer (default: 0)
  - dislike_count: integer (default: 0)
  - report_count: integer (default: 0)
  - average_relevance_score: number
  - last_updated: datetime

## Session and Analytics Entities

### UserSession
- **Description**: User session information for tracking and security
- **Fields**:
  - session_id: string (primary key, UUID)
  - user_id: string (foreign key to user)
  - session_token_hash: string
  - ip_address: string
  - user_agent: text
  - started_at: datetime
  - ended_at: datetime (nullable)
  - is_active: boolean (default: true)

### UserInteractionLog
- **Description**: Log of user interactions for analytics and improvement
- **Fields**:
  - log_id: string (primary key, UUID)
  - user_id: string (foreign key to user, nullable for non-authenticated)
  - session_id: string (foreign key to UserSession)
  - interaction_type: string (enum: 'chat_message', 'chapter_personalize', 'chapter_translate', 'auth_action', 'ui_interaction')
  - interaction_details: json (depends on interaction_type)
  - page_url: string
  - timestamp: datetime

## Relationships
- User Query belongs to Conversation History (one-to-many)
- Generated Response belongs to Conversation History (one-to-many)
- Book Content is referenced by Generated Response (many-to-many through sources)
- UserProfile belongs to User (one-to-one)
- Conversation History belongs to User (one-to-many)
- PersonalizedChapter belongs to User (one-to-many)
- TranslatedChapter belongs to User (one-to-many)
- RAGQuery belongs to User and Conversation History (many-to-one each)
- RAGResponse belongs to RAGQuery (one-to-one)
- UserFeedback belongs to User and RAGResponse (many-to-one each)
- UserSession belongs to User (one-to-many)
- UserInteractionLog belongs to User and UserSession (many-to-one each)

## Validation Rules
- User Query text must not be empty
- Book Content must have valid source URL
- Generated Response must reference at least one Book Content source
- Conversation History must have a valid threadId for persistence
- Email in User must be properly formatted
- Password in User must meet strength requirements
- Onboarding fields (software_background, hardware_os) required for full access
- Chapter IDs must follow the format: module-X/chapter-Y
- Content hashes used for tracking changes
- Personalized/translated content must not exceed 2x the original length
- Query text must be between 5 and 500 characters
- Selected text (when provided) must be between 10 and 5000 characters
- Response relevance scores must be between 0 and 1

## State Transitions

### User Account States
- UNREGISTERED → PARTIAL_USER (after Step 1 signup)
- PARTIAL_USER → FULL_USER (after Step 2 onboarding completion)
- FULL_USER → SUSPENDED (if account issues detected)

### Conversation States
- CREATED → ACTIVE (when first message sent)
- ACTIVE → INACTIVE (after 24 hours of inactivity)
- INACTIVE → ARCHIVED (after 30 days of inactivity)

### Personalization States
- ORIGINAL → PERSONALIZED (when user requests personalization)
- PERSONALIZED → ORIGINAL (when user resets personalization)
- PERSONALIZED → UPDATED (when original content changes)