# Data Model: RAG Chatbot UI for Docusaurus Book

## Entities

### ChatMessage
- **Fields**:
  - `id`: string (required) - Unique identifier for the message
  - `content`: string (required) - The text content of the message
  - `role`: string (required) - "user" or "assistant"
  - `timestamp`: datetime (required) - When the message was created
  - `likeStatus`: string (optional) - "liked", "disliked", or null
  - `sourceReferences`: array of SourceReference (optional) - References to book content used in the response

### ChatSession
- **Fields**:
  - `sessionId`: string (required) - Unique identifier for the conversation session
  - `userId`: string (optional) - Identifier for authenticated user (null for anonymous sessions)
  - `created_at`: datetime (required) - Timestamp when the session was created
  - `updated_at`: datetime (required) - Timestamp when the session was last updated
  - `messages`: array of ChatMessage (required) - History of messages in the conversation
  - `isActive`: boolean (required) - Whether the session is currently active

### SourceReference
- **Fields**:
  - `content`: string (required) - The text content that was referenced
  - `page_number`: integer (optional) - Page number in the book
  - `section_title`: string (optional) - Title of the section
  - `similarity_score`: float (optional) - Relevance score from vector search (0.0 to 1.0)
  - `source_url`: string (optional) - URL to the relevant section in the Docusaurus book

### UserFeedback
- **Fields**:
  - `messageId`: string (required) - ID of the message being rated
  - `userId`: string (optional) - ID of the user providing feedback (null for anonymous)
  - `feedbackType`: string (required) - "like" or "dislike"
  - `timestamp`: datetime (required) - When the feedback was provided

### ChatHistory
- **Fields**:
  - `historyId`: string (required) - Unique identifier for the saved history
  - `sessionId`: string (required) - Session that was saved
  - `userId`: string (optional) - User who saved the history (null for anonymous)
  - `title`: string (required) - Title for the saved conversation
  - `saved_at`: datetime (required) - When the history was saved
  - `messages`: array of ChatMessage (required) - The saved conversation