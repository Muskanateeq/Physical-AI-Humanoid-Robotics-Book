# Data Model: RAG Chatbot for Book Content Q&A

## Entities

### QueryRequest
- **Fields**:
  - `query`: string (required) - The user's question about the book content
  - `selected_text`: string (optional) - Specific text selected by the user (if any)
  - `context_length`: integer (optional, default: 5) - Number of surrounding paragraphs to include as context
  - `session_id`: string (optional) - ID to maintain conversation context

### QueryResponse
- **Fields**:
  - `response`: string (required) - The AI-generated answer
  - `sources`: array of SourceReference (optional) - List of book sections used to generate the response
  - `session_id`: string (required) - Session identifier for conversation context

### SourceReference
- **Fields**:
  - `content`: string (required) - The text content that was referenced
  - `page_number`: integer (optional) - Page number in the book
  - `section_title`: string (optional) - Title of the section
  - `similarity_score`: float (optional) - Relevance score from vector search (0.0 to 1.0)

### BookContent
- **Fields**:
  - `id`: string (required) - Unique identifier for the content chunk
  - `text`: string (required) - The actual text content
  - `page_number`: integer (optional) - Page number in the original book
  - `section_title`: string (optional) - Title of the section
  - `embedding`: array of float (required) - Vector embedding of the text content
  - `metadata`: object (optional) - Additional metadata about the content

### ConversationSession
- **Fields**:
  - `session_id`: string (required) - Unique identifier for the conversation
  - `created_at`: datetime (required) - Timestamp when the session was created
  - `updated_at`: datetime (required) - Timestamp when the session was last updated
  - `history`: array of Message (optional) - History of messages in the conversation

### Message
- **Fields**:
  - `role`: string (required) - "user" or "assistant"
  - `content`: string (required) - The message content
  - `timestamp`: datetime (required) - When the message was created
  - `sources`: array of SourceReference (optional) - Sources referenced in the response