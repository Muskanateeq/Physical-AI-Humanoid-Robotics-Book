# Data Model: Docusaurus Integrated RAG Chatbot

## Entity Relationships

```
[ChatSession] 1 -- * [ChatMessage]
[ChatMessage] 1 -- * [UserFeedback]
[ChatSession] 1 -- 1 [VoicePreference]
```

## Entity Definitions

### ChatSession
Represents a user's conversation with the chatbot

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for the session |
| user_id | VARCHAR(255) | NULLABLE | Identifier for authenticated user |
| created_at | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | When the session was created |
| updated_at | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | When the session was last updated |
| title | VARCHAR(255) | NULLABLE | Auto-generated title for the session |
| metadata | JSONB | NULLABLE | Additional session metadata |

### ChatMessage
Represents individual messages in a conversation

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for the message |
| session_id | UUID | FOREIGN KEY (chat_sessions.id) ON DELETE CASCADE | Reference to parent session |
| role | VARCHAR(10) | CHECK (role IN ('user', 'assistant')) | Sender of the message |
| content | TEXT | NOT NULL | The actual message content |
| timestamp | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | When the message was created |
| metadata | JSONB | NULLABLE | Additional message metadata |
| source_type | VARCHAR(20) | CHECK (source_type IN ('normal', 'selected-text')), DEFAULT 'normal' | How the response was generated |
| selected_text | TEXT | NULLABLE | Text that was selected by user (for selected-text mode) |

### UserFeedback
Represents user feedback on specific messages

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for the feedback |
| message_id | UUID | FOREIGN KEY (chat_messages.id) ON DELETE CASCADE | Reference to the message being rated |
| user_id | VARCHAR(255) | NULLABLE | Identifier for the user providing feedback |
| feedback_type | VARCHAR(10) | CHECK (feedback_type IN ('like', 'dislike')) | Type of feedback |
| comment | TEXT | NULLABLE | Optional comment with feedback |
| timestamp | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | When feedback was submitted |
| UNIQUE | (message_id, user_id) | Constraint | Each user can provide feedback once per message |

### VoicePreference
Represents user's preference for voice mode

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for the preference |
| session_id | UUID | FOREIGN KEY (chat_sessions.id) ON DELETE CASCADE | Reference to the session |
| user_id | VARCHAR(255) | NULLABLE | Identifier for the user |
| enabled | BOOLEAN | DEFAULT false | Whether voice mode is enabled |
| voice_type | VARCHAR(20) | DEFAULT 'robotic' | Type of voice to use |
| updated_at | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | When preference was last updated |

## Validation Rules

### ChatSession Validation
- `user_id` must be a valid string if provided
- `title` must be less than 256 characters if provided
- `created_at` must not be in the future
- `updated_at` must not be in the future

### ChatMessage Validation
- `session_id` must reference an existing session
- `role` must be either 'user' or 'assistant'
- `content` must not be empty
- `source_type` must be either 'normal' or 'selected-text'
- `timestamp` must not be in the future

### UserFeedback Validation
- `message_id` must reference an existing message
- `feedback_type` must be either 'like' or 'dislike'
- `timestamp` must not be in the future
- Each user can only provide feedback once per message

### VoicePreference Validation
- `session_id` must reference an existing session
- `voice_type` must be one of allowed values
- `updated_at` must not be in the future

## State Transitions

### ChatSession States
- New: Session created, no messages yet
- Active: Session has messages, still ongoing
- Closed: Session completed by user or timeout

### ChatMessage States
- Pending: Message sent, waiting for response
- Processed: Response received and processed
- Error: Error occurred during processing

### UserFeedback States
- Submitted: Feedback recorded in database
- Processed: Feedback included in statistics

## Indexes

### Performance Indexes
- `chat_sessions.user_id` - for user-specific queries
- `chat_messages.session_id` - for session-based queries
- `chat_messages.timestamp` - for chronological ordering
- `user_feedback.message_id` - for feedback lookup
- `user_feedback.timestamp` - for feedback statistics

### Unique Constraints
- `user_feedback(message_id, user_id)` - prevents duplicate feedback