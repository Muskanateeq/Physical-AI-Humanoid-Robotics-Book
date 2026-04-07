# API Contracts: Docusaurus Chatbot Integration

## Authentication API

### POST /auth/register
**Description**: Register a new user (Step 1 of two-step process)

**Request**:
```json
{
  "name": "string",
  "email": "string (valid email)",
  "password": "string (min 8 chars)",
  "password_confirmation": "string (same as password)"
}
```

**Response** (201 Created):
```json
{
  "success": true,
  "message": "Registration initiated, please complete onboarding",
  "user_id": "string",
  "step": "step_1_complete"
}
```

### POST /auth/onboarding
**Description**: Complete user onboarding (Step 2 of two-step process)

**Request**:
```json
{
  "user_id": "string",
  "software_background": "enum: ['Beginner', 'Intermediate', 'Advanced']",
  "hardware_os": "enum: ['Windows', 'Mac', 'Linux', 'Chromebook/Web']"
}
```

**Response** (200 OK):
```json
{
  "success": true,
  "message": "Onboarding completed successfully",
  "user_id": "string",
  "onboarding_completed": true
}
```

### POST /auth/login
**Description**: Authenticate user

**Request**:
```json
{
  "email": "string",
  "password": "string"
}
```

**Response** (200 OK):
```json
{
  "success": true,
  "token": "string (JWT or BetterAuth token)",
  "user": {
    "id": "string",
    "name": "string",
    "email": "string",
    "onboarding_completed": "boolean"
  }
}
```

### GET /auth/me
**Description**: Get current user profile

**Headers**:
- Authorization: Bearer {token}

**Response** (200 OK):
```json
{
  "user": {
    "id": "string",
    "name": "string",
    "email": "string",
    "onboarding_completed": "boolean",
    "profile": {
      "software_background": "string",
      "hardware_os": "string",
      "language_preference": "enum: ['en', 'ur']",
      "personalization_settings": "object"
    }
  }
}
```

## RAG Chatbot API

### POST /api/rag-chat
**Description**: Main RAG chat endpoint with dual-mode support

**Headers**:
- Authorization: Bearer {token} (optional for guest mode)

**Request**:
```json
{
  "message": "string (user query)",
  "session_id": "string (optional)",
  "mode": "enum: ['selected-text', 'standard-rag'] (default: 'standard-rag')",
  "selected_text": "string (required if mode is 'selected-text')",
  "language_preference": "enum: ['en', 'ur'] (optional)"
}
```

**Response** (200 OK):
```json
{
  "response": "string (chatbot response)",
  "sources": [
    {
      "chunk_id": "string",
      "document_url": "string",
      "content_preview": "string",
      "score": "number"
    }
  ],
  "session_id": "string",
  "language": "enum: ['en', 'ur']",
  "mode_used": "enum: ['selected-text', 'standard-rag']"
}
```

### GET /api/history
**Description**: Get conversation history

**Headers**:
- Authorization: Bearer {token}

**Query Parameters**:
- `session_id`: string (optional, if omitted returns all user sessions)
- `limit`: number (default: 50)
- `offset`: number (default: 0)

**Response** (200 OK):
```json
{
  "history": [
    {
      "id": "string",
      "session_id": "string",
      "messages": [
        {
          "role": "enum: ['user', 'assistant']",
          "content": "string",
          "timestamp": "datetime",
          "sources": "array (optional)"
        }
      ],
      "mode": "enum: ['selected-text', 'standard-rag']",
      "created_at": "datetime"
    }
  ]
}
```

### POST /api/feedback
**Description**: Submit user feedback on chat responses

**Headers**:
- Authorization: Bearer {token} (optional)

**Request**:
```json
{
  "conversation_id": "string",
  "message_id": "string",
  "rating": "enum: ['positive', 'negative']",
  "comment": "string (optional)"
}
```

**Response** (200 OK):
```json
{
  "success": true,
  "message": "Feedback received"
}
```

### GET /api/health
**Description**: Health check endpoint

**Response** (200 OK):
```json
{
  "status": "healthy",
  "timestamp": "datetime",
  "dependencies": {
    "qdrant": "boolean",
    "gemini": "boolean",
    "database": "boolean"
  }
}
```

## Personalization API

### POST /api/personalize
**Description**: Personalize chapter content based on user profile

**Headers**:
- Authorization: Bearer {token}

**Request**:
```json
{
  "chapter_id": "string",
  "chapter_content": "string (raw markdown content)",
  "user_id": "string (optional if token provided)"
}
```

**Response** (200 OK):
```json
{
  "personalized_content": "string (modified markdown content)",
  "modifications": [
    {
      "type": "enum: ['explanation', 'example', 'instruction']",
      "original": "string",
      "modified": "string"
    }
  ],
  "variant_id": "string"
}
```

### GET /api/personalization-history
**Description**: Get user's personalization history

**Headers**:
- Authorization: Bearer {token}

**Response** (200 OK):
```json
{
  "history": [
    {
      "id": "string",
      "chapter_id": "string",
      "content_variant": "string",
      "user_background": "object",
      "device_platform": "string",
      "timestamp": "datetime"
    }
  ]
}
```

## Translation API

### POST /api/translate
**Description**: Translate chapter content to target language

**Headers**:
- Authorization: Bearer {token}

**Request**:
```json
{
  "chapter_content": "string (raw markdown content)",
  "target_language": "enum: ['ur'] (default: 'ur')",
  "preserve_formatting": "boolean (default: true)"
}
```

**Response** (200 OK):
```json
{
  "translated_content": "string (translated markdown content)",
  "source_language": "enum: ['en']",
  "target_language": "enum: ['ur']",
  "preserved_elements": [
    {
      "type": "enum: ['code', 'heading', 'list']",
      "original": "string",
      "translated": "string"
    }
  ]
}
```

### POST /api/chatbot-translate
**Description**: Translate chatbot responses to user's preferred language

**Headers**:
- Authorization: Bearer {token} (optional)

**Request**:
```json
{
  "text": "string (text to translate)",
  "source_language": "enum: ['en'] (optional)",
  "target_language": "enum: ['ur'] (default: 'ur')"
}
```

**Response** (200 OK):
```json
{
  "translated_text": "string",
  "source_language": "enum: ['en']",
  "target_language": "enum: ['ur']"
}
```

## Search API

### POST /api/search
**Description**: Search through book content

**Request**:
```json
{
  "query": "string (search query)",
  "limit": "number (default: 10)",
  "language": "enum: ['en', 'ur'] (default: 'en')"
}
```

**Response** (200 OK):
```json
{
  "results": [
    {
      "id": "string",
      "title": "string",
      "content_preview": "string",
      "document_url": "string",
      "score": "number",
      "highlighted_snippets": ["string"]
    }
  ],
  "total_results": "number",
  "query_time_ms": "number"
}
```

## Error Response Format

All error responses follow this format:

**Response** (4xx/5xx):
```json
{
  "error": {
    "code": "string (error code)",
    "message": "string (human-readable error message)",
    "details": "object (optional, additional error details)"
  }
}
```

## Common Error Codes

- `AUTH_REQUIRED`: Authentication is required for this endpoint
- `ONBOARDING_REQUIRED`: User must complete onboarding before accessing features
- `INVALID_INPUT`: Request data validation failed
- `RESOURCE_NOT_FOUND`: Requested resource does not exist
- `SERVICE_UNAVAILABLE`: Upstream service (Qdrant, Gemini) is unavailable
- `RATE_LIMIT_EXCEEDED`: Too many requests from this client
- `INTERNAL_ERROR`: Unexpected server error occurred

## Security Requirements

- All endpoints except health check and public search require authentication
- Use HTTPS in production
- Implement rate limiting per IP/session
- Sanitize all user inputs
- Validate file uploads and content types
- Use prepared statements to prevent SQL injection