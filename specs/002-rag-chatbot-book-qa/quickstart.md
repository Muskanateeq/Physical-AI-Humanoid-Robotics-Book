# Quickstart Guide: RAG Chatbot for Book Content Q&A

## Prerequisites

- Python 3.11+
- Qdrant vector database (running instance)
- Google Gemini API key
- FastAPI dependencies

## Setup

1. **Install dependencies**:
   ```bash
   pip install fastapi uvicorn python-dotenv google-generativeai qdrant-client fastembed PyYAML
   ```

2. **Environment Configuration**:
   Create a `.env` file with the following variables:
   ```env
   GEMINI_API_KEY=your_gemini_api_key_here
   QDRANT_URL=your_qdrant_instance_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=book_content
   ```

3. **Start the service**:
   ```bash
   uvicorn backend.src.main:app --reload --port 8000
   ```

## API Endpoints

### 1. Query Endpoint
- **URL**: `POST /chat/query`
- **Request Body**:
  ```json
  {
    "query": "Your question about the book",
    "selected_text": "Optional selected text for context",
    "context_length": 5
  }
  ```
- **Response**:
  ```json
  {
    "response": "AI-generated answer",
    "sources": [
      {
        "content": "Referenced book content",
        "page_number": 42,
        "section_title": "Section Name",
        "similarity_score": 0.85
      }
    ],
    "session_id": "unique-session-id"
  }
  ```

### 2. Health Check
- **URL**: `GET /health`
- **Response**: `{"status": "healthy"}`

## Usage Example

```python
import requests

# Query the RAG chatbot
response = requests.post("http://localhost:8000/chat/query", json={
    "query": "What are the key concepts in chapter 3?",
    "selected_text": "Optional text selected by user"
})

data = response.json()
print(f"Answer: {data['response']}")
print(f"Sources: {data['sources']}")
```

## Development

1. **Run tests**:
   ```bash
   pytest backend/tests/
   ```

2. **Add book content to Qdrant**:
   - Use the content ingestion script to convert book content to embeddings
   - The script processes book content, generates embeddings using FastEmbed
   - Stores the embeddings in Qdrant with metadata