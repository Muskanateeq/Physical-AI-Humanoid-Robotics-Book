# Quickstart Guide: Docusaurus Integrated RAG Chatbot

## Prerequisites

- Python 3.11+
- Node.js 18+
- Access to Qdrant Cloud instance with book content already embedded
- Neon Serverless Postgres database
- Google Gemini API key

## Setup Instructions

### 1. Clone and Initialize Repository

```bash
git clone <your-repo-url>
cd <repo-name>
```

### 2. Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Frontend Setup

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install
```

### 4. Environment Configuration

Create `.env` file in the backend directory:

```env
# Gemini API
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_content

# Neon Postgres Configuration
DATABASE_URL=postgresql://username:password@ep-xxxxxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# Optional: Set other configuration
DEBUG=true
PORT=8000
```

### 5. Run the Application

#### Development Mode

**Backend:**
```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn src.main:app --reload --port 8000
```

**Frontend:**
```bash
cd frontend
npm run dev
```

#### Production Mode
```bash
# Using Docker (recommended for production)
docker-compose up --build
```

## API Endpoints

### Chat Endpoints
- `POST /chat` - Normal mode RAG chat
- `POST /select-and-chat` - Selected text mode chat
- `GET /history` - Get chat history
- `GET /history/list` - Get session list
- `DELETE /history` - Clear chat history

### Feedback Endpoints
- `POST /feedback` - Submit feedback on message
- `GET /feedback/stats` - Get feedback statistics

### Voice Endpoints
- `POST /voice/generate` - Generate voice from text
- `GET /voice/preferences` - Get voice preferences
- `POST /voice/preferences` - Update voice preferences

## Usage Examples

### Making a Chat Request (Normal Mode)
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ROS 2?",
    "sessionId": "session-123",
    "mode": "normal"
  }'
```

### Making a Chat Request (Selected Text Mode)
```bash
curl -X POST http://localhost:8000/select-and-chat \
  -H "Content-Type: application/json" \
  -d '{
    "selectedText": "ROS 2 is a set of libraries and tools...",
    "question": "Explain the concepts mentioned here?",
    "sessionId": "session-123",
    "mode": "selected-text"
  }'
```

## Docusaurus Integration

To integrate the chatbot with your Docusaurus site:

1. Add the ChatKit frontend components to your Docusaurus pages
2. Configure the API endpoints to point to your backend
3. Ensure the floating chat icon is visible on all book pages

Example integration in Docusaurus:
```jsx
// In your Docusaurus layout
import { ChatKitComponent } from './path-to-chatkit';

function Layout({children}) {
  return (
    <>
      <main>{children}</main>
      <ChatKitComponent
        apiUrl="http://localhost:8000"
        showFloatingIcon={true}
      />
    </>
  );
}
```

## Troubleshooting

### Common Issues

**Q: Chatbot not responding**
- A: Check that Qdrant and Gemini API keys are correctly configured
- A: Verify that book content is properly embedded in Qdrant

**Q: Selected text mode not working**
- A: Ensure the text selection capture is properly implemented in frontend
- A: Check that the selected text is being sent with the request

**Q: Voice mode not working**
- A: Verify TTS service is properly configured
- A: Check that audio playback is enabled in browser

### Development Tips

- Use `DEBUG=true` to enable detailed logging
- Check the browser console for frontend errors
- Monitor the backend logs for API issues
- Use the API documentation at `/docs` to test endpoints

## Next Steps

1. Customize the chatbot UI to match your Docusaurus theme
2. Fine-tune the system prompt for better book-specific responses
3. Implement additional features like conversation history persistence
4. Add analytics to track usage and feedback