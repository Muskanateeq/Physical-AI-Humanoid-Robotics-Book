# Quickstart Guide: RAG Chatbot UI for Docusaurus Book

## Prerequisites

- Node.js 18+ (for frontend Docusaurus development)
- Python 3.11+ (for backend FastAPI development)
- Qdrant vector database (running instance)
- Google Gemini API key
- FastAPI dependencies

## Frontend Setup (Docusaurus)

1. **Navigate to frontend directory**:
   ```bash
   cd frontend
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Add the chatbot component to your Docusaurus pages**:
   - Import the ChatbotContainer component in your desired pages
   - Ensure the slate blue theme is properly applied

4. **Run the Docusaurus development server**:
   ```bash
   npm run start
   ```

## Backend Setup

1. **Install backend dependencies**:
   ```bash
   pip install fastapi uvicorn python-dotenv google-generativeai openai pydantic
   ```

2. **Environment Configuration**:
   Create a `.env` file with the following variables:
   ```env
   GEMINI_API_KEY=your_gemini_api_key_here
   QDRANT_URL=your_qdrant_instance_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=physical_ai_humanoid_robotics
   DATABASE_URL=postgresql://user:password@localhost/dbname
   ```

3. **Start the backend service**:
   ```bash
   uvicorn backend.src.main:app --reload --port 8000
   ```

## API Endpoints

### 1. Query Endpoint
- **URL**: `POST /chat/query`
- **Description**: Process user questions about book content
- **Features**: Supports general questions and selected text queries

### 2. History Endpoints
- **URLs**: `POST /chat/history`, `POST /chat/history/save`
- **Description**: Manage conversation history with save/load functionality

### 3. Feedback Endpoint
- **URL**: `POST /chat/feedback`
- **Description**: Submit like/dislike feedback for responses

### 4. Health Check
- **URL**: `GET /health`
- **Description**: Check service health status

## UI Features

### Chat Interface
- Slate blue theme matching Docusaurus design
- Animated loading indicators (three dots) during Gemini processing
- Like/dislike buttons for each response
- Copy functionality for messages
- Chat history management with save capability

### Integration with Docusaurus
- Seamless integration with existing Docusaurus layout
- Responsive design for all device sizes
- Consistent styling with the rest of the book

## Development

1. **Run backend tests**:
   ```bash
   pytest backend/tests/
   ```

2. **Run frontend tests**:
   ```bash
   npm test
   ```

3. **Build for production**:
   ```bash
   # Frontend
   npm run build

   # Backend documentation
   uvicorn backend.src.main:app --reload --port 8000 --root-path /api
   ```

## Customization

### Theme Colors
- The chatbot UI uses the slate blue theme from your Docusaurus configuration
- Colors can be customized in the ThemedComponents.tsx file
- Ensure all UI elements maintain consistency with the existing website design