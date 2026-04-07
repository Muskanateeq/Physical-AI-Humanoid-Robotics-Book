# Quickstart Guide: Physical AI & Humanoid Robotics Book Semantic Search System

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Access to Qdrant vector database
- Gemini API key (optional, for additional processing)
- The "Physical AI & Humanoid Robotics" book content in frontend/docs directory with 5 modules and 19 total chapters:
  - 00-neurobotics-overview (2 chapters: course-overview.md, intro-physical-ai.md)
  - module-1-ros2 (3 chapters: chapter1.md, chapter2.md, chapter3.md)
  - module-2-simulation (7 chapters: chapter4.md through chapter92.md)
  - module-3-aibrain (6 chapters: chapter11.md through chapter16.md)
  - module-4-vla (3 chapters: 5-vla-introduction-and-voice.md, 6-llm-cognitive-planning-and-vision.md, 7-ros2-execution-and-capstone.md)

## Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Ensure book content is present
Verify that the "Physical AI & Humanoid Robotics" book content exists in frontend/docs with all 19 chapters across 5 modules:
```bash
ls -la frontend/docs/
# Should show: 00-neurobotics-overview, module-1-ros2, module-2-simulation, module-3-aibrain, module-4-vla
```

### 3. Navigate to backend directory
```bash
cd backend
```

### 4. Create virtual environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 5. Install dependencies
```bash
pip install -r requirements.txt
```

### 6. Configure environment variables
Create a `.env` file in the backend directory:
```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
GEMINI_API_KEY=your_gemini_api_key  # Optional
QDRANT_COLLECTION_NAME=physical_ai_humanoid_robotics
EMBEDDING_MODEL=all-MiniLM-L6-v2
```

## Running the System

### 1. Index the complete book content (all 19 chapters)
```bash
python -m src.cli.indexer_cli --index-all
```

### 2. Index specific book modules
```bash
# Index a specific module (all its chapters)
python -m src.cli.indexer_cli --module module-1-ros2

# Index multiple modules
python -m src.cli.indexer_cli --module module-1-ros2 --module module-2-simulation
```

### 3. Index specific chapters/files
```bash
python -m src.cli.indexer_cli --index-file path/to/file.md
```

### 4. Update existing index
```bash
python -m src.cli.indexer_cli --update-index
```

## API Endpoints (when available)

### Health Check
```
GET /health
```

### Semantic Search
```
POST /search
Content-Type: application/json

{
  "query": "your search query",
  "limit": 10
}
```

## Development

### Running tests
```bash
pytest tests/
```

### Running specific test suites
```bash
# Unit tests
pytest tests/unit/

# Integration tests
pytest tests/integration/
```

## Configuration

### Environment Variables
- `QDRANT_URL`: URL to your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant authentication
- `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings (default: physical_ai_humanoid_robotics)
- `EMBEDDING_MODEL`: Model to use for embeddings (default: all-MiniLM-L6-v2)
- `GEMINI_API_KEY`: Google Gemini API key (optional)
- `FRONTEND_DOCS_PATH`: Path to the docs directory (default: ../frontend/docs)

### Processing Options
- `BATCH_SIZE`: Number of documents to process in each batch (default: 10)
- `CHUNK_SIZE`: Maximum characters per content chunk (default: 1000)
- `CHUNK_OVERLAP`: Overlap between content chunks (default: 100)

## Book Structure
The system will create embeddings for each of the 19 individual chapters across the 5 modules of the "Physical AI & Humanoid Robotics" book:
- Module 0: 2 chapters (overview content)
- Module 1: 3 chapters (ROS2 content)
- Module 2: 7 chapters (simulation content)
- Module 3: 6 chapters (AI brain content)
- Module 4: 3 chapters (VLA content)