"""
Generate embeddings for Physical AI & Humanoid Robotics book
Reads markdown files from docs/ and uploads to Qdrant
"""

import os
import re
from pathlib import Path
from typing import List, Dict
from dotenv import load_dotenv
from fastembed import TextEmbedding
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

# Load environment variables
load_dotenv()

# Configuration
DOCS_DIR = Path(__file__).parent / "physical-ai-humanoid-robotics" / "docs"
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_robotics")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
EMBEDDING_MODEL = "BAAI/bge-small-en-v1.5"
VECTOR_SIZE = 384  # bge-small-en-v1.5 produces 384-dimensional vectors

def extract_metadata(file_path: Path) -> Dict[str, str]:
    """Extract metadata from file path"""
    parts = file_path.parts

    # Find module name
    module = "unknown"
    for part in parts:
        if part.startswith("module-"):
            module = part
            break
        elif part.startswith("00-"):
            module = part
            break

    return {
        "file_path": str(file_path.relative_to(DOCS_DIR.parent)),
        "module": module,
        "chapter": file_path.stem,
        "filename": file_path.name
    }

def read_markdown_file(file_path: Path) -> str:
    """Read and clean markdown content"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Remove frontmatter
        content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        # Remove code blocks (keep content readable)
        content = re.sub(r'```[\s\S]*?```', '[CODE_BLOCK]', content)

        # Clean up extra whitespace
        content = re.sub(r'\n{3,}', '\n\n', content)

        return content.strip()
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return ""

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """Split text into overlapping chunks"""
    if len(text) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # Try to break at sentence boundary
        if end < len(text):
            # Look for sentence end
            sentence_end = text.rfind('.', start, end)
            if sentence_end > start + chunk_size // 2:
                end = sentence_end + 1

        chunks.append(text[start:end].strip())
        start = end - overlap

    return chunks

def main():
    print("=" * 70)
    print("Physical AI & Humanoid Robotics Book - Embedding Generator")
    print("=" * 70)
    print()

    # Initialize embedding model
    print("Loading embedding model...")
    embedding_model = TextEmbedding(model_name=EMBEDDING_MODEL)
    print(f"[OK] Model loaded: {EMBEDDING_MODEL}")
    print()

    # Initialize Qdrant client
    print("Connecting to Qdrant...")
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=30
    )
    print(f"[OK] Connected to Qdrant")
    print()

    # Create collection if it doesn't exist
    print(f"Setting up collection: {COLLECTION_NAME}")
    collections = qdrant_client.get_collections().collections
    collection_names = [c.name for c in collections]

    if COLLECTION_NAME in collection_names:
        print(f"[WARN] Collection already exists, deleting...")
        qdrant_client.delete_collection(COLLECTION_NAME)

    qdrant_client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE)
    )
    print(f"[OK] Collection created")
    print()

    # Find all markdown files
    print("Scanning for markdown files...")
    md_files = list(DOCS_DIR.rglob("*.md")) + list(DOCS_DIR.rglob("*.mdx"))
    print(f"[OK] Found {len(md_files)} markdown files")
    print()

    # Process files
    print("Processing files and generating embeddings...")
    print("-" * 70)

    all_points = []
    point_id = 0

    for idx, file_path in enumerate(md_files, 1):
        print(f"[{idx}/{len(md_files)}] Processing: {file_path.name}")

        # Read content
        content = read_markdown_file(file_path)
        if not content:
            print(f"  [SKIP] Empty or error")
            continue

        # Extract metadata
        metadata = extract_metadata(file_path)

        # Chunk content
        chunks = chunk_text(content, chunk_size=800, overlap=150)
        print(f"  -> Split into {len(chunks)} chunks")

        # Generate embeddings for chunks
        for chunk_idx, chunk in enumerate(chunks):
            if len(chunk.strip()) < 50:  # Skip very short chunks
                continue

            # Generate embedding
            embedding = list(embedding_model.embed([chunk]))[0]

            # Create point
            point = PointStruct(
                id=point_id,
                vector=embedding.tolist(),
                payload={
                    "content_chunk": chunk,
                    "chunk_index": chunk_idx,
                    "total_chunks": len(chunks),
                    **metadata
                }
            )
            all_points.append(point)
            point_id += 1

        print(f"  [OK] Generated {len(chunks)} embeddings")

    print("-" * 70)
    print()

    # Upload to Qdrant
    print(f"Uploading {len(all_points)} vectors to Qdrant...")

    # Upload in batches
    batch_size = 100
    for i in range(0, len(all_points), batch_size):
        batch = all_points[i:i + batch_size]
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=batch
        )
        print(f"  -> Uploaded batch {i//batch_size + 1}/{(len(all_points)-1)//batch_size + 1}")

    print(f"[OK] All vectors uploaded successfully!")
    print()

    # Verify
    collection_info = qdrant_client.get_collection(COLLECTION_NAME)
    print("=" * 70)
    print("Summary:")
    print(f"  Collection: {COLLECTION_NAME}")
    print(f"  Total vectors: {collection_info.points_count}")
    print(f"  Vector size: {VECTOR_SIZE}")
    print(f"  Files processed: {len(md_files)}")
    print("=" * 70)
    print()
    print("[OK] Embedding generation complete!")
    print("Your chatbot is now ready to answer questions about the book.")

if __name__ == "__main__":
    main()
