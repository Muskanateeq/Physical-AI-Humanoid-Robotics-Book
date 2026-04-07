# Data Model: Physical AI & Humanoid Robotics Book Semantic Search System

## Entities

### BookDocument
Represents a chapter from the "Physical AI & Humanoid Robotics" Docusaurus book (total of 19 chapters across 5 modules)

- **id**: string (unique identifier for the document)
- **file_path**: string (relative path from frontend/docs directory, e.g., "module-1-ros2/chapter1.md")
- **title**: string (document title extracted from markdown)
- **module**: string (book module: "00-neurobotics-overview", "module-1-ros2", "module-2-simulation", "module-3-aibrain", or "module-4-vla")
- **chapter**: string (chapter name/number within the module)
- **content**: string (full text content of the document)
- **created_at**: datetime (when the document was first processed)
- **updated_at**: datetime (when the document was last modified)
- **checksum**: string (content hash for change detection)
- **metadata**: dict (additional metadata from markdown frontmatter)

### VectorRepresentation
Represents the vector representation of document content

- **id**: string (unique identifier for the vector)
- **document_id**: string (foreign key to BookDocument)
- **content_chunk**: string (portion of the document content that was embedded)
- **vector**: list[float] (the embedding vector values)
- **chunk_index**: int (position of this chunk in the original document)
- **created_at**: datetime (when the vector was created)
- **metadata**: dict (additional metadata for search filtering, including module and chapter information)

### QdrantCollection
Represents the storage container in Qdrant database for the vectors

- **collection_name**: string (name of the Qdrant collection, e.g., "physical_ai_humanoid_robotics")
- **vector_size**: int (dimension of the embedding vectors)
- **distance_metric**: string (cosine, euclidean, etc.)
- **created_at**: datetime (when the collection was created)
- **config**: dict (Qdrant collection configuration options)

## Relationships

- One BookDocument can have many VectorRepresentations (1 to many)
- One QdrantCollection can store many VectorRepresentations (1 to many)

## Validation Rules

### BookDocument
- file_path must be a valid path under frontend/docs and belong to one of the 5 book modules
- content must not exceed 10MB
- id must be unique across all documents
- module must be one of: "00-neurobotics-overview", "module-1-ros2", "module-2-simulation", "module-3-aibrain", "module-4-vla"
- Each of the 19 chapters across all modules must have individual embeddings

### VectorRepresentation
- document_id must reference an existing BookDocument
- vector must have the correct dimensionality for the collection
- chunk_index must be non-negative

### QdrantCollection
- collection_name must be unique
- vector_size must match the embedding model used
- distance_metric must be one of the supported values

## State Transitions

### BookDocument States
- PENDING: Document identified but not yet processed
- PROCESSING: Document content is being converted to vectors
- PROCESSED: Document has been successfully embedded
- FAILED: Document processing failed
- UPDATED: Document has been modified since last processing

## Indexes

- BookDocument.file_path (for quick lookup by path)
- BookDocument.module (for filtering by book module)
- BookDocument.chapter (for filtering by chapter within module)
- BookDocument.checksum (for change detection)
- VectorRepresentation.document_id (for document-to-vectors lookup)
- VectorRepresentation.chunk_index (for reassembling document order)