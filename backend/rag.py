
"""
RAG (Retrieval-Augmented Generation) module for book-grounded chatbot.

Responsibilities:
1. Embed user queries using fastembed
2. Search Qdrant vector database
3. Decide book relevance safely
4. Prepare TEXT-only context for LLM
"""

import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

from fastembed import TextEmbedding
from qdrant_client import QdrantClient
from qdrant_client.http import models

# -----------------------------
# Data Model
# -----------------------------


@dataclass
class SearchResult:
    text: str
    score: float
    metadata: Dict


# -----------------------------
# RAG Service
# -----------------------------


class RAGService:
    def __init__(self):
        # Qdrant config - handling both cloud and local instances
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url:
            # Cloud instance
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False,
            )
        else:
            # Local instance
            self.qdrant_client = QdrantClient(
                host="localhost",
                port=6333,
            )

        self.collection_name = os.getenv(
            "QDRANT_COLLECTION_NAME",
            "neurobotics-physical-ai-humanoid-robotics-book",
        )

        # MUST match book embedding model
        self.embedding_model = TextEmbedding(
            model_name=os.getenv(
                "EMBEDDING_MODEL",
                "BAAI/bge-small-en-v1.5",
            )
        )

        # Safe default threshold
        self.similarity_threshold = float(os.getenv("RAG_SIMILARITY_THRESHOLD", "0.6"))

    # -----------------------------
    # Embedding
    # -----------------------------

    def embed_query(self, query: str) -> List[float]:
        return list(self.embedding_model.embed([query]))[0]

    # -----------------------------
    # Vector Search
    # -----------------------------

    def search_qdrant(
        self,
        query_embedding: List[float],
        top_k: int = 5,
    ) -> List[SearchResult]:
        hits = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
        )

        results: List[SearchResult] = []

        for hit in hits:
            payload = hit.payload or {}

            # ✅ CORRECT FIELD
            text = payload.get("content_chunk", "").strip()
            if not text:
                continue

            results.append(
                SearchResult(
                    text=text,
                    score=hit.score,
                    metadata=payload,
                )
            )

        return results

    # -----------------------------
    # Relevance Decision (TOP-1)
    # -----------------------------

    def is_book_related(self, results: List[SearchResult]) -> bool:
        if not results:
            return False
        return results[0].score >= self.similarity_threshold

    # -----------------------------
    # Public API
    # -----------------------------

    def get_relevant_context(
        self,
        query: str,
        top_k: int = 5,
    ) -> Tuple[List[SearchResult], bool]:
        query_embedding = self.embed_query(query)
        results = self.search_qdrant(query_embedding, top_k)

        results.sort(key=lambda r: r.score, reverse=True)

        is_relevant = self.is_book_related(results)

        return (results if is_relevant else []), is_relevant

    # -----------------------------
    # LLM Context Formatting
    # -----------------------------

    def format_context_for_llm(
        self,
        query: str,
        results: List[SearchResult],
    ) -> str:
        context = [
            "You are NeuroBotics AI Assistant, a highly specialized expert in Physical AI and Humanoid Robotics. You are designed to help users understand complex robotics concepts in an easy, friendly, and comprehensive way.",
            "Your knowledge comes from the Physical AI and Humanoid Robotics book authored by Muskan Atiq, a fullstack agentic AI engineer and AI & Data Science expert.",
            "The book covers 4 main modules:",
            "Module 1: The Robotics Nervous System (ROS2)",
            "Module 2: The Digital Twin (Gazebo & Unity)",
            "Module 3: The AI Robot Brain (Nvidia Isaac)",
            "Module 4: Vision Language Action",
            "",
            "About the Author & Creator:",
            "- Book Author: Muskan Atiq",
            "- Chatbot Creator: Muskan Atiq (fullstack agentic AI engineer, AI & Data Science expert)",
            "- LinkedIn: https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert",
            "",
            "Your response guidelines:",
            "1. Provide detailed, comprehensive answers with clear explanations that even a beginner can understand",
            "2. Use friendly, accessible language and include analogies where helpful",
            "3. Structure your response with clear headings, bullet points, and organized sections",
            "4. Include relevant examples from the provided context",
            "5. If the information is partially available, provide what you can and suggest where to find more details",
            "6. Always include relevant resources and specify which chapter/module contains the information",
            "7. If the query is outside the book scope but related to Physical AI or Humanoid Robotics, provide general guidance and suggest external resources like Wikipedia, research papers, or specialized forums",
            "8. If the query is completely unrelated, politely explain your specialization and offer help with robotics topics",
            "9. If asked about the author or creator, mention Muskan Atiq and provide the LinkedIn profile link",
            "",
            "BOOK CONTEXT (from Physical AI and Humanoid Robotics by Muskan Atiq):",
        ]

        for i, r in enumerate(results, 1):
            meta = r.metadata
            module_info = f"Module: {meta.get('module', 'N/A')} | Chapter: {meta.get('chapter', 'N/A')} | File: {meta.get('file_path', 'N/A')}"
            context.append(f"\n{i}. {module_info}")
            context.append(f"Content: {r.text}")
            context.append("---")

        context.append(f"\nUSER QUESTION: {query}")
        context.append("\nDETAILED RESPONSE:")
        # Add a separator to help prevent token contamination
        context.append("---")

        return "\n".join(context)
