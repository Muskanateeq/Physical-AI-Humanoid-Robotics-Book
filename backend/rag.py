# """
# RAG (Retrieval-Augmented Generation) module for book-grounded chatbot.
#
# This module provides functionality to:
# 1. Embed user queries using fastembed
# 2. Search Qdrant vector database for relevant book content
# 3. Determine relevance using configurable thresholds
# 4. Format context for LLM consumption
# """
# import os
# from typing import List, Dict, Optional, Tuple
# from dataclasses import dataclass
#
# from qdrant_client import QdrantClient
# from qdrant_client.http import models
# from fastembed import TextEmbedding
#
#
# @dataclass
# class SearchResult:
#     """Represents a single search result from Qdrant"""
#     text: str
#     score: float
#     metadata: Dict
#
#
# class RAGService:
#     """Service class to handle RAG operations"""
#
#     def __init__(self):
#         # Initialize Qdrant client with environment configuration
#         self.qdrant_client = QdrantClient(
#             url=os.getenv("QDRANT_URL"),
#             api_key=os.getenv("QDRANT_API_KEY"),
#             prefer_grpc=False  # Using HTTP for simplicity
#         )
#
#         self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_robotics")
#
#         # Initialize embedding model (using the same model as was likely used for book embeddings)
#         # Using a common model that works well for most use cases
#         self.embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
#
#         # Configurable similarity threshold (0.0 to 1.0, where higher is more strict)
#         # Lowering default threshold to be more inclusive for better user experience
#         self.similarity_threshold = float(os.getenv("RAG_SIMILARITY_THRESHOLD", "0.1"))
#
#     def embed_query(self, query: str) -> List[float]:
#         """
#         Convert a text query into an embedding vector.
#
#         Args:
#             query: The user's text query
#
#         Returns:
#             A list of floats representing the embedding vector
#         """
#         # Generate embedding for the query
#         embeddings = list(self.embedding_model.embed([query]))
#         return embeddings[0]  # Return the first (and only) embedding
#
#     def search_qdrant(self, query_embedding: List[float], top_k: int = 5) -> List[SearchResult]:
#         """
#         Search the Qdrant collection for relevant book content.
#
#         Args:
#             query_embedding: The embedding vector for the user's query
#             top_k: Number of top results to retrieve
#
#         Returns:
#             List of SearchResult objects containing text, score, and metadata
#         """
#         # Perform vector search in Qdrant
#         search_results = self.qdrant_client.search(
#             collection_name=self.collection_name,
#             query_vector=query_embedding,
#             limit=top_k,
#             with_payload=True  # Include the original text content
#         )
#
#         results = []
#         for hit in search_results:
#             # Extract the text from payload (assuming it's stored in a 'text' field)
#             text_content = hit.payload.get("text", "") if hit.payload else ""
#             metadata = hit.payload or {}
#
#             result = SearchResult(
#                 text=text_content,
#                 score=hit.score,
#                 metadata=metadata
#             )
#             results.append(result)
#
#         return results
#
#     def is_relevant(self, score: float) -> bool:
#         """
#         Determine if a search result is relevant based on similarity threshold.
#
#         Args:
#             score: The similarity score from Qdrant (higher is better)
#
#         Returns:
#             True if the result is above the relevance threshold
#         """
#         return score >= self.similarity_threshold
#
#     def get_relevant_context(self, query: str, top_k: int = 5) -> Tuple[List[SearchResult], bool]:
#         """
#         Main method to get relevant book content for a user query.
#
#         Args:
#             query: The user's text query
#             top_k: Number of top results to retrieve
#
#         Returns:
#             A tuple containing:
#             - List of relevant SearchResult objects
#             - Boolean indicating if any relevant results were found
#         """
#         # Step 1: Embed the user query
#         query_embedding = self.embed_query(query)
#
#         # Step 2: Search Qdrant for relevant content
#         search_results = self.search_qdrant(query_embedding, top_k)
#
#         # Step 3: Filter results based on relevance threshold
#         relevant_results = [result for result in search_results if self.is_relevant(result.score)]
#
#         # Step 4: Return results and whether any were found
#         has_relevant_content = len(relevant_results) > 0
#
#         return relevant_results, has_relevant_content
#
#     def format_context_for_llm(self, query: str, relevant_results: List[SearchResult]) -> str:
#         """
#         Format the retrieved context for LLM consumption.
#
#         Args:
#             query: The original user query
#             relevant_results: List of relevant search results
#
#         Returns:
#             A formatted string containing the context for the LLM
#         """
#         if not relevant_results:
#             return ""
#
#         # Create a context string with the relevant book content
#         context_parts = [
#             "You are an expert assistant for the Physical AI and Humanoid Robotics book. Answer the user's question using ONLY the provided context from the book.",
#             "Provide detailed, comprehensive answers based on the book content.",
#             "Use clear headings, bullet points, and structured format where appropriate.",
#             "Include specific details, examples, and explanations from the context.",
#             "If the information is partially available, provide what you can from the context.",
#             "Do not add any information that is not in the provided context.",
#             "",
#             "BOOK CONTENT FOR REFERENCE:",
#         ]
#
#         # Sort results by relevance score (highest first) for better context ordering
#         sorted_results = sorted(relevant_results, key=lambda x: x.score, reverse=True)
#
#         for i, result in enumerate(sorted_results, 1):
#             context_parts.append(f"\n{i}. [Relevance Score: {result.score:.3f}]")
#             context_parts.append(f"Source Content: {result.text}")
#             context_parts.append("---")  # Separator between different sources
#
#         context_parts.append(f"\nUSER QUESTION: {query}")
#         context_parts.append("\nDETAILED ANSWER (based ONLY on the above book content):")
#
#         return "\n".join(context_parts)
#


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
            "physical_ai_humanoid_robotics",
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
