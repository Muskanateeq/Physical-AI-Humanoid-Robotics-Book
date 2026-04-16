"""
Chat API endpoint with RAG and OpenRouter integration.
"""
import os
import json
import sys
from pathlib import Path
from typing import AsyncGenerator
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import httpx
from dotenv import load_dotenv

# Add backend directory to path for RAG import
backend_dir = Path(__file__).parent.parent.parent
sys.path.insert(0, str(backend_dir))

from rag import RAGService

load_dotenv()

router = APIRouter()

# Initialize RAG service
rag_service = RAGService()

# OpenRouter configuration
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
OPENROUTER_BASE_URL = "https://openrouter.ai/api/v1"
DEFAULT_MODEL = os.getenv("OPENROUTER_MODEL", "mistralai/mistral-small-3.1-24b-instruct")


class ChatRequest(BaseModel):
    message: str
    model: str = DEFAULT_MODEL


@router.post("/chat")
async def chat(request: ChatRequest):
    """
    Chat endpoint with RAG integration and streaming response.
    """
    if not OPENROUTER_API_KEY:
        raise HTTPException(status_code=500, detail="OpenRouter API key not configured")

    try:
        # Check if user is asking about the creator/author
        import re
        creator_query = bool(
            re.search(
                r"\b(who (built|created|made|developed|trained|designed)|creator|developer|author|builder|made (this|the) (chatbot|bot|assistant)|who are you|about (you|author)|book author|your creator)\b",
                request.message.lower(),
            )
        )

        if creator_query:
            # Special system prompt for creator queries
            system_prompt = """You are NeuroBotics AI Assistant. When asked about your creator, author, or who built you, provide this information:

I was built and trained by **Muskan Atiq**, a fullstack agentic AI engineer and AI & Data Science expert. Muskan is also the author of the Physical AI and Humanoid Robotics book that I'm based on.

You can learn more about Muskan Atiq and connect on LinkedIn:
🔗 https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert

Muskan specializes in building intelligent AI systems and has deep expertise in Physical AI, Humanoid Robotics, and agentic AI architectures. The book covers 4 comprehensive modules on robotics, digital twins, AI robot brains, and vision-language-action systems.

If you have questions about the book content or robotics topics, I'd be happy to help!"""
        else:
            # Step 1: Get relevant context from RAG
            relevant_results, is_book_related = rag_service.get_relevant_context(
                query=request.message,
                top_k=5
            )

            # Step 2: Format context for LLM
            if is_book_related and relevant_results:
                system_prompt = rag_service.format_context_for_llm(
                    query=request.message,
                    results=relevant_results
                )
            else:
                # General assistant mode
                system_prompt = """You are NeuroBotics AI Assistant, built and trained by Muskan Atiq (fullstack agentic AI engineer and AI & Data Science expert), a friendly expert in Physical AI and Humanoid Robotics.
The user's question is not directly related to the Physical AI and Humanoid Robotics book content.
Provide helpful, accurate information about robotics, AI, or related topics.
If the question is completely unrelated to robotics/AI, politely explain your specialization and offer to help with robotics topics.
If asked about the author or creator, mention Muskan Atiq. LinkedIn: https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert"""

        # Step 3: Stream response from OpenRouter
        async def generate_stream() -> AsyncGenerator[bytes, None]:
            async with httpx.AsyncClient(timeout=60.0) as client:
                async with client.stream(
                    "POST",
                    f"{OPENROUTER_BASE_URL}/chat/completions",
                    headers={
                        "Authorization": f"Bearer {OPENROUTER_API_KEY}",
                        "Content-Type": "application/json",
                        "HTTP-Referer": os.getenv("APP_URL", "https://neurobotics-ai-book.vercel.app"),
                        "X-Title": "NeuroBotics AI Assistant"
                    },
                    json={
                        "model": request.model,
                        "messages": [
                            {"role": "system", "content": system_prompt},
                            {"role": "user", "content": request.message}
                        ],
                        "stream": True,
                        "temperature": 0.7,
                        "max_tokens": 2000
                    }
                ) as response:
                    if response.status_code != 200:
                        error_text = await response.aread()
                        raise HTTPException(
                            status_code=response.status_code,
                            detail=f"OpenRouter API error: {error_text.decode()}"
                        )

                    async for line in response.aiter_lines():
                        if line.startswith("data: "):
                            data = line[6:]  # Remove "data: " prefix
                            if data.strip() == "[DONE]":
                                break
                            # Forward the SSE data as-is
                            yield f"data: {data}\n\n".encode()

        return StreamingResponse(
            generate_stream(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
                "X-Book-Related": str(is_book_related)
            }
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/chat/health")
async def chat_health():
    """Health check for chat service"""
    return {
        "status": "healthy",
        "rag_enabled": True,
        "openrouter_configured": bool(OPENROUTER_API_KEY),
        "model": DEFAULT_MODEL
    }
