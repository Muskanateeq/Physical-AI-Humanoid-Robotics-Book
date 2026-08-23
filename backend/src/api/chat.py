"""
Chat API endpoint with RAG and OpenRouter integration.
"""
import os
import json
from typing import AsyncGenerator
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import httpx
from dotenv import load_dotenv

load_dotenv()

# Create router first (before any imports that might fail)
router = APIRouter()

# Try to import and initialize RAG service
RAG_AVAILABLE = False
rag_service = None

try:
    import sys
    from pathlib import Path
    backend_dir = Path(__file__).parent.parent.parent
    sys.path.insert(0, str(backend_dir))
    from rag import RAGService
    rag_service = RAGService()
    RAG_AVAILABLE = True
    print("✓ RAG service initialized successfully")
except Exception as e:
    print(f"⚠ RAG service not available: {e}")
    print("  Chat will work without RAG context")

# OpenRouter configuration
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
OPENROUTER_BASE_URL = "https://openrouter.ai/api/v1"
DEFAULT_MODEL = os.getenv("OPENROUTER_MODEL", "openrouter/free")
FALLBACK_MODELS = tuple(
    model.strip()
    for model in os.getenv(
        "OPENROUTER_FALLBACK_MODELS",
        "nvidia/nemotron-3-ultra-550b-a55b:free",
    ).split(",")
    if model.strip()
)

PLAIN_TEXT_RESPONSE_RULES = """Response formatting requirements:

1. Return clean, professional plain text only.
2. Do not use Markdown syntax, Markdown headings, emphasis markers, code fences, or Markdown tables.
3. Write headings as normal text without decorative symbols.
4. Use numbered lists with normal numbers when a list is needed.
5. Do not use bullets, hashtags, asterisks, underscores, backticks, or tildes for formatting.
6. Keep paragraphs readable with appropriate spacing and normal punctuation.
7. Do not add unnecessary symbols before or after text.
8. Express emphasis naturally through wording instead of special formatting.
9. Before responding, review the complete answer and remove any remaining Markdown formatting.
10. The final answer must look like clean human-written plain text."""


def get_model_candidates(primary_model: str) -> list[str]:
    """Return the primary and fallback models in priority order, without duplicates."""
    return list(dict.fromkeys((primary_model, *FALLBACK_MODELS)))


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
        # Initialize is_book_related (used in response headers)
        is_book_related = False

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
            # Step 1: Get relevant context from RAG (if available)
            if RAG_AVAILABLE and rag_service:
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
            else:
                # RAG not available, use general mode
                system_prompt = """You are NeuroBotics AI Assistant, built and trained by Muskan Atiq (fullstack agentic AI engineer and AI & Data Science expert), a friendly expert in Physical AI and Humanoid Robotics.
Provide helpful, accurate information about robotics, AI, or related topics.
If asked about the author or creator, mention Muskan Atiq. LinkedIn: https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert"""

        # Apply the same response style to creator, RAG, and general answers.
        system_prompt = f"{system_prompt.rstrip()}\n\n{PLAIN_TEXT_RESPONSE_RULES}"

        # Step 3: Stream response from OpenRouter
        async def generate_stream() -> AsyncGenerator[bytes, None]:
            async with httpx.AsyncClient(timeout=60.0) as client:
                model_candidates = get_model_candidates(request.model)
                candidate_index = 0

                # OpenRouter normally handles model fallbacks before streaming.
                # If a provider fails after streaming has started, retry once with
                # the next model and tell the client to discard the partial text.
                for attempt in range(2):
                    retry_requested = False
                    remaining_models = model_candidates[candidate_index:]

                    if not remaining_models:
                        yield b'data: {"type":"error","message":"No AI model is currently available. Please try again shortly."}\n\n'
                        break

                    request_body = {
                        "model": remaining_models[0],
                        "messages": [
                            {"role": "system", "content": system_prompt},
                            {"role": "user", "content": request.message}
                        ],
                        "stream": True,
                        "temperature": 0.7,
                        "max_tokens": 8000,
                        "provider": {"allow_fallbacks": True}
                    }

                    # OpenRouter tries these models in order when the primary
                    # model/provider fails before response streaming begins.
                    if len(remaining_models) > 1:
                        request_body["models"] = remaining_models[1:]

                    async with client.stream(
                        "POST",
                        f"{OPENROUTER_BASE_URL}/chat/completions",
                        headers={
                            "Authorization": f"Bearer {OPENROUTER_API_KEY}",
                            "Content-Type": "application/json",
                            "HTTP-Referer": os.getenv("APP_URL", "https://neurobotics-ai-book.vercel.app"),
                            "X-Title": "NeuroBotics AI Assistant"
                        },
                        json=request_body
                    ) as response:
                        if response.status_code != 200:
                            await response.aread()
                            if attempt == 0 and len(remaining_models) > 1:
                                candidate_index += 1
                                retry_requested = True
                                yield b'data: {"type":"retry"}\n\n'
                            else:
                                yield b'data: {"type":"error","message":"The AI service is temporarily unavailable. Please try again."}\n\n'
                            continue

                        async for line in response.aiter_lines():
                            if not line.startswith("data: "):
                                continue

                            data = line[6:]  # Remove "data: " prefix
                            if data.strip() == "[DONE]":
                                break

                            try:
                                event = json.loads(data)
                                choices = event.get("choices") or []
                                first_choice = choices[0] if choices else {}
                                finish_reason = first_choice.get("finish_reason")
                                provider_error = event.get("error")
                            except (json.JSONDecodeError, IndexError, AttributeError):
                                # Forward non-standard provider events unchanged.
                                event = None
                                finish_reason = None
                                provider_error = None

                            if finish_reason == "error" or provider_error:
                                failed_model = event.get("model") if event else None
                                if failed_model in model_candidates:
                                    candidate_index = model_candidates.index(failed_model) + 1
                                else:
                                    candidate_index += 1

                                has_fallback = candidate_index < len(model_candidates)
                                if attempt == 0 and has_fallback:
                                    retry_requested = True
                                    yield b'data: {"type":"retry"}\n\n'
                                else:
                                    yield b'data: {"type":"error","message":"All configured AI models are temporarily unavailable. Please try again shortly."}\n\n'
                                break

                            # Forward the complete SSE data event as-is.
                            yield f"data: {data}\n\n".encode()

                    if not retry_requested:
                        break

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
        "model": DEFAULT_MODEL,
        "fallback_models": FALLBACK_MODELS
    }
