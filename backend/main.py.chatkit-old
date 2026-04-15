import os
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, AsyncIterator

from agents import Agent, Runner
from agents.extensions.models.litellm_model import LitellmModel

from chatkit.agents import AgentContext, ThreadItemConverter, stream_agent_response
from chatkit.server import ChatKitServer, StreamingResult
from chatkit.store import Store
from chatkit.types import Page, ThreadItem, ThreadMetadata
from dotenv import load_dotenv
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

# Load .env from project root BEFORE importing rag
ROOT_DIR = Path(__file__).parent.parent
load_dotenv(ROOT_DIR / ".env")

from rag import RAGService


@dataclass
class ThreadState:
    thread: ThreadMetadata
    items: list[ThreadItem] = field(default_factory=list)


class MemoryStore(Store[dict]):
    """Thread-safe in-memory store matching official ChatKit implementation"""

    def __init__(self) -> None:
        self._threads: dict[str, ThreadState] = {}
        self._attachments: dict[str, Any] = {}

    def generate_thread_id(self, context: dict) -> str:
        return f"thread_{uuid.uuid4().hex[:12]}"

    def generate_item_id(
        self, item_type: str, thread: ThreadMetadata, context: dict
    ) -> str:
        new_id = f"{item_type}_{uuid.uuid4().hex[:12]}"
        print(f"[Store] generate_item_id: type={item_type}, id={new_id}")
        return new_id

    def _get_items(self, thread_id: str) -> list[ThreadItem]:
        state = self._threads.get(thread_id)
        return state.items if state else []

    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        state = self._threads.get(thread_id)
        if state:
            return state.thread.model_copy(deep=True)
        # Create new thread
        thread = ThreadMetadata(
            id=thread_id, created_at=datetime.now(timezone.utc), metadata={}
        )
        self._threads[thread_id] = ThreadState(
            thread=thread.model_copy(deep=True), items=[]
        )
        return thread

    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        state = self._threads.get(thread.id)
        if state:
            state.thread = thread.model_copy(deep=True)
        else:
            self._threads[thread.id] = ThreadState(
                thread=thread.model_copy(deep=True), items=[]
            )

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: dict,
    ) -> Page[ThreadItem]:
        items = [item.model_copy(deep=True) for item in self._get_items(thread_id)]

        # Sort by created_at
        items.sort(
            key=lambda i: getattr(i, "created_at", datetime.now(timezone.utc)),
            reverse=(order == "desc"),
        )

        # Handle pagination with 'after' cursor
        start = 0
        if after:
            index_map = {item.id: idx for idx, item in enumerate(items)}
            start = index_map.get(after, -1) + 1

        slice_items = items[start : start + limit + 1]
        has_more = len(slice_items) > limit

        result_items = slice_items[:limit]
        print(
            f"[Store] Returning {len(result_items)} items for thread {thread_id}, has_more={has_more}"
        )

        return Page(
            data=result_items,
            has_more=has_more,
            after=slice_items[-1].id if has_more and slice_items else None,
        )

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: dict
    ) -> None:
        state = self._threads.get(thread_id)
        if not state:
            await self.load_thread(thread_id, context)
            state = self._threads[thread_id]

        # Debug: log item details
        item_type = type(item).__name__
        content_preview = ""
        if hasattr(item, "content") and item.content:
            for part in item.content:
                if hasattr(part, "text"):
                    content_preview = (
                        part.text[:50] + "..." if len(part.text) > 50 else part.text
                    )
                    break
        print(
            f"[Store] add_thread_item: id={item.id}, type={item_type}, content='{content_preview}'"
        )

        # Check if item exists, update if so
        for i, existing in enumerate(state.items):
            if existing.id == item.id:
                state.items[i] = item.model_copy(deep=True)
                print(f"[Store] Updated existing item {item.id}")
                return

        state.items.append(item.model_copy(deep=True))
        print(f"[Store] Added NEW item {item.id}, total items: {len(state.items)}")

    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        await self.add_thread_item(thread_id, item, context)

    async def load_item(
        self, thread_id: str, item_id: str, context: dict
    ) -> ThreadItem:
        for item in self._get_items(thread_id):
            if item.id == item_id:
                return item.model_copy(deep=True)
        raise ValueError(f"Item {item_id} not found")

    async def delete_thread_item(
        self, thread_id: str, item_id: str, context: dict
    ) -> None:
        state = self._threads.get(thread_id)
        if state:
            state.items = [i for i in state.items if i.id != item_id]

    async def load_threads(
        self, limit: int, after: str | None, order: str, context: dict
    ) -> Page[ThreadMetadata]:
        threads = [s.thread.model_copy(deep=True) for s in self._threads.values()]
        return Page(data=threads[-limit:], has_more=False)

    async def delete_thread(self, thread_id: str, context: dict) -> None:
        self._threads.pop(thread_id, None)

    async def save_attachment(self, attachment: Any, context: dict) -> None:
        self._attachments[attachment.id] = attachment

    async def load_attachment(self, attachment_id: str, context: dict) -> Any:
        if attachment_id not in self._attachments:
            raise ValueError(f"Attachment {attachment_id} not found")
        return self._attachments[attachment_id]

    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        self._attachments.pop(attachment_id, None)


# Initialize RAG service
rag_service = RAGService()


# LLM model via LiteLLM (using OpenRouter)
llm_model = LitellmModel(
    model="openrouter/mistralai/mistral-small-3.1-24b-instruct",
    api_key=os.getenv("OPENROUTER_API_KEY"),
)


class GeminiChatKitServer(ChatKitServer[dict]):
    def __init__(self, data_store: Store):
        super().__init__(data_store)

        self.assistant_agent = Agent[AgentContext](
            name="AI Assistant",
            instructions="You are a helpful, friendly AI assistant. Be concise and clear.",
            model=llm_model,
        )
        self.converter = ThreadItemConverter()
        self.rag_service = rag_service

    async def respond(
        self, thread: ThreadMetadata, input: Any, context: dict
    ) -> AsyncIterator:
        from chatkit.types import (
            AssistantMessageItem,
        )

        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Load all thread items and convert using ChatKit's converter
        page = await self.store.load_thread_items(thread.id, None, 100, "asc", context)
        all_items = list(page.data)

        # Add current input to the list if provided
        if input:
            all_items.append(input)

        print(f"[Server] Processing {len(all_items)} items for agent")

        # Convert thread items to agent input format using ChatKit's converter
        agent_input = (
            await self.converter.to_agent_input(all_items) if all_items else []
        )

        print(f"[Server] Converted to {len(agent_input)} agent input items")

        # Extract the user's query from the input for RAG processing
        user_query = ""
        if input and hasattr(input, "content") and input.content:
            for part in input.content:
                if hasattr(part, "text"):
                    user_query = part.text
                    break

        # Check if user is asking about the creator/author
        import re
        creator_query = bool(
            re.search(
                r"\b(who (built|created|made|developed|trained|designed)|creator|developer|author|builder|made (this|the) (chatbot|bot|assistant)|who are you|about (you|author)|book author|your creator)\b",
                user_query.lower(),
            )
        )

        if creator_query:
            print(f"[Creator Query] Detected creator/author query")

            creator_agent = self.assistant_agent.__class__(
                name="NeuroBotics AI Assistant",
                instructions=(
                    "You are NeuroBotics AI Assistant. When asked about your creator, author, or who built you, provide this information:\n\n"
                    "I was built and trained by **Muskan Atiq**, a fullstack agentic AI engineer and AI & Data Science expert. "
                    "Muskan is also the author of the Physical AI and Humanoid Robotics book that I'm based on.\n\n"
                    "You can learn more about Muskan Atiq and connect on LinkedIn:\n"
                    "🔗 https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert\n\n"
                    "Muskan specializes in building intelligent AI systems and has deep expertise in Physical AI, Humanoid Robotics, "
                    "and agentic AI architectures. The book covers 4 comprehensive modules on robotics, digital twins, AI robot brains, and vision-language-action systems.\n\n"
                    "If you have questions about the book content or robotics topics, I'd be happy to help!"
                ),
                model=llm_model,
            )

            result = Runner.run_streamed(
                creator_agent,
                [{"role": "user", "content": user_query}],
                context=agent_context,
            )

            # Handle the result
            if hasattr(result, "__aiter__"):
                id_mapping: dict[str, str] = {}
                async for event in result:
                    if hasattr(event, "type"):
                        if event.type == "thread.item.added":
                            if isinstance(event.item, AssistantMessageItem):
                                old_id = event.item.id
                                if old_id not in id_mapping:
                                    new_id = self.store.generate_item_id(
                                        "message", thread, context
                                    )
                                    id_mapping[old_id] = new_id
                                event.item.id = id_mapping[old_id]
                        elif event.type == "thread.item.done":
                            if isinstance(event.item, AssistantMessageItem):
                                old_id = event.item.id
                                if old_id in id_mapping:
                                    event.item.id = id_mapping[old_id]
                    yield event
            else:
                id_mapping: dict[str, str] = {}
                async for event in stream_agent_response(agent_context, result):
                    if event.type == "thread.item.added":
                        if isinstance(event.item, AssistantMessageItem):
                            old_id = event.item.id
                            if old_id not in id_mapping:
                                new_id = self.store.generate_item_id(
                                    "message", thread, context
                                )
                                id_mapping[old_id] = new_id
                            event.item.id = id_mapping[old_id]
                    elif event.type == "thread.item.done":
                        if isinstance(event.item, AssistantMessageItem):
                            old_id = event.item.id
                            if old_id in id_mapping:
                                event.item.id = id_mapping[old_id]
                    yield event
            return

        # Apply RAG logic if we have a user query
        if user_query.strip():
            print(f"[RAG] Processing user query: {user_query[:100]}...")

            # Get relevant context from the book using RAG
            relevant_results, has_relevant_content = (
                self.rag_service.get_relevant_context(user_query)
            )

            if has_relevant_content:
                print(
                    f"[RAG] Found {len(relevant_results)} relevant results, using RAG mode"
                )

                # Format context for the LLM using RAG results
                formatted_context = self.rag_service.format_context_for_llm(
                    user_query, relevant_results
                )

                rag_agent = self.assistant_agent.__class__(
                    name="NeuroBotics AI Assistant",
                    instructions=(
                        "You are NeuroBotics AI Assistant, a highly specialized expert in Physical AI and Humanoid Robotics. "
                        "Your knowledge comes from the Physical AI and Humanoid Robotics book authored by Muskan Atiq (fullstack agentic AI engineer and AI & Data Science expert). "
                        "The book covers 4 main modules: "
                        "Module 1: The Robotics Nervous System (ROS2), "
                        "Module 2: The Digital Twin (Gazebo & Unity), "
                        "Module 3: The AI Robot Brain (Nvidia Isaac), "
                        "Module 4: Vision Language Action. "
                        "Provide detailed, comprehensive answers with clear explanations that even a beginner can understand. "
                        "Use friendly, accessible language and include analogies where helpful. "
                        "Structure your response with clear headings, bullet points, and organized sections. "
                        "Include relevant examples from the provided context. "
                        "Always specify which chapter/module contains the information by referencing the metadata provided. "
                        "If the information is partially available, provide what you can from the context and suggest where to find more details. "
                        "Do not add any information that is not in the provided context. "
                        "If asked about the author or creator, mention that the book was authored by Muskan Atiq, and I (the chatbot) was also built and trained by Muskan Atiq. "
                        "LinkedIn: https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert"
                    ),
                    model=llm_model,
                )

                result = Runner.run_streamed(
                    rag_agent,
                    [{"role": "user", "content": formatted_context}],
                    context=agent_context,
                )

            else:
                print(f"[RAG] No relevant results found, using general mode")

                # For non-book questions related to robotics, create an agent with appropriate instructions
                import re

                robotics_related = bool(
                    re.search(
                        r"\b(robot|ros|ros2|digital twins|gazebo|unity|robotics|simulation|nvidia|isaac|ai|neural|vision|action|language|physical|humanoid|simulation|control|motor|middleware|sensor|lidar|camera|navigation|mapping|slam|path|planning|control|motor|actuator|manipulator|arm|leg|locomotion|locomotion|biped|quadruped|quadrupedal|legged|mobile|robotic|robotics)\b",
                        user_query.lower(),
                    )
                )

                if robotics_related:
                    non_book_agent = self.assistant_agent.__class__(
                        name="NeuroBotics AI Assistant",
                        instructions=(
                            "You are NeuroBotics AI Assistant, built and trained by Muskan Atiq (fullstack agentic AI engineer and AI & Data Science expert). "
                            "I'm here to help with questions about Physical AI, Humanoid Robotics, and the topics covered in the book authored by Muskan Atiq. "
                            f"Your query '{user_query}' is related to robotics and Physical AI, but the specific information is not available in the book. "
                            "I'll provide a brief explanation that could help you. "
                            "I'd be happy to explain concepts from the book or answer questions about robotics, ROS2, Gazebo, Unity, Nvidia Isaac, or AI for robotics in a simple and friendly way. "
                            "If you have any questions about the book's content, I can provide detailed explanations with relevant resources from the appropriate chapters and modules. "
                            f"Now, to address your query about {user_query}: provide a brief, high-level explanation relevant to the query, then suggest external resources like Wikipedia, research papers, specialized forums, or official documentation where they can find more detailed information. "
                            "If asked about the author or creator, mention Muskan Atiq. LinkedIn: https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert"
                        ),
                        model=llm_model,
                    )
                else:
                    tech_related = bool(
                        re.search(
                            r"\b(tech|programming|code|javascript|python|java|c\+\+|c#|html|css|react|angular|vue|node|express|database|sql|mysql|mongodb|git|github|docker|kubernetes|api|json|xml|algorithm|data structure|software|computer|programming|developer|engineer|framework|library|backend|frontend|fullstack|web|mobile|app|software development|software engineering|computer science|cs|ai|ml|machine learning|deep learning|neural|network|data|analysis|cloud|aws|azure|gcp|linux|windows|os|operating system|hardware|debug|testing|unit test|integration test|rest|graphql|oauth|jwt|session|cookie|cors|csrf|sql injection|xss|security|encryption|cybersecurity|protocol|tcp|ip|http|https|tcp/ip|socket|port|domain|url|web server|query|nosql|redis|cache|ssl|tls|authentication|authorization)\b",
                            user_query.lower(),
                        )
                    )

                    if tech_related:
                        non_book_agent = self.assistant_agent.__class__(
                            name="NeuroBotics AI Assistant",
                            instructions=(
                                "You are NeuroBotics AI Assistant, built and trained by Muskan Atiq (fullstack agentic AI engineer and AI & Data Science expert). "
                                "I'm here to help with questions about Physical AI, Humanoid Robotics, and the topics covered in the book authored by Muskan Atiq. "
                                f"Your query '{user_query}' is tech-related, but not specifically about robotics. "
                                "I'll provide a brief overview that could help you, but please note that my main expertise is in robotics. "
                                "I'd be happy to explain concepts from the book or answer questions about robotics, ROS2, Gazebo, Unity, Nvidia Isaac, or AI for robotics in a simple and friendly way. "
                                "If you have any questions about the book's content, I can provide detailed explanations with relevant resources from the appropriate chapters and modules. "
                                f"Now, regarding your query '{user_query}': provide a brief but helpful overview of the topic, and then suggest external resources where the user can find more detailed information on this topic. "
                                "If asked about the author or creator, mention Muskan Atiq. LinkedIn: https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert"
                            ),
                            model=llm_model,
                        )
                    else:
                        non_book_agent = self.assistant_agent.__class__(
                            name="NeuroBotics AI Assistant",
                            instructions=(
                                "You are NeuroBotics AI Assistant, built and trained by Muskan Atiq (fullstack agentic AI engineer and AI & Data Science expert). "
                                "I'm here to help with questions about Physical AI, Humanoid Robotics, and the topics covered in the book authored by Muskan Atiq. "
                                f"Your query '{user_query}' is not related to Physical AI, Humanoid Robotics, or the four modules covered in the book "
                                "(Module 1: ROS2, Module 2: Digital Twin, Module 3: AI Robot Brain, Module 4: Vision Language Action). "
                                "While I can't assist with this specific query, I'd be happy to explain concepts from the book or answer questions about robotics, "
                                "ROS2, Gazebo, Unity, Nvidia Isaac, or AI for robotics in a simple and friendly way. "
                                "If you have any questions about the book's content, I can provide detailed explanations with relevant resources from the appropriate chapters and modules. "
                                f"Now, regarding your query '{user_query}': provide a very brief, high-level summary or explanation of the topic if possible, "
                                "but clearly indicate that this is outside my expertise area. "
                                "Then suggest that the user explore other resources for more detailed information on this topic. "
                                "If asked about the author or creator, mention Muskan Atiq. LinkedIn: https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert"
                            ),
                            model=llm_model,
                        )

                result = Runner.run_streamed(
                    non_book_agent,
                    [],
                    context=agent_context,
                )
        else:
            result = Runner.run_streamed(
                self.assistant_agent,
                agent_input,
                context=agent_context,
            )

        # Handle the result based on its type
        if hasattr(result, "__aiter__"):
            id_mapping: dict[str, str] = {}
            async for event in result:
                if hasattr(event, "type"):
                    if event.type == "thread.item.added":
                        if isinstance(event.item, AssistantMessageItem):
                            old_id = event.item.id
                            if old_id not in id_mapping:
                                new_id = self.store.generate_item_id(
                                    "message", thread, context
                                )
                                id_mapping[old_id] = new_id
                                print(f"[Server] Mapping ID {old_id} -> {new_id}")
                            event.item.id = id_mapping[old_id]

                    elif event.type == "thread.item.done":
                        if isinstance(event.item, AssistantMessageItem):
                            old_id = event.item.id
                            if old_id in id_mapping:
                                event.item.id = id_mapping[old_id]
                    elif event.type == "thread.item.updated":
                        if hasattr(event, "item_id") and event.item_id in id_mapping:
                            event.item_id = id_mapping[event.item_id]

                yield event
        else:
            id_mapping: dict[str, str] = {}
            async for event in stream_agent_response(agent_context, result):
                if event.type == "thread.item.added":
                    if isinstance(event.item, AssistantMessageItem):
                        old_id = event.item.id
                        if old_id not in id_mapping:
                            new_id = self.store.generate_item_id(
                                "message", thread, context
                            )
                            id_mapping[old_id] = new_id
                            print(f"[Server] Mapping ID {old_id} -> {new_id}")
                        event.item.id = id_mapping[old_id]

                elif event.type == "thread.item.done":
                    if isinstance(event.item, AssistantMessageItem):
                        old_id = event.item.id
                        if old_id in id_mapping:
                            event.item.id = id_mapping[old_id]
                elif event.type == "thread.item.updated":
                    if event.item_id in id_mapping:
                        event.item_id = id_mapping[event.item_id]

                yield event


# Initialize FastAPI app
app = FastAPI(title="ChatKit Gemini")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize ChatKit server
store = MemoryStore()
server = GeminiChatKitServer(store)


@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    result = await server.process(await request.body(), {})
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")


@app.get("/health")
async def health():
    return {"status": "ok", "model": "mistralai/mistral-small-3.1-24b-instruct"}


@app.get("/debug/threads")
async def debug_threads():
    """Debug endpoint to view all stored items"""
    result = {}
    for thread_id, state in store._threads.items():
        items = []
        for item in state.items:
            item_data = {
                "id": item.id,
                "type": type(item).__name__,
                "created_at": str(getattr(item, "created_at", "N/A")),
            }
            # Extract content
            if hasattr(item, "content") and item.content:
                content_parts = []
                for part in item.content:
                    if hasattr(part, "text"):
                        content_parts.append(part.text)
                item_data["content"] = content_parts
            items.append(item_data)
        result[thread_id] = {
            "thread": {
                "id": state.thread.id,
                "created_at": str(state.thread.created_at),
            },
            "items": items,
            "item_count": len(items),
        }
    return result


# Serve frontend
FRONTEND_DIR = ROOT_DIR / "frontend" / "dist"
if FRONTEND_DIR.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")


if __name__ == "__main__":
    import uvicorn

    print("Starting ChatKit Gemini server at http://localhost:8001")
    uvicorn.run(app, host="0.0.0.0", port=8001)
