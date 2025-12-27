"""
Agent service using Gemini via OpenAI-compatible endpoint.

Provides AI-powered question answering using Gemini 2.0 Flash Exp model
with retrieve() tool for accessing textbook content.
"""

import json
from typing import List, Tuple

from openai import AsyncOpenAI

from src.config import settings
from src.logger import logger
from src.models.ingest import TextChunk
from src.services.retrieval import RetrievalService
from src.utils.exceptions import GeminiAPIError


class AgentService:
    """
    Service for AI-powered question answering with Gemini.

    Uses OpenAI SDK configured for Gemini's OpenAI-compatible endpoint.
    Implements retrieve() tool for RAG (Retrieval Augmented Generation).

    Attributes:
        client: AsyncOpenAI client configured for Gemini endpoint
        model: Gemini model name
        system_prompt: System instructions enforcing textbook-only responses
        retrieval_service: Service for retrieving textbook chunks
    """

    def __init__(self, retrieval_service: RetrievalService):
        """
        Initialize the agent service.

        Args:
            retrieval_service: Service for retrieving textbook content
        """
        self.client = AsyncOpenAI(
            api_key=settings.GEMINI_API_KEY,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        self.model = "gemini-1.5-flash"
        self.retrieval_service = retrieval_service

        # System prompt enforcing textbook-only responses
        self.system_prompt = """You are an AI tutor for a Physical AI textbook.

CRITICAL RULES:
1. Use the retrieve() tool to search for relevant content before answering
2. Answer questions ONLY using information from the retrieved textbook chunks
3. If the information is not in the textbook, respond exactly: "I don't have information about this in the textbook"
4. Always cite your sources by mentioning which sections or topics you used
5. Do not use external knowledge or make assumptions beyond what's in the textbook
6. Be concise and accurate in your responses"""

    async def answer_question(
        self,
        query: str
    ) -> Tuple[str, List[str]]:
        """
        Answer a user question using textbook content.

        Uses Gemini with retrieve() tool to find relevant content and
        generate an accurate, sourced answer.

        Args:
            query: User's question

        Returns:
            Tuple of (answer, source_urls)

        Raises:
            GeminiAPIError: If Gemini API call fails
        """
        try:
            logger.info(
                "Starting agent question answering",
                extra={"query_length": len(query)}
            )

            # Define the retrieve() tool
            tools = [
                {
                    "type": "function",
                    "function": {
                        "name": "retrieve",
                        "description": "Search the Physical AI textbook for relevant content. Use this tool before answering any question.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "query": {
                                    "type": "string",
                                    "description": "The semantic search query to find relevant textbook content"
                                }
                            },
                            "required": ["query"]
                        }
                    }
                }
            ]

            # First API call - agent should call retrieve() tool
            messages = [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": query}
            ]

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                tools=tools,
                tool_choice="auto"
            )

            # Check if agent wants to call retrieve() tool
            assistant_message = response.choices[0].message

            if assistant_message.tool_calls:
                # Extract tool call arguments
                tool_call = assistant_message.tool_calls[0]
                function_name = tool_call.function.name
                function_args = json.loads(tool_call.function.arguments)

                logger.info(
                    "Agent calling tool",
                    extra={
                        "tool": function_name,
                        "args": function_args
                    }
                )

                if function_name == "retrieve":
                    # Execute retrieval
                    retrieval_query = function_args.get("query", query)
                    retrieval_result = await self.retrieval_service.retrieve(
                        retrieval_query
                    )

                    # Format retrieved chunks for context
                    context_parts = []
                    for i, chunk in enumerate(retrieval_result.chunks):
                        context_parts.append(
                            f"[Source {i+1}: {chunk.title}]\n{chunk.text}"
                        )
                    context = "\n\n".join(context_parts)

                    # Add tool response to conversation
                    messages.append({
                        "role": "assistant",
                        "content": None,
                        "tool_calls": [
                            {
                                "id": tool_call.id,
                                "type": "function",
                                "function": {
                                    "name": function_name,
                                    "arguments": json.dumps(function_args)
                                }
                            }
                        ]
                    })

                    messages.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": context
                    })

                    # Second API call - agent generates answer with context
                    final_response = await self.client.chat.completions.create(
                        model=self.model,
                        messages=messages
                    )

                    answer = final_response.choices[0].message.content
                    sources = retrieval_result.sources

                    logger.info(
                        "Agent answered question",
                        extra={
                            "answer_length": len(answer),
                            "sources_count": len(sources)
                        }
                    )

                    return answer, sources

            # If no tool call, return direct response (shouldn't happen with our prompt)
            answer = assistant_message.content or "I don't have information about this in the textbook"
            sources = []

            logger.warning(
                "Agent did not use retrieve() tool",
                extra={"query": query}
            )

            return answer, sources

        except Exception as e:
            logger.error(
                "Gemini API error",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__
                }
            )
            raise GeminiAPIError(f"Failed to answer question: {str(e)}")
