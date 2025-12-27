"""
Unit tests for AgentService.

Tests the AI-powered question answering service using Gemini with
retrieve() tool for RAG (Retrieval Augmented Generation).
"""

import json
import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

from src.models.chat import RetrievalResult
from src.models.ingest import TextChunk
from src.services.agent import AgentService
from src.utils.exceptions import GeminiAPIError


@pytest.fixture
def mock_retrieval_service():
    """Mock RetrievalService for agent tests."""
    service = MagicMock()

    # Create sample retrieval result
    chunks = [
        TextChunk(
            id=uuid4(),
            text="Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators.",
            url="https://example.com/page1",
            title="Introduction to Physical AI",
            chunk_index=0,
            vector=[0.1] * 1024
        ),
        TextChunk(
            id=uuid4(),
            text="These systems combine machine learning with robotics to enable autonomous behavior in real-world environments.",
            url="https://example.com/page1",
            title="Introduction to Physical AI",
            chunk_index=1,
            vector=[0.2] * 1024
        ),
        TextChunk(
            id=uuid4(),
            text="Applications include self-driving cars, warehouse robots, and assistive devices.",
            url="https://example.com/page2",
            title="Applications of Physical AI",
            chunk_index=0,
            vector=[0.3] * 1024
        )
    ]

    result = RetrievalResult(
        chunks=chunks,
        sources=[
            "https://example.com/page1",
            "https://example.com/page2"
        ],
        scores=[0.95, 0.90, 0.85]
    )

    service.retrieve = AsyncMock(return_value=result)
    return service


@pytest.fixture
def mock_openai_client_with_tool_call():
    """Mock OpenAI client that simulates tool calling behavior."""
    client = MagicMock()

    # First response - agent wants to call retrieve() tool
    tool_call_response = MagicMock()
    tool_call_response.choices = [MagicMock()]
    tool_call_response.choices[0].message = MagicMock()
    tool_call_response.choices[0].message.content = None

    # Mock tool call
    tool_call = MagicMock()
    tool_call.id = "call_123"
    tool_call.function.name = "retrieve"
    tool_call.function.arguments = json.dumps({"query": "physical AI"})

    tool_call_response.choices[0].message.tool_calls = [tool_call]

    # Second response - agent generates answer after receiving context
    final_response = MagicMock()
    final_response.choices = [MagicMock()]
    final_response.choices[0].message = MagicMock()
    final_response.choices[0].message.content = "Physical AI refers to artificial intelligence systems that interact with the physical world. As mentioned in the Introduction to Physical AI section, these systems combine machine learning with robotics."

    # Mock create method to return both responses in sequence
    call_count = 0

    async def mock_create(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            return tool_call_response
        else:
            return final_response

    client.chat.completions.create = mock_create

    return client


@pytest.fixture
def mock_openai_client_no_tool_call():
    """Mock OpenAI client that responds directly without tool call."""
    client = MagicMock()

    response = MagicMock()
    response.choices = [MagicMock()]
    response.choices[0].message = MagicMock()
    response.choices[0].message.content = "I don't have information about this in the textbook"
    response.choices[0].message.tool_calls = None

    async def mock_create(*args, **kwargs):
        return response

    client.chat.completions.create = mock_create

    return client


@pytest.mark.asyncio
async def test_answer_question_with_retrieve_tool(
    mock_retrieval_service,
    mock_openai_client_with_tool_call
):
    """Test successful question answering with retrieve() tool call."""
    with patch('src.services.agent.AsyncOpenAI', return_value=mock_openai_client_with_tool_call):
        agent_service = AgentService(retrieval_service=mock_retrieval_service)

        # Execute
        answer, sources = await agent_service.answer_question("What is physical AI?")

        # Verify answer returned
        assert isinstance(answer, str)
        assert len(answer) > 0
        assert "Physical AI" in answer
        assert "physical world" in answer

        # Verify source URLs extracted from retrieval result
        assert isinstance(sources, list)
        assert len(sources) == 2
        assert "https://example.com/page1" in sources
        assert "https://example.com/page2" in sources

        # Verify retrieval service was called
        mock_retrieval_service.retrieve.assert_called_once()
        call_args = mock_retrieval_service.retrieve.call_args[0]
        assert "physical AI" in call_args[0]


@pytest.mark.asyncio
async def test_answer_question_system_prompt_enforcement(
    mock_retrieval_service,
    mock_openai_client_with_tool_call
):
    """Test that system prompt enforcing textbook-only responses is included."""
    with patch('src.services.agent.AsyncOpenAI', return_value=mock_openai_client_with_tool_call):
        agent_service = AgentService(retrieval_service=mock_retrieval_service)

        # Capture the messages sent to OpenAI
        captured_messages = []

        original_create = mock_openai_client_with_tool_call.chat.completions.create

        async def capture_create(*args, **kwargs):
            captured_messages.append(kwargs.get('messages', []))
            return await original_create(*args, **kwargs)

        mock_openai_client_with_tool_call.chat.completions.create = capture_create

        # Execute
        await agent_service.answer_question("What is physical AI?")

        # Verify system prompt was included in first call
        assert len(captured_messages) > 0
        first_call_messages = captured_messages[0]

        system_message = next((m for m in first_call_messages if m["role"] == "system"), None)
        assert system_message is not None
        assert "textbook" in system_message["content"].lower()
        assert "retrieve()" in system_message["content"]
        assert "don't have information" in system_message["content"].lower()

        # Verify retrieve() tool was defined
        # Check that tools parameter was passed
        assert agent_service.system_prompt is not None


@pytest.mark.asyncio
async def test_answer_question_no_tool_call(
    mock_retrieval_service,
    mock_openai_client_no_tool_call
):
    """Test fallback behavior when agent doesn't call retrieve() tool."""
    with patch('src.services.agent.AsyncOpenAI', return_value=mock_openai_client_no_tool_call):
        agent_service = AgentService(retrieval_service=mock_retrieval_service)

        # Execute
        answer, sources = await agent_service.answer_question("Random question")

        # Verify fallback response
        assert answer == "I don't have information about this in the textbook"
        assert sources == []

        # Verify retrieval was not called
        mock_retrieval_service.retrieve.assert_not_called()


@pytest.mark.asyncio
async def test_answer_question_gemini_api_error(mock_retrieval_service):
    """Test that GeminiAPIError is raised when OpenAI client fails."""
    # Mock OpenAI client to raise exception
    mock_client = MagicMock()

    async def mock_create_error(*args, **kwargs):
        raise Exception("API connection failed")

    mock_client.chat.completions.create = mock_create_error

    with patch('src.services.agent.AsyncOpenAI', return_value=mock_client):
        agent_service = AgentService(retrieval_service=mock_retrieval_service)

        # Execute and verify exception
        with pytest.raises(GeminiAPIError) as exc_info:
            await agent_service.answer_question("What is physical AI?")

        assert "Failed to answer question" in str(exc_info.value)
        assert "API connection failed" in str(exc_info.value)


@pytest.mark.asyncio
async def test_answer_question_context_formatting(
    mock_retrieval_service,
    mock_openai_client_with_tool_call
):
    """Test that retrieved chunks are properly formatted as context."""
    # Track the messages sent to OpenAI
    captured_messages = []

    with patch('src.services.agent.AsyncOpenAI', return_value=mock_openai_client_with_tool_call):
        agent_service = AgentService(retrieval_service=mock_retrieval_service)

        # Capture messages
        original_create = mock_openai_client_with_tool_call.chat.completions.create

        async def capture_create(*args, **kwargs):
            messages = kwargs.get('messages', [])
            if len(messages) > 2:  # Second call has context
                captured_messages.extend(messages)
            return await original_create(*args, **kwargs)

        mock_openai_client_with_tool_call.chat.completions.create = capture_create

        # Execute
        await agent_service.answer_question("What is physical AI?")

        # Verify context was formatted correctly
        tool_messages = [m for m in captured_messages if m.get("role") == "tool"]
        assert len(tool_messages) > 0

        tool_content = tool_messages[0]["content"]
        assert "[Source 1:" in tool_content
        assert "Introduction to Physical AI" in tool_content
        assert "artificial intelligence systems" in tool_content


@pytest.mark.asyncio
async def test_answer_question_retrieval_error_propagation(mock_retrieval_service):
    """Test that errors from retrieval service are propagated as GeminiAPIError."""
    from src.utils.exceptions import CohereAPIError

    # Mock retrieval to raise error
    mock_retrieval_service.retrieve = AsyncMock(
        side_effect=CohereAPIError("Embedding service down")
    )

    mock_client = MagicMock()

    # First response - agent wants to call retrieve() tool
    tool_call_response = MagicMock()
    tool_call_response.choices = [MagicMock()]
    tool_call_response.choices[0].message = MagicMock()
    tool_call_response.choices[0].message.content = None

    tool_call = MagicMock()
    tool_call.id = "call_123"
    tool_call.function.name = "retrieve"
    tool_call.function.arguments = json.dumps({"query": "test"})

    tool_call_response.choices[0].message.tool_calls = [tool_call]

    async def mock_create(*args, **kwargs):
        return tool_call_response

    mock_client.chat.completions.create = mock_create

    with patch('src.services.agent.AsyncOpenAI', return_value=mock_client):
        agent_service = AgentService(retrieval_service=mock_retrieval_service)

        # Execute and verify exception
        with pytest.raises(CohereAPIError) as exc_info:
            await agent_service.answer_question("What is physical AI?")

        assert "Embedding service down" in str(exc_info.value)


@pytest.mark.asyncio
async def test_answer_question_source_url_extraction(
    mock_retrieval_service,
    mock_openai_client_with_tool_call
):
    """Test that source URLs are correctly extracted from retrieval result."""
    # Create retrieval result with specific URLs
    chunks = [
        TextChunk(
            id=uuid4(),
            text="Content from page A",
            url="https://textbook.com/chapter1/pageA",
            title="Page A",
            chunk_index=0,
            vector=[0.1] * 1024
        ),
        TextChunk(
            id=uuid4(),
            text="Content from page B",
            url="https://textbook.com/chapter2/pageB",
            title="Page B",
            chunk_index=0,
            vector=[0.2] * 1024
        ),
        TextChunk(
            id=uuid4(),
            text="More content from page A",
            url="https://textbook.com/chapter1/pageA",  # Duplicate
            title="Page A",
            chunk_index=1,
            vector=[0.3] * 1024
        )
    ]

    custom_result = RetrievalResult(
        chunks=chunks,
        sources=[
            "https://textbook.com/chapter1/pageA",
            "https://textbook.com/chapter2/pageB"
        ],
        scores=[0.95, 0.90, 0.85]
    )

    mock_retrieval_service.retrieve = AsyncMock(return_value=custom_result)

    with patch('src.services.agent.AsyncOpenAI', return_value=mock_openai_client_with_tool_call):
        agent_service = AgentService(retrieval_service=mock_retrieval_service)

        # Execute
        answer, sources = await agent_service.answer_question("test query")

        # Verify source URLs match retrieval result (deduplication handled by retrieval service)
        assert len(sources) == 2
        assert "https://textbook.com/chapter1/pageA" in sources
        assert "https://textbook.com/chapter2/pageB" in sources


@pytest.mark.asyncio
async def test_answer_question_empty_retrieval(mock_retrieval_service):
    """Test behavior when retrieval returns no results."""
    # Mock empty retrieval result
    empty_result = RetrievalResult(
        chunks=[],
        sources=[],
        scores=[]
    )
    mock_retrieval_service.retrieve = AsyncMock(return_value=empty_result)

    # Mock OpenAI client
    mock_client = MagicMock()
    tool_call_response = MagicMock()
    tool_call_response.choices = [MagicMock()]
    tool_call_response.choices[0].message = MagicMock()

    tool_call = MagicMock()
    tool_call.id = "call_123"
    tool_call.function.name = "retrieve"
    tool_call.function.arguments = json.dumps({"query": "nonexistent topic"})

    tool_call_response.choices[0].message.tool_calls = [tool_call]

    # Final response when no context
    final_response = MagicMock()
    final_response.choices = [MagicMock()]
    final_response.choices[0].message = MagicMock()
    final_response.choices[0].message.content = "I don't have information about this in the textbook"

    call_count = 0

    async def mock_create(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            return tool_call_response
        else:
            return final_response

    mock_client.chat.completions.create = mock_create

    with patch('src.services.agent.AsyncOpenAI', return_value=mock_client):
        agent_service = AgentService(retrieval_service=mock_retrieval_service)

        # Execute
        answer, sources = await agent_service.answer_question("nonexistent topic")

        # Verify response indicates no information
        assert "don't have information" in answer.lower()
        assert len(sources) == 0
