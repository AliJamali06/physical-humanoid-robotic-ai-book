from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import os
from typing import Optional, List
from dotenv import load_dotenv
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

app = FastAPI(title="Robotics AI Chatbot API")

# CORS configuration - Update with your GitHub Pages URL
# CORS configuration - Temporarily allow ALL origins for debugging
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow ALL origins (TEMPORARY - for debugging)
    allow_credentials=False,  # Must be False when allow_origins=["*"]
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))  # For embeddings AND chat
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = "physical_ai_docs"  # Updated to match new upload
EMBEDDING_MODEL = "embed-english-v3.0"  # Cohere embedding model (1024 dimensions)
# Valid Cohere chat models (2025):
# - command-r7b-12-2024 (lightweight, free tier, 7B params)
# - command-r-08-2024 (production RAG, medium)
# - command-a-03-2025 (flagship, best for RAG, 111B params)
CHAT_MODEL = "command-r7b-12-2024"  # Using lightweight model for free tier compatibility

logger.info(f"🚀 Initializing chatbot with model: {CHAT_MODEL}")

def ensure_collection_exists():
    """Ensure Qdrant collection exists, create if missing"""
    try:
        collections = qdrant_client.get_collections().collections
        collection_names = [c.name for c in collections]

        if COLLECTION_NAME not in collection_names:
            print(f"⚠️ Collection '{COLLECTION_NAME}' not found!")
            print(f"Available collections: {collection_names}")
            print("Run: cd backend && python ingest_documents.py")
            return False
        return True
    except Exception as e:
        print(f"Error checking collection: {e}")
        return False

# Check collection on startup
ensure_collection_exists()

class ChatQuery(BaseModel):
    question: str
    selected_text: Optional[str] = None
    conversation_history: Optional[List[dict]] = []

class ChatResponse(BaseModel):
    answer: str
    sources: List[dict]
    confidence: float

@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "online",
        "service": "Robotics AI Chatbot",
        "version": "1.0.0"
    }

@app.post("/chat", response_model=ChatResponse)
async def chat(query: ChatQuery):
    """
    Main RAG chat endpoint
    - Retrieves relevant context from Qdrant using Cohere embeddings
    - Generates answer using Cohere chat model (optimized for RAG)
    - Supports text selection queries and conversation history
    """
    try:
        logger.info(f"📩 Received chat request: '{query.question[:100]}...'")

        # Generate embedding for the question using Cohere
        logger.info("🔄 Generating embedding with Cohere...")
        embedding_response = cohere_client.embed(
            texts=[query.question],
            model=EMBEDDING_MODEL,
            input_type="search_query"
        )
        query_embedding = embedding_response.embeddings[0]
        logger.info(f"✅ Embedding generated (dim: {len(query_embedding)})")

        # Search Qdrant for relevant context
        logger.info(f"🔍 Searching Qdrant collection: {COLLECTION_NAME}")
        search_results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_embedding,
            limit=5,
            score_threshold=0.3  # Lower threshold for better retrieval
        ).points
        logger.info(f"✅ Found {len(search_results)} results from Qdrant")

        # ⚠️ CRITICAL: Check if Qdrant returned any results
        if not search_results or len(search_results) == 0:
            logger.warning("⚠️ No search results found in Qdrant!")
            return ChatResponse(
                answer="⚠️ **No data found in knowledge base!**\n\nThis means:\n1. The Qdrant collection is empty\n2. Data was never ingested\n\n**Fix**: Run `cd backend && python ingest_documents.py` to upload your textbook content.",
                sources=[],
                confidence=0.0
            )

        # Build context from search results
        context_chunks = []
        sources = []

        for hit in search_results:
            context_chunks.append(hit.payload["text"])
            sources.append({
                "text": hit.payload["text"][:200] + "...",
                "module": hit.payload.get("module", "Unknown"),
                "chapter": hit.payload.get("chapter", "Unknown"),
                "score": round(hit.score, 3)
            })

        context = "\n\n".join(context_chunks)

        # If user selected text, prioritize it
        if query.selected_text:
            context = f"**Selected Text from Page:**\n{query.selected_text}\n\n**Related Context from Book:**\n{context}"

        # Build system message (preamble for Cohere)
        system_message = """You are an expert AI assistant for a Physical AI & Humanoid Robotics textbook.

Your role:
- Answer questions based ONLY on the provided context from the book
- Explain ROS 2 concepts, Isaac Sim, Gazebo, Unity, and Vision-Language-Action pipelines
- Provide code examples when relevant
- If the context doesn't contain the answer, say "I don't have information about this in the textbook"
- Be concise but thorough
- Use technical terminology accurately

Teaching approach:
- Break down complex concepts into simple explanations
- Reference specific modules and chapters when possible
- Suggest related topics for further learning"""

        # Build user message with context
        user_message = f"""**Context from Textbook:**
{context}

**Student Question:**
{query.question}

Please provide a clear, educational answer based on the textbook context above."""

        # Build chat history for Cohere (convert from conversation history)
        chat_history = []
        if query.conversation_history:
            # Add last 4 messages from conversation history
            for msg in query.conversation_history[-4:]:
                role = msg.get("role", "user").lower()
                if role == "user":
                    chat_history.append({"role": "USER", "message": msg.get("content", "")})
                elif role == "assistant":
                    chat_history.append({"role": "CHATBOT", "message": msg.get("content", "")})

        logger.info(f"💬 Chat history length: {len(chat_history)}")

        # Generate answer using Cohere
        logger.info(f"🤖 Calling Cohere chat API (model: {CHAT_MODEL})...")
        try:
            response = cohere_client.chat(
                model=CHAT_MODEL,
                message=user_message,
                preamble=system_message,
                chat_history=chat_history if chat_history else None,
                temperature=0.7,
                max_tokens=800
            )
            logger.info(f"✅ Cohere response received (length: {len(response.text)} chars)")
            answer = response.text

        except Exception as cohere_error:
            logger.error(f"❌ Cohere chat API error: {str(cohere_error)}")
            logger.error(f"Model: {CHAT_MODEL}, Message length: {len(user_message)}")
            raise HTTPException(
                status_code=500,
                detail=f"Cohere API error: {str(cohere_error)}"
            )

        # Calculate confidence based on retrieval scores
        avg_score = sum(hit.score for hit in search_results) / len(search_results) if search_results else 0
        confidence = round(avg_score, 2)

        logger.info(f"✅ Chat response ready (confidence: {confidence})")

        return ChatResponse(
            answer=answer,
            sources=sources,
            confidence=confidence
        )

    except HTTPException:
        raise  # Re-raise HTTP exceptions as-is
    except Exception as e:
        logger.error(f"❌ Unexpected error in chat endpoint: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Chat error: {str(e)}")

@app.get("/health")
async def health_check():
    """Check if all services are connected"""
    try:
        # Test Qdrant connection
        collections = qdrant_client.get_collections()
        qdrant_status = "connected"
        collection_exists = COLLECTION_NAME in [c.name for c in collections.collections]
    except Exception as e:
        qdrant_status = f"error: {str(e)}"
        collection_exists = False

    try:
        # Test Cohere embedding connection
        test_response = cohere_client.embed(
            texts=["test"],
            model=EMBEDDING_MODEL,
            input_type="search_query"
        )
        cohere_embed_status = "connected" if test_response.embeddings else "error"
    except Exception as e:
        cohere_embed_status = f"error: {str(e)}"

    try:
        # Test Cohere chat connection with a minimal request
        logger.info(f"Testing Cohere chat with model: {CHAT_MODEL}")
        test_response = cohere_client.chat(
            model=CHAT_MODEL,
            message="Hi",
            max_tokens=10
        )
        cohere_chat_status = "connected" if test_response.text else "error"
        logger.info(f"✅ Cohere chat test successful")
    except Exception as e:
        logger.error(f"❌ Cohere chat test failed: {str(e)}")
        cohere_chat_status = f"error: {str(e)[:100]}"  # Truncate long errors

    health_status = {
        "qdrant": qdrant_status,
        "collection_exists": collection_exists,
        "cohere_embeddings": cohere_embed_status,
        "cohere_chat": cohere_chat_status,
        "collection": COLLECTION_NAME,
        "embedding_model": EMBEDDING_MODEL,
        "chat_model": CHAT_MODEL
    }

    logger.info(f"Health check: Qdrant={qdrant_status}, Embeddings={cohere_embed_status}, Chat={cohere_chat_status[:50]}")

    return health_status

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
