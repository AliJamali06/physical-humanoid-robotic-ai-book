# RAG Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics textbook chatbot.

## Setup

1. **Install dependencies:**
```bash
pip install -r requirements.txt
```

2. **Configure environment variables:**
```bash
cp .env.example .env
# Edit .env with your API keys
```

3. **Run document ingestion:**
```bash
python ingest_documents.py
```

4. **Start the server:**
```bash
python main.py
# Or with uvicorn:
uvicorn main:app --reload
```

## API Endpoints

- `POST /chat` - Main chat endpoint
  ```json
  {
    "question": "What is ROS 2?",
    "selected_text": "optional highlighted text",
    "conversation_history": []
  }
  ```

- `GET /health` - Health check

## Deployment

Deploy to Railway, Render, or Fly.io:

```bash
# Railway
railway up

# Render
# Connect your GitHub repo and deploy

# Fly.io
fly deploy
```
