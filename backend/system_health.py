"""
COMPLETE SYSTEM HEALTH CHECK
Run this anytime to verify your RAG chatbot is working
"""
import os
import sys
import time
import requests
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import cohere

load_dotenv()

def check(test_name, passed):
    """Print test result"""
    status = "[OK]" if passed else "[FAIL]"
    print(f"    {status} {test_name}")
    return passed

def main():
    print("\n" + "="*70)
    print("COMPLETE SYSTEM HEALTH CHECK")
    print("="*70)

    all_passed = True

    # SECTION 1: Environment Variables
    print("\n[1] ENVIRONMENT VARIABLES")
    all_passed &= check("QDRANT_URL exists", bool(os.getenv("QDRANT_URL")))
    all_passed &= check("QDRANT_API_KEY exists", bool(os.getenv("QDRANT_API_KEY")))
    all_passed &= check("COHERE_API_KEY exists", bool(os.getenv("COHERE_API_KEY")))

    # SECTION 2: Qdrant Cloud
    print("\n[2] QDRANT CLOUD")
    try:
        client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        all_passed &= check("Can connect to Qdrant", True)

        collection = client.get_collection("physical_ai_docs")
        all_passed &= check("Collection 'physical_ai_docs' exists", True)
        all_passed &= check(f"Has {collection.points_count} vectors", collection.points_count > 0)
        all_passed &= check("Collection status GREEN", collection.status == "green")
    except Exception as e:
        all_passed &= check(f"Qdrant connection failed: {e}", False)

    # SECTION 3: Cohere API
    print("\n[3] COHERE API")
    try:
        cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))
        response = cohere_client.embed(
            texts=["test"],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        all_passed &= check("Can generate embeddings", len(response.embeddings) > 0)
        all_passed &= check("Embedding dimension is 1024", len(response.embeddings[0]) == 1024)
    except Exception as e:
        all_passed &= check(f"Cohere API failed: {e}", False)

    # SECTION 4: Backend Server
    print("\n[4] BACKEND SERVER")
    try:
        r = requests.get("http://localhost:8001/", timeout=5)
        all_passed &= check("Backend is running", r.status_code == 200)
    except:
        all_passed &= check("Backend is running", False)
        print("        Start with: cd backend && python main.py")

    try:
        r = requests.get("http://localhost:8001/health", timeout=5)
        if r.status_code == 200:
            data = r.json()
            all_passed &= check("Qdrant connected via backend", data.get('qdrant') == 'connected')
            all_passed &= check("Cohere connected via backend", data.get('cohere') == 'connected')
            all_passed &= check(f"Using collection: {data.get('collection')}", data.get('collection') == 'physical_ai_docs')
    except:
        all_passed &= check("Health endpoint accessible", False)

    # SECTION 5: Chat Endpoint
    print("\n[5] CHAT ENDPOINT")
    try:
        r = requests.post(
            "http://localhost:8001/chat",
            json={"question": "What is ROS 2?"},
            timeout=30
        )

        if r.status_code == 200:
            data = r.json()
            all_passed &= check("Chat endpoint responds", True)
            all_passed &= check("Returns answer", bool(data.get('answer')))
            all_passed &= check(f"Returns {len(data.get('sources', []))} sources", len(data.get('sources', [])) > 0)
            all_passed &= check(f"Confidence: {data.get('confidence', 0):.2f}", data.get('confidence', 0) > 0)
        else:
            all_passed &= check(f"Chat returned status {r.status_code}", False)

    except Exception as e:
        all_passed &= check(f"Chat endpoint failed: {e}", False)

    # SECTION 6: Frontend
    print("\n[6] FRONTEND")
    try:
        r = requests.get("http://localhost:3000/", timeout=5)
        all_passed &= check("Frontend is running", r.status_code == 200)
    except:
        all_passed &= check("Frontend is running", False)
        print("        Start with: cd Humain-robotic-ai-book && npm start")

    # SECTION 7: CORS
    print("\n[7] CORS CONFIGURATION")
    try:
        r = requests.post(
            "http://localhost:8001/chat",
            json={"question": "test"},
            headers={"Origin": "http://localhost:3000"},
            timeout=10
        )
        cors_header = r.headers.get('Access-Control-Allow-Origin')
        all_passed &= check(f"CORS allows localhost:3000", cors_header is not None)
    except:
        all_passed &= check("CORS check failed", False)

    # FINAL VERDICT
    print("\n" + "="*70)
    if all_passed:
        print("VERDICT: ALL SYSTEMS OPERATIONAL")
        print("="*70)
        print("""
Your RAG chatbot is fully functional!

If chat board appears silent in browser:
1. Press F12 to open DevTools
2. Check Console tab for JavaScript errors
3. Check Network tab for /chat requests
4. Verify responses have 'answer' field
5. See BROWSER_DEBUG_GUIDE.md for details
        """)
    else:
        print("VERDICT: ISSUES DETECTED")
        print("="*70)
        print("""
One or more components failed checks.

Common fixes:
- Backend not running? → cd backend && python main.py
- Frontend not running? → cd Humain-robotic-ai-book && npm start
- Qdrant errors? → Check QDRANT_URL and API key in .env
- Cohere errors? → Check COHERE_API_KEY in .env

For detailed diagnosis:
- python audit_qdrant.py
- python diagnose_chatboard.py
        """)

    print("="*70 + "\n")
    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
