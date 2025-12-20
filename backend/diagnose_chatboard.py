"""
CHATBOARD SILENCE DIAGNOSIS
Test each layer of the RAG stack independently
"""
import requests
import json

print("\n" + "="*70)
print("CHATBOARD SILENCE DIAGNOSIS")
print("="*70)

BASE_URL = "http://localhost:8001"

# Test 1: Is backend reachable?
print("\n[TEST 1] Backend Reachability")
try:
    r = requests.get(f"{BASE_URL}/", timeout=5)
    print(f"    [OK] Backend responds: {r.status_code}")
    print(f"    Response: {r.json()}")
except Exception as e:
    print(f"    [ERROR] Cannot reach backend: {e}")
    print("    FIX: Make sure backend is running (python main.py)")
    exit(1)

# Test 2: Health endpoint
print("\n[TEST 2] Health Check")
try:
    r = requests.get(f"{BASE_URL}/health", timeout=10)
    data = r.json()
    print(f"    [OK] Health: {json.dumps(data, indent=4)}")

    if data.get('qdrant') != 'connected':
        print("    [ERROR] Qdrant not connected")
        exit(1)
    if data.get('cohere') != 'connected':
        print("    [ERROR] Cohere not connected")
        exit(1)

except Exception as e:
    print(f"    [ERROR] Health check failed: {e}")
    exit(1)

# Test 3: Simple chat request
print("\n[TEST 3] Simple Chat Request")
try:
    payload = {
        "question": "What is ROS 2?",
        "selected_text": None,
        "conversation_history": []
    }

    print(f"    Sending: {json.dumps(payload, indent=4)}")

    r = requests.post(
        f"{BASE_URL}/chat",
        json=payload,
        timeout=30
    )

    print(f"    Status: {r.status_code}")

    if r.status_code == 500:
        print(f"    [ERROR] Server error: {r.text}")
        print("\n    Common causes:")
        print("    1. Cohere API quota exceeded")
        print("    2. Qdrant timeout")
        print("    3. Invalid chat model name")
        exit(1)

    if r.status_code == 200:
        data = r.json()
        print(f"\n    [OK] Response received!")
        print(f"    Sources: {len(data.get('sources', []))}")
        print(f"    Confidence: {data.get('confidence', 0)}")
        print(f"    Answer: {data.get('answer', '')[:200]}...")

        if not data.get('answer'):
            print("\n    [WARN] Answer is empty")
            print("    Check: LLM might be failing silently")

except Exception as e:
    print(f"    [ERROR] Chat request failed: {e}")
    exit(1)

# Test 4: CORS headers
print("\n[TEST 4] CORS Headers")
try:
    # Preflight request (OPTIONS)
    r = requests.options(f"{BASE_URL}/chat")
    print(f"    OPTIONS status: {r.status_code}")

    cors_headers = {
        'Access-Control-Allow-Origin': r.headers.get('Access-Control-Allow-Origin'),
        'Access-Control-Allow-Methods': r.headers.get('Access-Control-Allow-Methods'),
        'Access-Control-Allow-Headers': r.headers.get('Access-Control-Allow-Headers'),
    }

    print(f"    CORS Headers: {json.dumps(cors_headers, indent=4)}")

    if not cors_headers.get('Access-Control-Allow-Origin'):
        print("    [WARN] CORS may not be configured")
        print("    Frontend may be blocked")

except Exception as e:
    print(f"    [ERROR] CORS test failed: {e}")

# Test 5: Frontend accessibility
print("\n[TEST 5] Frontend Accessibility")
try:
    r = requests.get("http://localhost:3000/", timeout=5)
    if r.status_code == 200:
        print(f"    [OK] Frontend is running")
    else:
        print(f"    [WARN] Frontend returned: {r.status_code}")
except Exception as e:
    print(f"    [ERROR] Frontend not accessible: {e}")
    print("    FIX: Run 'npm start' in Humain-robotic-ai-book folder")

# Summary
print("\n" + "="*70)
print("DIAGNOSIS SUMMARY")
print("="*70)
print("""
If all tests passed, your backend is working perfectly.

NEXT STEPS TO DEBUG SILENT CHAT:

1. Open browser DevTools (F12)
2. Go to Network tab
3. Click chat button and send message
4. Look for /chat request
5. Check:
   - Request payload (should have question)
   - Response status (should be 200)
   - Response body (should have answer)
   - Console errors (check for JavaScript errors)

COMMON SILENT FAILURES:

A) ChatWidget not showing errors:
   - Check ChatWidget/index.js error handling
   - Add console.log in catch blocks

B) CORS blocking requests:
   - Check browser console for CORS errors
   - Verify main.py has localhost:3000 in allow_origins

C) Empty response displayed as silent:
   - Backend returns answer but UI doesn't render it
   - Check state updates in ChatWidget

D) Request never sent:
   - Check BACKEND_URL in ChatWidget
   - Should be http://localhost:8001

E) LLM failing silently:
   - Check if you're using Claude or Cohere
   - Verify API keys are correct
""")
print("="*70 + "\n")
