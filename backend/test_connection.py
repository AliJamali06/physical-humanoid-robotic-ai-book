"""Test backend RAG chatbot end-to-end"""
import requests
import json

print("\n" + "="*60)
print("Testing RAG Chatbot Connection")
print("="*60)

# Test 1: Health check
print("\n[1] Testing backend health...")
try:
    r = requests.get("http://localhost:8001/health", timeout=10)
    if r.status_code == 200:
        data = r.json()
        print(f"    [OK] Backend is online")
        print(f"    [OK] Qdrant: {data.get('qdrant')}")
        print(f"    [OK] Cohere: {data.get('cohere')}")
        print(f"    [OK] Collection: {data.get('collection')}")
    else:
        print(f"    [ERROR] Health check failed: {r.status_code}")
except Exception as e:
    print(f"    [ERROR] Connection error: {e}")
    exit(1)

# Test 2: Chat endpoint
print("\n[2] Testing chat endpoint...")
test_questions = [
    "What is ROS 2?",
    "Explain Isaac Sim",
    "How does Gazebo work?"
]

for i, question in enumerate(test_questions, 1):
    print(f"\n    Question {i}: {question}")
    try:
        r = requests.post(
            "http://localhost:8001/chat",
            json={"question": question},
            timeout=30
        )

        if r.status_code == 200:
            data = r.json()
            sources = data.get('sources', [])
            confidence = data.get('confidence', 0)
            answer = data.get('answer', '')

            print(f"    [OK] Status: Success")
            print(f"    [OK] Sources found: {len(sources)}")
            print(f"    [OK] Confidence: {confidence:.2f}")
            print(f"    [OK] Answer preview: {answer[:100]}...")

            if sources:
                print(f"    [OK] Sample source: {sources[0].get('module', 'Unknown')}")
        else:
            print(f"    [ERROR] Error: {r.status_code}")
            print(f"       {r.text[:200]}")

    except Exception as e:
        print(f"    [ERROR] Request failed: {e}")

print("\n" + "="*60)
print("Test Complete!")
print("="*60 + "\n")
