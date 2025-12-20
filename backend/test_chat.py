import requests
import json

try:
    print("Testing chat endpoint...")
    r = requests.post('http://localhost:8001/chat',
                     json={'question': 'What is Isaac Sim?'},
                     timeout=30)

    print(f"Status code: {r.status_code}")

    if r.status_code == 200:
        data = r.json()
        print(f"Success: {'answer' in data}")
        print(f"Sources found: {len(data.get('sources', []))}")
        print(f"Confidence: {data.get('confidence', 0)}")
        print(f"\nAnswer preview: {data.get('answer', '')[:200]}...")
    else:
        print(f"Error: {r.text}")

except Exception as e:
    print(f"Exception occurred: {type(e).__name__}: {e}")
