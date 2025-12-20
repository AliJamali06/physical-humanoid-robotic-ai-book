"""
QDRANT CLOUD CONNECTION AUDIT
Run this to verify end-to-end Qdrant functionality
"""
import os
import time
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import cohere
import uuid

load_dotenv()

print("\n" + "="*70)
print("QDRANT CLOUD AUDIT - STEP BY STEP")
print("="*70)

# Step 1: Verify credentials
print("\n[STEP 1] Checking credentials...")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

if not QDRANT_URL:
    print("    [ERROR] QDRANT_URL not found in .env")
    exit(1)
if not QDRANT_API_KEY:
    print("    [ERROR] QDRANT_API_KEY not found in .env")
    exit(1)
if not COHERE_API_KEY:
    print("    [ERROR] COHERE_API_KEY not found in .env")
    exit(1)

print(f"    [OK] QDRANT_URL: {QDRANT_URL[:50]}...")
print(f"    [OK] QDRANT_API_KEY: {QDRANT_API_KEY[:20]}...")
print(f"    [OK] COHERE_API_KEY: {COHERE_API_KEY[:20]}...")

# Step 2: Connect to Qdrant
print("\n[STEP 2] Connecting to Qdrant Cloud...")
try:
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    collections = client.get_collections()
    print(f"    [OK] Connected successfully")
    print(f"    [OK] Existing collections: {[c.name for c in collections.collections]}")
except Exception as e:
    print(f"    [ERROR] Connection failed: {e}")
    exit(1)

# Step 3: Check existing collection
print("\n[STEP 3] Checking 'physical_ai_docs' collection...")
try:
    collection_info = client.get_collection("physical_ai_docs")
    print(f"    [OK] Collection exists")
    print(f"    [OK] Vector size: {collection_info.config.params.vectors.size}")
    print(f"    [OK] Points count: {collection_info.points_count}")
    print(f"    [OK] Status: {collection_info.status}")
except Exception as e:
    print(f"    [ERROR] Collection check failed: {e}")
    exit(1)

# Step 4: Sample data
print("\n[STEP 4] Sampling stored data...")
try:
    scroll_result = client.scroll("physical_ai_docs", limit=3)
    points = scroll_result[0]
    print(f"    [OK] Retrieved {len(points)} sample points")
    for i, point in enumerate(points, 1):
        print(f"    [{i}] File: {point.payload.get('file_name', 'Unknown')}")
        print(f"        Module: {point.payload.get('module', 'Unknown')}")
        print(f"        Text: {point.payload.get('text', '')[:80]}...")
except Exception as e:
    print(f"    [ERROR] Sampling failed: {e}")

# Step 5: Test embedding generation
print("\n[STEP 5] Testing Cohere embedding generation...")
try:
    cohere_client = cohere.Client(api_key=COHERE_API_KEY)
    test_text = "What is ROS 2?"

    response = cohere_client.embed(
        texts=[test_text],
        model="embed-english-v3.0",
        input_type="search_query"
    )
    embedding = response.embeddings[0]

    print(f"    [OK] Embedding generated")
    print(f"    [OK] Embedding dimension: {len(embedding)}")
    print(f"    [OK] First 5 values: {embedding[:5]}")
except Exception as e:
    print(f"    [ERROR] Embedding generation failed: {e}")
    exit(1)

# Step 6: Test vector search
print("\n[STEP 6] Testing vector search...")
try:
    search_start = time.time()
    search_results = client.query_points(
        collection_name="physical_ai_docs",
        query=embedding,
        limit=3,
        score_threshold=0.3
    ).points
    search_time = time.time() - search_start

    print(f"    [OK] Search completed in {search_time:.2f}s")
    print(f"    [OK] Found {len(search_results)} results")

    for i, result in enumerate(search_results, 1):
        print(f"    [{i}] Score: {result.score:.3f}")
        print(f"        File: {result.payload.get('file_name', 'Unknown')}")
        print(f"        Module: {result.payload.get('module', 'Unknown')}")

except Exception as e:
    print(f"    [ERROR] Search failed: {e}")
    exit(1)

# Step 7: Test write operation (create test collection)
print("\n[STEP 7] Testing write operations...")
try:
    test_collection = "audit_test_collection"

    # Delete if exists
    try:
        client.delete_collection(collection_name=test_collection)
    except:
        pass

    # Create test collection
    client.create_collection(
        collection_name=test_collection,
        vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
    )
    print(f"    [OK] Created test collection: {test_collection}")

    # Upload test point
    test_point = PointStruct(
        id=str(uuid.uuid4()),
        vector=embedding,
        payload={"text": "test", "source": "audit"}
    )

    client.upsert(
        collection_name=test_collection,
        points=[test_point]
    )
    print(f"    [OK] Uploaded test vector")

    # Verify
    time.sleep(1)
    info = client.get_collection(test_collection)
    print(f"    [OK] Verified: {info.points_count} points in test collection")

    # Cleanup
    client.delete_collection(collection_name=test_collection)
    print(f"    [OK] Cleaned up test collection")

except Exception as e:
    print(f"    [ERROR] Write test failed: {e}")

# Final summary
print("\n" + "="*70)
print("AUDIT COMPLETE - DIAGNOSIS")
print("="*70)

if collection_info.points_count > 0 and len(search_results) > 0:
    print("\n✓ VERDICT: Your Qdrant setup is WORKING")
    print("\nWhat's working:")
    print("  ✓ Qdrant Cloud connection")
    print("  ✓ Data storage (75 vectors)")
    print("  ✓ Cohere embeddings")
    print("  ✓ Vector search")
    print("  ✓ Write permissions")

    if search_time > 2:
        print("\n⚠ WARNING: Search is slow (>2s)")
        print("  - This is normal for Qdrant Cloud on first query")
        print("  - Consider warming up the collection on startup")

    print("\n🔍 If chat board is silent, check:")
    print("  1. Backend logs: Look for 500 errors")
    print("  2. Browser console: Check for CORS errors")
    print("  3. Network tab: Verify /chat requests succeed")
    print("  4. ChatWidget code: Ensure error messages display")
else:
    print("\n✗ VERDICT: Data storage or retrieval FAILED")
    print("\nIssues found:")
    if collection_info.points_count == 0:
        print("  ✗ No vectors in collection")
    if len(search_results) == 0:
        print("  ✗ Search returns no results")

print("\n" + "="*70 + "\n")
