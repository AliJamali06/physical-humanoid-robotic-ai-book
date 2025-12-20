"""Test Qdrant search with the uploaded data"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import cohere

load_dotenv()

# Initialize clients
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

# Collection info
collection_info = qdrant_client.get_collection("physical_ai_docs")
print(f"Collection: physical_ai_docs")
print(f"Total vectors: {collection_info.points_count}")
print(f"Status: {collection_info.status}")

# Sample some records
print(f"\n--- Sample Records ---")
scroll_result = qdrant_client.scroll("physical_ai_docs", limit=5)
for i, point in enumerate(scroll_result[0], 1):
    print(f"\n{i}. {point.payload.get('file_name', 'Unknown')}")
    print(f"   Module: {point.payload.get('module', 'Unknown')}")
    print(f"   Chapter: {point.payload.get('chapter', 'Unknown')}")
    print(f"   Text preview: {point.payload.get('text', '')[:100]}...")

# Test search
print(f"\n--- Testing Search ---")
query = "What is ROS 2?"
print(f"Query: {query}")

# Generate query embedding
response = cohere_client.embed(
    texts=[query],
    model="embed-english-v3.0",
    input_type="search_query"
)
query_embedding = response.embeddings[0]

# Search
search_results = qdrant_client.query_points(
    collection_name="physical_ai_docs",
    query=query_embedding,
    limit=3,
    score_threshold=0.3
).points

print(f"\nFound {len(search_results)} results:")
for i, result in enumerate(search_results, 1):
    print(f"\n{i}. Score: {result.score:.3f}")
    print(f"   File: {result.payload.get('file_name', 'Unknown')}")
    print(f"   Chapter: {result.payload.get('chapter', 'Unknown')}")
    print(f"   Text: {result.payload.get('text', '')[:150]}...")
