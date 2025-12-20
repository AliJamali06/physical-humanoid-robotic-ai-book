"""
Physical AI Textbook Ingestion Pipeline
Uploads markdown content to Qdrant Cloud with Cohere embeddings
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Tuple
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import uuid
from dotenv import load_dotenv
from tqdm import tqdm
import time

# Load environment variables
load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

COLLECTION_NAME = "physical_ai_docs"
EMBEDDING_MODEL = "embed-english-v3.0"
EMBEDDING_DIMENSION = 1024
CHUNK_SIZE = 800  # words
CHUNK_OVERLAP = 100  # words

# Initialize clients
print("\n[*] Initializing API clients...")
cohere_client = cohere.Client(api_key=COHERE_API_KEY)
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)


def remove_frontmatter(content: str) -> str:
    """Remove YAML frontmatter from markdown"""
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)
    return content.strip()


def chunk_by_words(text: str, chunk_size: int = 800, overlap: int = 100) -> List[str]:
    """Split text into overlapping chunks by word count"""
    if not text or len(text.strip()) == 0:
        return []

    # Split into words
    words = text.split()

    if len(words) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(words):
        # Get chunk_size words
        end = min(start + chunk_size, len(words))
        chunk_words = words[start:end]
        chunk_text = ' '.join(chunk_words)

        if chunk_text.strip():
            chunks.append(chunk_text)

        # Move forward by (chunk_size - overlap) words
        start += (chunk_size - overlap)

        # Prevent infinite loop
        if start == 0:
            start = chunk_size

    return chunks


def extract_chapter_info(file_path: Path) -> Dict[str, str]:
    """Extract module and chapter info from file path"""
    parts = file_path.parts

    module = "Unknown"
    chapter = file_path.stem

    # Look for module-X pattern
    for i, part in enumerate(parts):
        if part.startswith('module-'):
            module = part.replace('module-', 'Module ').replace('-', ' ').title()
            if i + 1 < len(parts):
                chapter = parts[i + 1].replace('.md', '').replace('-', ' ').title()
            break

    return {
        'module': module,
        'chapter': chapter,
        'file_name': file_path.name
    }


def process_markdown_file(file_path: Path) -> List[Dict]:
    """Process a single markdown file into chunks"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Remove frontmatter
        clean_content = remove_frontmatter(content)

        if not clean_content:
            return []

        # Get chapter info
        chapter_info = extract_chapter_info(file_path)

        # Chunk by words
        chunks = chunk_by_words(clean_content, CHUNK_SIZE, CHUNK_OVERLAP)

        # Create document objects
        documents = []
        for i, chunk in enumerate(chunks):
            documents.append({
                'text': chunk,
                'file_path': str(file_path),
                'module': chapter_info['module'],
                'chapter': chapter_info['chapter'],
                'file_name': chapter_info['file_name'],
                'chunk_index': i,
                'total_chunks': len(chunks)
            })

        return documents

    except Exception as e:
        print(f"\n[ERROR] Failed to process {file_path.name}: {e}")
        return []


def create_collection():
    """Create or recreate Qdrant collection"""
    try:
        # Delete if exists
        try:
            qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
            print(f"[*] Deleted existing collection: {COLLECTION_NAME}")
        except:
            pass

        # Create new collection
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBEDDING_DIMENSION,
                distance=Distance.COSINE
            )
        )
        print(f"[OK] Created collection: {COLLECTION_NAME}")
        print(f"    Vector size: {EMBEDDING_DIMENSION}")
        print(f"    Distance: COSINE")

    except Exception as e:
        print(f"[ERROR] Failed to create collection: {e}")
        raise


def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """Generate embeddings for a batch of texts using Cohere"""
    try:
        response = cohere_client.embed(
            texts=texts,
            model=EMBEDDING_MODEL,
            input_type="search_document"
        )
        return response.embeddings
    except Exception as e:
        print(f"\n[ERROR] Embedding generation failed: {e}")
        raise


def upload_to_qdrant(documents: List[Dict]):
    """Upload documents to Qdrant with embeddings"""
    if not documents:
        print("[WARN] No documents to upload")
        return

    print(f"\n[*] Uploading {len(documents)} chunks to Qdrant Cloud...")

    # Process in batches for embedding generation (Cohere limit: 96)
    embedding_batch_size = 96
    upload_batch_size = 100

    total_uploaded = 0

    # Generate embeddings in batches
    all_points = []

    for i in tqdm(range(0, len(documents), embedding_batch_size),
                  desc="Generating embeddings", unit="batch"):
        batch = documents[i:i + embedding_batch_size]
        texts = [doc['text'] for doc in batch]

        try:
            # Generate embeddings
            embeddings = generate_embeddings_batch(texts)

            # Create points
            for doc, embedding in zip(batch, embeddings):
                point = PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload={
                        'text': doc['text'],
                        'file_path': doc['file_path'],
                        'module': doc['module'],
                        'chapter': doc['chapter'],
                        'file_name': doc['file_name'],
                        'chunk_index': doc['chunk_index'],
                        'total_chunks': doc['total_chunks']
                    }
                )
                all_points.append(point)

        except Exception as e:
            print(f"\n[ERROR] Batch {i//embedding_batch_size + 1} failed: {e}")
            continue

    # Upload to Qdrant in batches
    print(f"\n[*] Uploading {len(all_points)} vectors to Qdrant...")

    for i in tqdm(range(0, len(all_points), upload_batch_size),
                  desc="Uploading to Qdrant", unit="batch"):
        batch_points = all_points[i:i + upload_batch_size]

        try:
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                points=batch_points
            )
            total_uploaded += len(batch_points)
        except Exception as e:
            print(f"\n[ERROR] Upload batch {i//upload_batch_size + 1} failed: {e}")

    print(f"[OK] Successfully uploaded {total_uploaded} vectors")
    return total_uploaded


def main():
    """Main ingestion pipeline"""
    start_time = time.time()

    print("\n" + "="*70)
    print(">>> Physical AI Textbook Ingestion Pipeline")
    print("="*70)

    # Verify credentials
    if not all([QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY]):
        print("[ERROR] Missing credentials in .env file")
        print(f"  QDRANT_URL: {'✓' if QDRANT_URL else '✗'}")
        print(f"  QDRANT_API_KEY: {'✓' if QDRANT_API_KEY else '✗'}")
        print(f"  COHERE_API_KEY: {'✓' if COHERE_API_KEY else '✗'}")
        return

    print(f"\n[OK] Credentials loaded")
    print(f"    Qdrant URL: {QDRANT_URL[:50]}...")
    print(f"    Embedding model: {EMBEDDING_MODEL}")

    # Test Qdrant connection
    try:
        collections = qdrant_client.get_collections()
        print(f"[OK] Connected to Qdrant Cloud")
        print(f"    Existing collections: {len(collections.collections)}")
    except Exception as e:
        print(f"[ERROR] Failed to connect to Qdrant: {e}")
        return

    # Create collection
    print(f"\n[*] Setting up collection...")
    create_collection()

    # Find all markdown files
    docs_dir = Path("../Humain-robotic-ai-book/docs")

    if not docs_dir.exists():
        print(f"[ERROR] Directory not found: {docs_dir.absolute()}")
        return

    print(f"\n[*] Scanning for markdown files in: {docs_dir.absolute()}")

    # Get both .md and .mdx files
    md_files = list(docs_dir.rglob("*.md"))
    mdx_files = list(docs_dir.rglob("*.mdx"))

    # Filter out system files
    md_files = [f for f in md_files if not f.name.startswith('_') and f.name != 'README.md']
    mdx_files = [f for f in mdx_files if not f.name.startswith('_')]

    all_files = md_files + mdx_files

    print(f"[OK] Found {len(all_files)} markdown files ({len(md_files)} .md, {len(mdx_files)} .mdx)")

    # Process all files
    print(f"\n[*] Processing markdown files...")
    all_documents = []

    for file_path in tqdm(all_files, desc="Processing files", unit="file"):
        docs = process_markdown_file(file_path)
        all_documents.extend(docs)

    print(f"\n[OK] Processed {len(all_files)} files")
    print(f"[*] Total chunks created: {len(all_documents)}")
    print(f"[*] Average chunks per file: {len(all_documents)/len(md_files):.1f}")

    # Upload to Qdrant
    if all_documents:
        total_uploaded = upload_to_qdrant(all_documents)

        # Verify upload
        time.sleep(2)  # Wait for Qdrant to index
        collection_info = qdrant_client.get_collection(collection_name=COLLECTION_NAME)

        elapsed_time = time.time() - start_time

        # Print summary
        print("\n" + "="*70)
        print(">>> INGESTION COMPLETE")
        print("="*70)
        print(f"[*] Files processed: {len(all_files)}")
        print(f"[*] Chunks created: {len(all_documents)}")
        print(f"[*] Vectors uploaded: {total_uploaded}")
        print(f"[*] Vectors in Qdrant: {collection_info.points_count}")
        print(f"[*] Collection: {COLLECTION_NAME}")
        print(f"[*] Time taken: {elapsed_time:.2f} seconds")
        print("="*70 + "\n")
    else:
        print("[ERROR] No documents to upload!")


if __name__ == "__main__":
    main()
