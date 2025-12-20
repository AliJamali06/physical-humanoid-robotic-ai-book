"""
Document Ingestion Script for RAG Chatbot
Reads all book chapters, chunks them, generates embeddings, and stores in Qdrant
"""

import os
import re
from pathlib import Path
from typing import List, Dict
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import uuid
from dotenv import load_dotenv
from tqdm import tqdm
import time

# Load environment variables
load_dotenv()

# Initialize clients
cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = "physical_ai_docs"  # MUST match main.py
EMBEDDING_MODEL = "embed-english-v3.0"  # Cohere embedding model
EMBEDDING_DIMENSION = 1024  # Cohere embed-english-v3.0 dimension
CHUNK_SIZE = 800
CHUNK_OVERLAP = 200

def extract_frontmatter(content: str) -> Dict[str, str]:
    """Extract YAML frontmatter from MDX files"""
    frontmatter = {}
    match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)
    if match:
        for line in match.group(1).split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                frontmatter[key.strip()] = value.strip().strip('"'"'"'')
    return frontmatter

def clean_markdown(text: str) -> str:
    """Remove markdown syntax"""
    try:
        # Remove YAML frontmatter
        text = re.sub(r'^---\n.*?\n---\n', '', text, flags=re.DOTALL)

        # Remove code blocks (non-greedy matching)
        text = re.sub(r'```[^`]*```', '[Code Block]', text, flags=re.DOTALL)

        # Remove images
        text = re.sub(r'!\[[^\]]*\]\([^\)]*\)', '', text)

        # Remove links but keep text
        text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', text)

        # Remove HTML tags (safer pattern)
        text = re.sub(r'<[^<>]+>', '', text)

        # Normalize whitespace
        text = re.sub(r'\n{3,}', '\n\n', text)

        # Remove excessive spaces
        text = re.sub(r' {2,}', ' ', text)

        return text.strip()
    except Exception as e:
        print(f"Error in clean_markdown: {e}")
        return text[:1000]  # Return first 1000 chars as fallback

def chunk_text(text: str, chunk_size: int = 800, overlap: int = 200) -> List[str]:
    """Split text into overlapping chunks"""
    if not text or len(text) == 0:
        return []

    # Safety check - if text is abnormally large, truncate it
    if len(text) > 1_000_000:  # 1MB limit
        print(f"Warning: Text too large ({len(text)} chars), truncating to 1MB")
        text = text[:1_000_000]

    chunks = []
    start = 0
    text_length = len(text)

    try:
        while start < text_length:
            # Calculate end position
            end = min(start + chunk_size, text_length)

            # Find sentence boundary if not at the end of text
            if end < text_length:
                # Look for sentence ending punctuation within the chunk
                sentence_end = max(
                    text.rfind('.', start, end),
                    text.rfind('!', start, end),
                    text.rfind('?', start, end),
                    text.rfind('\n', start, end)  # Also break at newlines
                )
                # Only use sentence boundary if it's reasonably far from start
                if sentence_end > start + 100:
                    end = sentence_end + 1

            # Extract and clean the chunk
            chunk = text[start:end].strip()
            if chunk and len(chunk) > 10:  # Only add meaningful chunks
                chunks.append(chunk)

            # Calculate next start position with overlap
            # CRITICAL: Always move forward by at least (chunk_size - overlap)
            if end < text_length:
                next_start = end - overlap
                # Ensure we're always moving forward
                if next_start <= start:
                    next_start = start + (chunk_size - overlap)
                start = next_start
            else:
                break  # We've reached the end

            # Safety: prevent infinite loops
            if len(chunks) > 1000:  # Reasonable limit
                print(f"Warning: Too many chunks generated ({len(chunks)}), stopping")
                break

    except Exception as e:
        print(f"Error chunking text: {e}")
        import traceback
        traceback.print_exc()
        # Return text as single chunk as fallback
        if text.strip():
            chunks.append(text[:chunk_size])

    return chunks

def process_document(file_path: Path) -> List[Dict]:
    """Process a single document"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Check file size
        if len(content) > 5_000_000:  # 5MB limit for a single file
            print(f"Warning: {file_path.name} is very large ({len(content)} chars), skipping")
            return []

        frontmatter = extract_frontmatter(content)
        title = frontmatter.get('title', file_path.stem)

        parts = file_path.parts
        module = "Unknown"
        chapter = "Unknown"

        for i, part in enumerate(parts):
            if part.startswith('module-'):
                module = part.replace('module-', 'Module ')
                if i + 1 < len(parts):
                    chapter = parts[i + 1].replace('.mdx', '').replace('-', ' ').title()

        clean_text = clean_markdown(content)
        print(f"  Cleaned text length: {len(clean_text)} chars")

        chunks = chunk_text(clean_text)
        print(f"  Generated {len(chunks)} chunks")

        documents = []
        for i, chunk in enumerate(chunks):
            documents.append({
                'text': chunk,
                'metadata': {
                    'title': title,
                    'module': module,
                    'chapter': chapter,
                    'file_path': str(file_path),
                    'chunk_index': i
                }
            })
        return documents

    except Exception as e:
        print(f"Error processing {file_path.name}: {e}")
        import traceback
        traceback.print_exc()
        return []

def create_collection():
    """Create Qdrant collection"""
    try:
        qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
    except:
        pass
    qdrant_client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=EMBEDDING_DIMENSION, distance=Distance.COSINE)
    )
    print(f"Created collection: {COLLECTION_NAME}")

def ingest_documents(docs_dir: str):
    """Main ingestion function"""
    print("\n" + "="*60)
    print(">>> Starting Document Ingestion")
    print("="*60)

    start_time = time.time()

    # Create collection
    print("\n[*] Creating Qdrant collection...")
    create_collection()

    docs_path = Path(docs_dir)

    # Collect all files
    mdx_files = [f for f in docs_path.rglob("*.mdx") if not f.name.startswith('_')]
    md_files = [f for f in docs_path.rglob("*.md") if not f.name.startswith('_') and f.name != 'README.md']
    all_files = mdx_files + md_files

    print(f"\n[*] Found {len(all_files)} files to process")
    print(f"    - MDX files: {len(mdx_files)}")
    print(f"    - MD files: {len(md_files)}")

    # Process documents with progress bar
    all_documents = []
    files_processed = 0
    files_failed = 0

    print("\n[*] Processing documents...")
    for file_path in tqdm(all_files, desc="Reading files", unit="file"):
        try:
            docs = process_document(file_path)
            if docs:
                all_documents.extend(docs)
                files_processed += 1
        except Exception as e:
            print(f"\n[ERROR] Error processing {file_path.name}: {e}")
            files_failed += 1

    if len(all_documents) == 0:
        print("\n[ERROR] No documents to index!")
        return

    print(f"\n[OK] Processed {files_processed} files ({files_failed} failed)")
    print(f"[*] Total chunks generated: {len(all_documents)}")

    # Upload to Qdrant with progress bar
    print(f"\n[*] Uploading to Qdrant...")
    batch_size = 96  # Cohere allows max 96 texts per request
    num_batches = (len(all_documents) + batch_size - 1) // batch_size

    uploaded_count = 0

    for i in tqdm(range(0, len(all_documents), batch_size), desc="Uploading batches", unit="batch", total=num_batches):
        batch = all_documents[i:i + batch_size]
        texts = [doc['text'] for doc in batch]

        try:
            # Generate embeddings using Cohere (batch processing)
            response = cohere_client.embed(
                texts=texts,
                model=EMBEDDING_MODEL,
                input_type="search_document"
            )
            embeddings = response.embeddings

            points = []
            for doc, embedding in zip(batch, embeddings):
                points.append(PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload={
                        'text': doc['text'],
                        'title': doc['metadata']['title'],
                        'module': doc['metadata']['module'],
                        'chapter': doc['metadata']['chapter']
                    }
                ))

            qdrant_client.upsert(collection_name=COLLECTION_NAME, points=points)
            uploaded_count += len(batch)
        except Exception as e:
            print(f"\n[ERROR] Error uploading batch {i//batch_size + 1}: {e}")

    # Get final count from Qdrant
    collection_info = qdrant_client.get_collection(collection_name=COLLECTION_NAME)
    vector_count = collection_info.points_count

    elapsed_time = time.time() - start_time

    # Print summary
    print("\n" + "="*60)
    print(">>> INGESTION COMPLETE")
    print("="*60)
    print(f"[*] Files processed: {files_processed}")
    print(f"[*] Files failed: {files_failed}")
    print(f"[*] Total chunks created: {len(all_documents)}")
    print(f"[*] Vectors in Qdrant: {vector_count}")
    print(f"[*] Time taken: {elapsed_time:.2f} seconds")
    print("="*60 + "\n")

if __name__ == "__main__":
    # Path relative to backend/ directory
    DOCS_DIR = "../docs"
    if not os.path.exists(DOCS_DIR):
        DOCS_DIR = "../Humain-robotic-ai-book/docs"  # Fallback

    if os.path.exists(DOCS_DIR):
        print(f"[*] Using docs directory: {DOCS_DIR}")
        ingest_documents(DOCS_DIR)
    else:
        print(f"Error: No docs directory found. Tried ../docs and ../Humain-robotic-ai-book/docs")
