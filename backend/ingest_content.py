import os
import glob
import requests
import argparse
from typing import List, Dict

def get_markdown_files(docs_dir: str) -> List[str]:
    """Get all markdown files from the docs directory."""
    # Match .md and .mdx files recursively
    files = glob.glob(os.path.join(docs_dir, "**/*.md"), recursive=True)
    files.extend(glob.glob(os.path.join(docs_dir, "**/*.mdx"), recursive=True))
    return files

def ingest_file(file_path: str, api_url: str, docs_root: str) -> bool:
    """Ingest a single markdown file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Create a relative URL path from the file path
        rel_path = os.path.relpath(file_path, docs_root)
        # Convert to web URL format (replace backslashes, remove extension)
        url_path = rel_path.replace("\\", "/").replace(".mdx", "").replace(".md", "")
        
        # Determine section from directory structure
        section = os.path.dirname(rel_path).replace("\\", "/")
        if section == ".":
            section = "root"
            
        payload = {
            "content": content,
            "url": f"/docs/{url_path}",
            "section": section
        }
        
        response = requests.post(f"{api_url}/api/v1/admin/content/ingest/markdown", json=payload) # Fixed endpoint path
        
        if response.status_code in [200, 201]:
            print(f"✅ Ingested: {rel_path}")
            return True
        else:
            print(f"❌ Failed: {rel_path} - {response.status_code} - {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Error processing {file_path}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Ingest textbook content into Vector DB")
    parser.add_argument("--docs-dir", default="frontend/docs", help="Path to docs directory")
    parser.add_argument("--api-url", default="http://localhost:8000", help="Backend API URL")
    args = parser.parse_args()
    
    print(f"🚀 Starting ingestion from {args.docs_dir} to {args.api_url}...")
    
    if not os.path.exists(args.docs_dir):
        print(f"❌ Directory not found: {args.docs_dir}")
        return
        
    files = get_markdown_files(args.docs_dir)
    print(f"📄 Found {len(files)} markdown files")
    
    success_count = 0
    for file_path in files:
        if ingest_file(file_path, args.api_url, args.docs_dir):
            success_count += 1
            
    print(f"\n✨ Ingestion complete! {success_count}/{len(files)} files processed successfully.")

if __name__ == "__main__":
    main()

