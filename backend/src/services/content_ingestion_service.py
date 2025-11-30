"""Service for ingesting textbook content into Qdrant vector database."""
from typing import List, Dict, Any, Optional
from qdrant_client.models import PointStruct
from src.database.vector_db import get_vector_db
from src.services.chatbot_service import chatbot_service
from src.config import settings
from src.utils.logger import logger
import hashlib


class ContentIngestionService:
    """Service for ingesting and indexing textbook content in Qdrant."""
    
    def __init__(self):
        """Initialize the content ingestion service."""
        self.vector_db = get_vector_db()
        self.chatbot_service = chatbot_service
    
    def _generate_point_id(self, content: str, url: str) -> int:
        """Generate a consistent point ID from content and URL."""
        combined = f"{url}:{content[:100]}"
        hash_obj = hashlib.md5(combined.encode())
        # Convert first 8 bytes to int (positive)
        return int.from_bytes(hash_obj.digest()[:8], byteorder='big') % (2**63 - 1)
    
    def ingest_content(
        self,
        content: str,
        url: str,
        section: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> bool:
        """
        Ingest a single piece of content into the vector database.
        
        Args:
            content: Text content to ingest
            url: URL or identifier for the content
            section: Optional section/chapter identifier
            metadata: Optional additional metadata
            
        Returns:
            True if successful
        """
        try:
            # Get embedding for content
            embedding = self.chatbot_service.get_embedding(content)
            
            # Prepare payload
            payload = {
                "content": content,
                "url": url,
            }
            if section:
                payload["section"] = section
            if metadata:
                payload.update(metadata)
            
            # Generate point ID
            point_id = self._generate_point_id(content, url)
            
            # Create point
            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload=payload
            )
            
            # Add to vector DB
            self.vector_db.add_points([point])
            
            logger.info(f"Ingested content from {url} (point_id: {point_id})")
            return True
            
        except Exception as e:
            logger.error(f"Error ingesting content from {url}: {e}")
            return False
    
    def ingest_batch(
        self,
        contents: List[Dict[str, Any]]
    ) -> Dict[str, int]:
        """
        Ingest multiple pieces of content in batch.
        
        Args:
            contents: List of dicts with 'content', 'url', and optional 'section', 'metadata'
            
        Returns:
            Dict with 'success' and 'failed' counts
        """
        results = {"success": 0, "failed": 0}
        
        for item in contents:
            if self.ingest_content(
                content=item.get("content", ""),
                url=item.get("url", ""),
                section=item.get("section"),
                metadata=item.get("metadata")
            ):
                results["success"] += 1
            else:
                results["failed"] += 1
        
        logger.info(f"Batch ingestion complete: {results['success']} succeeded, {results['failed']} failed")
        return results
    
    def ingest_from_markdown(
        self,
        markdown_content: str,
        url: str,
        section: Optional[str] = None
    ) -> bool:
        """
        Ingest content from a markdown file, splitting into chunks.
        
        Args:
            markdown_content: Full markdown content
            url: URL for the content
            section: Optional section identifier
            
        Returns:
            True if successful
        """
        # Simple chunking: split by double newlines (paragraphs)
        # In production, use more sophisticated chunking
        chunks = [chunk.strip() for chunk in markdown_content.split("\n\n") if chunk.strip()]
        
        results = {"success": 0, "failed": 0}
        
        for i, chunk in enumerate(chunks):
            # Skip very short chunks
            if len(chunk) < 50:
                continue
            
            chunk_url = f"{url}#chunk-{i}"
            if self.ingest_content(
                content=chunk,
                url=chunk_url,
                section=section,
                metadata={"chunk_index": i, "total_chunks": len(chunks)}
            ):
                results["success"] += 1
            else:
                results["failed"] += 1
        
        logger.info(f"Ingested {results['success']} chunks from {url}")
        return results["failed"] == 0


# Global service instance
content_ingestion_service = ContentIngestionService()



