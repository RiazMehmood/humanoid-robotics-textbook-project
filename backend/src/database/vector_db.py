"""Vector Database (Qdrant) client configuration."""
from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from src.config import settings
from src.utils.logger import logger


class VectorDB:
    """Qdrant vector database client wrapper."""
    
    def __init__(self):
        """Initialize Qdrant client."""
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key
            )
            self.collection_name = settings.qdrant_collection_name
            self._ensure_collection()
        except Exception as e:
            logger.warning(f"Vector DB initialization failed: {e}. Will work without vector DB.")
            self.client = None
            self.collection_name = settings.qdrant_collection_name
    
    def _ensure_collection(self):
        """Ensure the collection exists, create if it doesn't."""
        try:
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]
            
            if self.collection_name not in collection_names:
                logger.info(f"Creating collection: {self.collection_name}")
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=768,  # Google Gemini embedding dimension
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Collection {self.collection_name} created successfully")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error ensuring collection: {e}")
            raise
    
    def add_points(
        self,
        points: List[PointStruct]
    ) -> bool:
        """
        Add points to the collection.
        
        Args:
            points: List of PointStruct objects to add
            
        Returns:
            True if successful
        """
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            return True
        except Exception as e:
            logger.error(f"Error adding points: {e}")
            raise
    
    def search(
        self,
        query_vector: List[float],
        limit: int = 10,
        score_threshold: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors.
        
        Args:
            query_vector: Query vector to search for
            limit: Maximum number of results
            score_threshold: Minimum score threshold
            
        Returns:
            List of search results with payloads
        """
        if self.client is None:
            logger.debug("Vector DB client not available, returning empty results")
            return []
        
        try:
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold
            )
            
            results = []
            for result in search_result:
                results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload
                })
            
            return results
        except Exception as e:
            logger.debug(f"Error searching vector DB: {e}, returning empty results")
            return []
    
    def delete_points(
        self,
        point_ids: List[int]
    ) -> bool:
        """
        Delete points by IDs.
        
        Args:
            point_ids: List of point IDs to delete
            
        Returns:
            True if successful
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=point_ids
            )
            return True
        except Exception as e:
            logger.error(f"Error deleting points: {e}")
            raise


# Global vector DB instance - initialize on first use
_vector_db_instance = None

def get_vector_db():
    """Get or create vector DB instance (lazy initialization)."""
    global _vector_db_instance
    if _vector_db_instance is None:
        try:
            _vector_db_instance = VectorDB()
        except Exception as e:
            logger.warning(f"Vector DB initialization failed: {e}. Running without vector DB.")
            # Return a mock object that returns empty results
            from unittest.mock import MagicMock
            _vector_db_instance = MagicMock()
            _vector_db_instance.search = lambda *args, **kwargs: []
            _vector_db_instance.client = None
    return _vector_db_instance

# For backward compatibility - lazy access
class LazyVectorDB:
    """Lazy wrapper for vector_db."""
    def __getattr__(self, name):
        return getattr(get_vector_db(), name)

vector_db = LazyVectorDB()

