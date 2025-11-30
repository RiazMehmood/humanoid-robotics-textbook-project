"""API endpoints for content ingestion (admin/internal use)."""
from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from src.services.content_ingestion_service import content_ingestion_service
from src.utils.logger import logger

router = APIRouter(prefix="/admin/content", tags=["content-ingestion"])


class ContentItem(BaseModel):
    """Single content item for ingestion."""
    content: str
    url: str
    section: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None


class BatchIngestionRequest(BaseModel):
    """Request for batch content ingestion."""
    contents: List[ContentItem]


@router.post("/ingest", status_code=status.HTTP_201_CREATED)
async def ingest_content(item: ContentItem):
    """
    Ingest a single piece of content into the vector database.
    
    Note: This is an admin endpoint for populating the RAG knowledge base.
    """
    try:
        success = content_ingestion_service.ingest_content(
            content=item.content,
            url=item.url,
            section=item.section,
            metadata=item.metadata
        )
        
        if success:
            return {
                "status": "success",
                "message": f"Content from {item.url} ingested successfully"
            }
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to ingest content"
            )
    except Exception as e:
        logger.error(f"Error in content ingestion: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to ingest content: {str(e)}"
        )


@router.post("/ingest/batch", status_code=status.HTTP_201_CREATED)
async def ingest_batch(request: BatchIngestionRequest):
    """
    Ingest multiple pieces of content in batch.
    
    Note: This is an admin endpoint for populating the RAG knowledge base.
    """
    try:
        contents = [
            {
                "content": item.content,
                "url": item.url,
                "section": item.section,
                "metadata": item.metadata
            }
            for item in request.contents
        ]
        
        results = content_ingestion_service.ingest_batch(contents)
        
        return {
            "status": "success",
            "results": results,
            "message": f"Ingested {results['success']} items, {results['failed']} failed"
        }
    except Exception as e:
        logger.error(f"Error in batch content ingestion: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to ingest batch: {str(e)}"
        )


@router.post("/ingest/markdown", status_code=status.HTTP_201_CREATED)
async def ingest_markdown(
    content: str,
    url: str,
    section: Optional[str] = None
):
    """
    Ingest content from markdown, automatically chunking it.
    
    Note: This is an admin endpoint for populating the RAG knowledge base.
    """
    try:
        success = content_ingestion_service.ingest_from_markdown(
            markdown_content=content,
            url=url,
            section=section
        )
        
        if success:
            return {
                "status": "success",
                "message": f"Markdown content from {url} ingested successfully"
            }
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to ingest markdown content"
            )
    except Exception as e:
        logger.error(f"Error in markdown ingestion: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to ingest markdown: {str(e)}"
        )



