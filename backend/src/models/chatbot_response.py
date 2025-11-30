"""Chatbot Response data model."""
from datetime import datetime
from typing import List, Optional, Dict
from pydantic import BaseModel

# Optional SQLAlchemy imports - don't fail if not available
SQLALCHEMY_AVAILABLE = False
try:
    from sqlalchemy import Column, String, Integer, DateTime, ForeignKey, JSON, Text, Float
    from sqlalchemy.orm import relationship
    try:
        from src.database.base import Base
        SQLALCHEMY_AVAILABLE = True
    except ImportError:
        Base = type('Base', (), {})
except ImportError:
    # SQLAlchemy not installed - that's OK
    Base = type('Base', (), {})


if SQLALCHEMY_AVAILABLE:
    class ChatbotResponse(Base):
        """Chatbot Response entity model for database."""
        
        __tablename__ = "chatbot_responses"
        
        id = Column(Integer, primary_key=True, index=True)
        query_id = Column(Integer, ForeignKey("chatbot_queries.id"), nullable=False, unique=True, index=True)
        timestamp = Column(DateTime, default=datetime.utcnow, nullable=False)
        response_text = Column(Text, nullable=False)
        source_references = Column(JSON, default=list)  # e.g., ["textbook_sections": [...], "external_links": [...]]
        confidence_score = Column(Float, nullable=True)
        
        # Relationships
        query = relationship("ChatbotQuery", back_populates="response")
else:
    # Dummy class when SQLAlchemy not available
    class ChatbotResponse:
        pass


class ChatbotResponseCreate(BaseModel):
    """Pydantic model for chatbot response creation."""
    query_id: int
    response_text: str
    source_references: Optional[List[str]] = None
    confidence_score: Optional[float] = None


class ChatbotResponseAPI(BaseModel):
    """Pydantic model for chatbot response API response."""
    responseText: str
    sourceReferences: List[str] = []
    confidenceScore: Optional[float] = None
    
    class Config:
        from_attributes = True

