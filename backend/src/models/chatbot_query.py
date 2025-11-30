"""Chatbot Query data model."""
from datetime import datetime
from typing import Optional, Dict
from pydantic import BaseModel

# Optional SQLAlchemy imports
try:
    from sqlalchemy import Column, String, Integer, DateTime, ForeignKey, JSON, Text
    from sqlalchemy.orm import relationship
    from src.database.base import Base
    SQLALCHEMY_AVAILABLE = True
except ImportError:
    SQLALCHEMY_AVAILABLE = False
    # Dummy Base for when SQLAlchemy not available
    Base = type('Base', (), {})


if SQLALCHEMY_AVAILABLE:
    class ChatbotQuery(Base):
        """Chatbot Query entity model for database."""
        
        __tablename__ = "chatbot_queries"
        
        id = Column(Integer, primary_key=True, index=True)
        user_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
        timestamp = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)
        query_text = Column(Text, nullable=False)
        context = Column(JSON, default=dict)  # e.g., {"current_page_url": "...", "selected_text": "..."}
        
        # Relationships
        user = relationship("User", backref="chatbot_queries")
        response = relationship("ChatbotResponse", back_populates="query", uselist=False, cascade="all, delete-orphan")
else:
    # Dummy class when SQLAlchemy not available
    class ChatbotQuery:
        pass


class ChatbotQueryCreate(BaseModel):
    """Pydantic model for chatbot query creation."""
    queryText: str
    context: Optional[Dict] = None


class ChatbotQueryResponse(BaseModel):
    """Pydantic model for chatbot query response."""
    id: int
    user_id: int
    timestamp: datetime
    query_text: str
    context: Dict = {}
    
    class Config:
        from_attributes = True

