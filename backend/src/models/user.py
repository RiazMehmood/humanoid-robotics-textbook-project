"""User data model."""
from datetime import datetime
from typing import Optional, Dict, List
from sqlalchemy import Column, String, Integer, DateTime, JSON, Boolean
from sqlalchemy.orm import relationship
from pydantic import BaseModel, EmailStr
from src.database.base import Base


class User(Base):
    """User entity model for database."""
    
    __tablename__ = "users"
    
    id = Column(Integer, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    password_hash = Column(String, nullable=False)
    preferences = Column(JSON, default=dict)  # e.g., {"language_code": "en", "theme": "light"}
    interaction_history = Column(JSON, default=dict)  # e.g., {"read_chapters": [], "chatbot_queries": []}
    background_info = Column(JSON, default=dict)  # User background questions: {"software_exp": "...", "hardware_exp": "...", etc.}
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    is_active = Column(Boolean, default=True)
    
    # Relationships
    auth_tokens = relationship("AuthToken", back_populates="user", cascade="all, delete-orphan")


class UserCreate(BaseModel):
    """Pydantic model for user creation."""
    email: EmailStr
    password: str
    background_info: Optional[Dict] = None  # User background questions


class UserResponse(BaseModel):
    """Pydantic model for user response."""
    id: int
    email: str
    preferences: Dict = {}
    interaction_history: Dict = {}
    created_at: datetime
    updated_at: datetime
    is_active: bool
    
    class Config:
        from_attributes = True


class UserUpdate(BaseModel):
    """Pydantic model for user update."""
    preferences: Optional[Dict] = None
    interaction_history: Optional[Dict] = None

