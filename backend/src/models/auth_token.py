"""Authentication Token/Session data model."""
from datetime import datetime
from sqlalchemy import Column, String, Integer, DateTime, Boolean, ForeignKey
from sqlalchemy.orm import relationship
from pydantic import BaseModel
from src.database.base import Base


class AuthToken(Base):
    """Authentication Token/Session entity model for database."""
    
    __tablename__ = "auth_tokens"
    
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
    token_value = Column(String, unique=True, nullable=False, index=True)
    expiration_timestamp = Column(DateTime, nullable=False, index=True)
    is_active = Column(Boolean, default=True, index=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    last_used_at = Column(DateTime, nullable=True)
    
    # Relationships
    user = relationship("User", back_populates="auth_tokens")


class TokenCreate(BaseModel):
    """Pydantic model for token creation."""
    user_id: int
    token_value: str
    expiration_timestamp: datetime


class TokenResponse(BaseModel):
    """Pydantic model for token response."""
    id: int
    user_id: int
    token_value: str
    expiration_timestamp: datetime
    is_active: bool
    created_at: datetime
    last_used_at: datetime | None
    
    class Config:
        from_attributes = True

