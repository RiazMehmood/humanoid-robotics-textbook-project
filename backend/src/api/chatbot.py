"""Chatbot API endpoints."""
from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from src.services.chatbot_service import chatbot_service
from src.utils.errors import AppError, handle_app_error
from src.utils.logger import logger

# Optional imports for database
try:
    from sqlalchemy.orm import Session
    from src.database.base import get_db
    from src.services.auth_service import get_current_user
    from src.models.user import User
    SQLALCHEMY_AVAILABLE = True
except ImportError:
    SQLALCHEMY_AVAILABLE = False
    # Create mock types
    Session = type('Session', (), {})
    def get_db():
        from unittest.mock import MagicMock
        yield MagicMock()
    def get_current_user(db, token):
        from unittest.mock import MagicMock
        user = MagicMock()
        user.id = 1
        return user

# Pydantic models (don't require SQLAlchemy)
try:
    from src.models.chatbot_query import ChatbotQueryCreate
    from src.models.chatbot_response import ChatbotResponseAPI
except ImportError as e:
    logger.warning(f"Could not import chatbot models: {e}")
    # Create minimal Pydantic models if imports fail
    from pydantic import BaseModel
    from typing import Optional, Dict, List
    
    class ChatbotQueryCreate(BaseModel):
        queryText: str
        context: Optional[Dict] = None
    
    class ChatbotResponseAPI(BaseModel):
        responseText: str
        sourceReferences: List[str] = []
        confidenceScore: Optional[float] = None

router = APIRouter(prefix="/chatbot", tags=["chatbot"])
security = HTTPBearer(auto_error=False)  # Make authentication optional for development


@router.post("/query", response_model=ChatbotResponseAPI, status_code=status.HTTP_200_OK)
async def submit_query(
    query_data: ChatbotQueryCreate,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    db: Session = Depends(get_db)
):
    """
    Submit a query to the AI chatbot.
    
    Based on ai-chatbot-api.openapi.yaml /query endpoint.
    Note: Authentication is optional for development - if no token provided, uses anonymous user.
    """
    try:
        # Get current user from token (optional for development)
        user_id = 1  # Default anonymous user ID for development
        
        # Try to get user from token if provided
        if credentials and credentials.credentials:
            try:
                token = credentials.credentials
                user = get_current_user(db, token)
                user_id = user.id
            except Exception as e:
                logger.debug(f"Token authentication failed: {e}, using anonymous user")
        
        # For development without database: use default user_id
        # In production with database, you should create/get anonymous user from DB
        try:
            # Try to create/get anonymous user if database is available
            from src.models.user import User
            if hasattr(db, 'query'):  # Check if it's a real database session
                anonymous_user = db.query(User).filter(User.email == "anonymous@example.com").first()
                if not anonymous_user:
                    from src.services.auth_service import get_password_hash
                    anonymous_user = User(
                        email="anonymous@example.com",
                        password_hash=get_password_hash("anonymous"),
                        preferences={},
                        interaction_history={}
                    )
                    db.add(anonymous_user)
                    db.commit()
                    db.refresh(anonymous_user)
                user_id = anonymous_user.id
        except Exception as e:
            logger.debug(f"Database user creation skipped: {e}, using default user_id=1")
        
        # Process query
        response_data = chatbot_service.process_query(
            db=db,
            user_id=user_id,
            query_text=query_data.queryText,
            context=query_data.context
        )
        
        return ChatbotResponseAPI(
            responseText=response_data["response_text"],
            sourceReferences=response_data["source_references"],
            confidenceScore=response_data["confidence_score"]
        )
    except AppError as e:
        raise handle_app_error(e)
    except Exception as e:
        logger.error(f"Chatbot query error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "code": "CHATBOT_ERROR",
                "message": f"Failed to process query: {str(e)}"
            }
        )

