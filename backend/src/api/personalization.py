"""API endpoints for content personalization and translation."""
from typing import Optional, List, Dict, Any
from fastapi import APIRouter, Depends, HTTPException, status, Query
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from src.services.personalization_service import personalization_service
from src.services.translation_service import translation_service
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
    Session = type('Session', (), {})
    def get_db():
        from unittest.mock import MagicMock
        yield MagicMock()
    def get_current_user(db, token):
        from unittest.mock import MagicMock
        user = MagicMock()
        user.id = 1
        return user

# Pydantic models
from pydantic import BaseModel

router = APIRouter(prefix="/personalization", tags=["personalization"])
security = HTTPBearer(auto_error=False)  # Make authentication optional for development


class PreferencesUpdate(BaseModel):
    """Request model for updating user preferences."""
    language: Optional[str] = None
    theme: Optional[str] = None


class PreferencesResponse(BaseModel):
    """Response model for user preferences."""
    language: str
    theme: str


class TranslationResponse(BaseModel):
    """Response model for translated content."""
    contentId: str
    translatedText: str
    sourceLanguage: str
    targetLanguage: str


class RecommendationItem(BaseModel):
    """Model for a content recommendation."""
    contentId: str
    title: str
    type: Optional[str] = None
    reason: Optional[str] = None


class RecommendationsResponse(BaseModel):
    """Response model for content recommendations."""
    recommendations: List[RecommendationItem]


@router.get("/preferences", response_model=PreferencesResponse, status_code=status.HTTP_200_OK)
async def get_preferences(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    db: Session = Depends(get_db)
):
    """
    Get user content preferences.
    
    Based on content-personalization-api.openapi.yaml /preferences GET endpoint.
    """
    try:
        # Get current user from token (optional for development)
        user_id = 1  # Default anonymous user ID for development
        
        if credentials and credentials.credentials:
            try:
                token = credentials.credentials
                user = get_current_user(db, token)
                user_id = user.id
            except Exception as e:
                logger.debug(f"Token authentication failed: {e}, using anonymous user")
        
        preferences = personalization_service.get_user_preferences(db, user_id)
        
        return PreferencesResponse(
            language=preferences.get("language", "en"),
            theme=preferences.get("theme", "light")
        )
    except AppError as e:
        raise handle_app_error(e)
    except Exception as e:
        logger.error(f"Error getting preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "code": "PREFERENCES_ERROR",
                "message": f"Failed to get preferences: {str(e)}"
            }
        )


@router.put("/preferences", status_code=status.HTTP_200_OK)
async def update_preferences(
    preferences_update: PreferencesUpdate,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    db: Session = Depends(get_db)
):
    """
    Update user content preferences.
    
    Based on content-personalization-api.openapi.yaml /preferences PUT endpoint.
    """
    try:
        # Get current user from token (optional for development)
        user_id = 1  # Default anonymous user ID for development
        
        if credentials and credentials.credentials:
            try:
                token = credentials.credentials
                user = get_current_user(db, token)
                user_id = user.id
            except Exception as e:
                logger.debug(f"Token authentication failed: {e}, using anonymous user")
        
        # Build preferences dict from update
        prefs_dict = {}
        if preferences_update.language is not None:
            prefs_dict["language"] = preferences_update.language
        if preferences_update.theme is not None:
            prefs_dict["theme"] = preferences_update.theme
        
        updated_prefs = personalization_service.update_user_preferences(
            db, user_id, prefs_dict
        )
        
        return {
            "message": "Preferences updated successfully",
            "preferences": updated_prefs
        }
    except AppError as e:
        raise handle_app_error(e)
    except Exception as e:
        logger.error(f"Error updating preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "code": "PREFERENCES_ERROR",
                "message": f"Failed to update preferences: {str(e)}"
            }
        )


@router.get("/content/{contentId}/translate", response_model=TranslationResponse, status_code=status.HTTP_200_OK)
async def translate_content(
    contentId: str,
    targetLanguage: str = Query(..., description="Target language code (e.g., 'es')"),
    sourceLanguage: str = Query("en", description="Source language code (e.g., 'en')"),
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    db: Session = Depends(get_db)
):
    """
    Get translated content for a specific item.
    
    Based on content-personalization-api.openapi.yaml /content/{contentId}/translate endpoint.
    """
    try:
        # Get current user from token to check preferences
        user_id = 1  # Default anonymous user ID for development
        
        if credentials and credentials.credentials:
            try:
                token = credentials.credentials
                user = get_current_user(db, token)
                user_id = user.id
            except Exception as e:
                logger.debug(f"Token authentication failed: {e}, using anonymous user")
        
        # Get user preferences to determine source language if not provided
        if sourceLanguage == "en":
            prefs = personalization_service.get_user_preferences(db, user_id)
            user_lang = prefs.get("language", "en")
            if user_lang != "en":
                sourceLanguage = user_lang
        
        # For now, we'll use a mock content lookup
        # In a real implementation, you'd fetch the actual content from a database or file system
        mock_content = f"Content for {contentId}"
        
        # Translate the content
        translation_result = translation_service.translate_content(
            content=mock_content,
            source_language=sourceLanguage,
            target_language=targetLanguage
        )
        
        return TranslationResponse(
            contentId=contentId,
            translatedText=translation_result["translatedText"],
            sourceLanguage=translation_result["sourceLanguage"],
            targetLanguage=translation_result["targetLanguage"]
        )
    except AppError as e:
        raise handle_app_error(e)
    except Exception as e:
        logger.error(f"Error translating content: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "code": "TRANSLATION_ERROR",
                "message": f"Failed to translate content: {str(e)}"
            }
        )


@router.get("/content/recommendations", response_model=RecommendationsResponse, status_code=status.HTTP_200_OK)
async def get_recommendations(
    limit: int = Query(5, ge=1, le=20, description="Maximum number of recommendations"),
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    db: Session = Depends(get_db)
):
    """
    Get personalized content recommendations for the user.
    
    Based on content-personalization-api.openapi.yaml /content/recommendations endpoint.
    """
    try:
        # Get current user from token (optional for development)
        user_id = 1  # Default anonymous user ID for development
        
        if credentials and credentials.credentials:
            try:
                token = credentials.credentials
                user = get_current_user(db, token)
                user_id = user.id
            except Exception as e:
                logger.debug(f"Token authentication failed: {e}, using anonymous user")
        
        recommendations = personalization_service.get_content_recommendations(
            db, user_id, limit=limit
        )
        
        return RecommendationsResponse(
            recommendations=[
                RecommendationItem(**rec) for rec in recommendations
            ]
        )
    except AppError as e:
        raise handle_app_error(e)
    except Exception as e:
        logger.error(f"Error getting recommendations: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "code": "RECOMMENDATIONS_ERROR",
                "message": f"Failed to get recommendations: {str(e)}"
            }
        )


