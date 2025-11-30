"""Authentication API endpoints."""
from datetime import datetime, timedelta
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from src.database.base import get_db
from src.models.user import User, UserCreate, UserResponse
from src.models.auth_token import TokenResponse
from src.services.auth_service import (
    register_user,
    authenticate_user,
    create_access_token,
    create_refresh_token,
    decode_token,
    create_user_session,
    deactivate_user_session,
    get_current_user
)
from src.config import settings
from src.utils.errors import AuthenticationError, handle_app_error
from src.utils.logger import logger

router = APIRouter(prefix="/auth", tags=["authentication"])
security = HTTPBearer()


@router.post("/register", response_model=dict, status_code=status.HTTP_201_CREATED)
async def register(user_data: UserCreate, db: Session = Depends(get_db)):
    """
    Register a new user.
    
    Based on authentication-api.openapi.yaml /register endpoint.
    """
    try:
        user = register_user(
            db, 
            user_data.email, 
            user_data.password,
            background_info=user_data.background_info
        )
        
        # Create access token
        access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
        access_token = create_access_token(
            data={"sub": str(user.id), "email": user.email},
            expires_delta=access_token_expires
        )
        
        # Create refresh token
        refresh_token = create_refresh_token(
            data={"sub": str(user.id), "email": user.email}
        )
        
        # Store refresh token in database
        refresh_expiration = datetime.utcnow() + timedelta(
            days=settings.refresh_token_expire_days
        )
        create_user_session(db, user.id, refresh_token, refresh_expiration)
        
        return {
            "message": "User registered successfully",
            "userId": str(user.id),
            "token": access_token
        }
    except AuthenticationError as e:
        raise handle_app_error(e)
    except Exception as e:
        logger.error(f"Registration error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"code": "REGISTRATION_ERROR", "message": str(e)}
        )


@router.post("/login", response_model=dict)
async def login(user_data: UserCreate, db: Session = Depends(get_db)):
    """
    Authenticate and log in a user.
    
    Based on authentication-api.openapi.yaml /login endpoint.
    """
    try:
        user = authenticate_user(db, user_data.email, user_data.password)
        if not user:
            raise AuthenticationError("Invalid email or password")
        
        # Create access token
        access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
        access_token = create_access_token(
            data={"sub": str(user.id), "email": user.email},
            expires_delta=access_token_expires
        )
        
        # Create refresh token
        refresh_token = create_refresh_token(
            data={"sub": str(user.id), "email": user.email}
        )
        
        # Store refresh token in database
        refresh_expiration = datetime.utcnow() + timedelta(
            days=settings.refresh_token_expire_days
        )
        create_user_session(db, user.id, refresh_token, refresh_expiration)
        
        return {
            "message": "User logged in successfully",
            "token": access_token
        }
    except AuthenticationError as e:
        raise handle_app_error(e)
    except Exception as e:
        logger.error(f"Login error: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"code": "LOGIN_ERROR", "message": str(e)}
        )


@router.post("/logout", response_model=dict)
async def logout(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
):
    """
    Log out a user.
    
    Based on authentication-api.openapi.yaml /logout endpoint.
    """
    try:
        token = credentials.credentials
        deactivate_user_session(db, token)
        
        return {"message": "User logged out successfully"}
    except Exception as e:
        logger.error(f"Logout error: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"code": "LOGOUT_ERROR", "message": "Invalid token"}
        )


@router.post("/refresh-token", response_model=dict)
async def refresh_token(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
):
    """
    Refresh an expired authentication token.
    
    Based on authentication-api.openapi.yaml /refresh-token endpoint.
    """
    try:
        refresh_token_value = credentials.credentials
        payload = decode_token(refresh_token_value)
        
        # Verify it's a refresh token
        if payload.get("type") != "refresh":
            raise AuthenticationError("Invalid token type")
        
        user_id = payload.get("sub")
        if not user_id:
            raise AuthenticationError("Invalid token payload")
        
        # Verify token exists and is active in database
        from src.models.auth_token import AuthToken
        token_record = db.query(AuthToken).filter(
            AuthToken.token_value == refresh_token_value,
            AuthToken.is_active == True
        ).first()
        
        if not token_record:
            raise AuthenticationError("Token not found or inactive")
        
        # Get user
        user = db.query(User).filter(User.id == int(user_id)).first()
        if not user:
            raise AuthenticationError("User not found")
        
        # Create new access token
        access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
        access_token = create_access_token(
            data={"sub": str(user.id), "email": user.email},
            expires_delta=access_token_expires
        )
        
        return {
            "message": "Token refreshed successfully",
            "token": access_token
        }
    except AuthenticationError as e:
        raise handle_app_error(e)
    except Exception as e:
        logger.error(f"Refresh token error: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"code": "REFRESH_ERROR", "message": str(e)}
        )

