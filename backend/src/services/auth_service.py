"""Authentication/authorization service."""
from datetime import datetime, timedelta
from typing import Optional
from passlib.context import CryptContext
import bcrypt
from jose import JWTError, jwt
from sqlalchemy.orm import Session
from src.models.user import User
from src.models.auth_token import AuthToken
from src.config import settings
from src.utils.errors import AuthenticationError, NotFoundError
from src.utils.logger import logger

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash.
    
    Note: bcrypt has a maximum password length of 72 bytes.
    Passwords longer than 72 bytes will be truncated to exactly 72 bytes before verification.
    """
    # Truncate password to 72 bytes (bcrypt limitation) for verification
    # Use the same truncation function to ensure consistency
    plain_password = _truncate_password_to_72_bytes(plain_password)
    
    # Ensure password is <= 72 bytes
    password_bytes = plain_password.encode('utf-8')
    if len(password_bytes) > 72:
        password_bytes = password_bytes[:72]
        plain_password = password_bytes.decode('utf-8', errors='ignore')
    
    # Use bcrypt directly for verification to avoid passlib issues
    try:
        password_bytes = plain_password.encode('utf-8')
        hashed_bytes = hashed_password.encode('utf-8')
        return bcrypt.checkpw(password_bytes, hashed_bytes)
    except Exception:
        # Fallback to passlib if bcrypt direct fails
        return pwd_context.verify(plain_password, hashed_password)


def _truncate_password_to_72_bytes(password: str) -> str:
    """
    Truncate password to exactly 72 bytes or less.
    This is a helper function to ensure passwords meet bcrypt's 72-byte limit.
    """
    if not isinstance(password, str):
        password = str(password)
    
    # Convert to bytes to check actual byte length
    password_bytes = password.encode('utf-8')
    original_length = len(password_bytes)
    
    # If already <= 72 bytes, return as-is
    if original_length <= 72:
        return password
    
    # Truncate to exactly 72 bytes
    password_bytes = password_bytes[:72]
    
    # Decode back to string, handling potential incomplete multi-byte characters
    # Use errors='ignore' to skip any incomplete characters at the end
    truncated_password = password_bytes.decode('utf-8', errors='ignore')
    
    # Verify the truncated password, when re-encoded, is <= 72 bytes
    # Keep truncating if needed (shouldn't happen, but safety check)
    iterations = 0
    while True:
        final_bytes = truncated_password.encode('utf-8')
        if len(final_bytes) <= 72:
            break
        iterations += 1
        if iterations > 10:  # Safety limit
            logger.error(f"Truncation loop exceeded limit, forcing empty password")
            truncated_password = ""
            break
        # If somehow still too long, truncate one more byte and try again
        password_bytes = password_bytes[:-1]
        if len(password_bytes) == 0:
            truncated_password = ""
            break
        truncated_password = password_bytes.decode('utf-8', errors='ignore')
    
    final_length = len(truncated_password.encode('utf-8'))
    if original_length > 72:
        logger.info(f"Password truncated from {original_length} to {final_length} bytes")
    
    return truncated_password


def get_password_hash(password: str) -> str:
    """Hash a password.
    
    Note: bcrypt has a maximum password length of 72 bytes.
    Passwords longer than 72 bytes will be truncated to exactly 72 bytes.
    """
    # Convert to bytes first to check length
    if not isinstance(password, str):
        password = str(password)
    
    original_password = password
    password_bytes = password.encode('utf-8')
    original_length = len(password_bytes)
    
    # Truncate to exactly 72 bytes if needed
    if original_length > 72:
        password_bytes = password_bytes[:72]
        # Decode back to string
        password = password_bytes.decode('utf-8', errors='ignore')
        # Verify it's <= 72 bytes when re-encoded
        final_bytes = password.encode('utf-8')
        final_length = len(final_bytes)
        
        # If still too long (shouldn't happen), keep truncating
        while final_length > 72 and len(password_bytes) > 0:
            password_bytes = password_bytes[:-1]
            password = password_bytes.decode('utf-8', errors='ignore')
            final_bytes = password.encode('utf-8')
            final_length = len(final_bytes)
        
        if final_length > 72:
            # Absolute last resort: use empty string
            logger.error(f"Password still {final_length} bytes after all truncation attempts!")
            password = ""
            final_length = 0
        
        logger.info(f"Password truncated from {original_length} to {final_length} bytes")
    else:
        final_length = original_length
    
    # CRITICAL: Convert to bytes and ensure it's exactly <= 72 bytes before hashing
    # This is the final safety check
    password_bytes_final = password.encode('utf-8')
    if len(password_bytes_final) > 72:
        logger.error(f"CRITICAL: Password is {len(password_bytes_final)} bytes! Force truncating to 72.")
        password_bytes_final = password_bytes_final[:72]
        password = password_bytes_final.decode('utf-8', errors='ignore')
    
    # Log the final password length before hashing
    logger.debug(f"Final password length before hashing: {len(password.encode('utf-8'))} bytes")
    
    # Now hash the (possibly truncated) password
    # Use bcrypt directly to avoid passlib's internal testing issues during initialization
    try:
        # Hash using bcrypt directly to ensure we control the password length
        password_bytes_for_hash = password.encode('utf-8')
        if len(password_bytes_for_hash) > 72:
            logger.error(f"Password bytes for hash still {len(password_bytes_for_hash)} bytes! Truncating.")
            password_bytes_for_hash = password_bytes_for_hash[:72]
        
        # Use bcrypt directly to hash
        salt = bcrypt.gensalt()
        hashed = bcrypt.hashpw(password_bytes_for_hash, salt)
        
        # Return the hash as a string (passlib format)
        return hashed.decode('utf-8')
    except ValueError as e:
        if "72" in str(e) or "bytes" in str(e).lower():
            logger.error(f"Bcrypt error: {e}")
            # Last resort: use even shorter password
            password_bytes_for_hash = password.encode('utf-8')[:71]
            salt = bcrypt.gensalt()
            hashed = bcrypt.hashpw(password_bytes_for_hash, salt)
            return hashed.decode('utf-8')
        raise
    except Exception as e:
        logger.error(f"Unexpected error hashing password: {e}")
        raise


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token.
    
    Args:
        data: Data to encode in the token
        expires_delta: Optional expiration time delta
        
    Returns:
        Encoded JWT token
    """
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(
            minutes=settings.access_token_expire_minutes
        )
    
    to_encode.update({"exp": expire, "type": "access"})
    encoded_jwt = jwt.encode(
        to_encode,
        settings.secret_key,
        algorithm=settings.algorithm
    )
    return encoded_jwt


def create_refresh_token(data: dict) -> str:
    """
    Create a JWT refresh token.
    
    Args:
        data: Data to encode in the token
        
    Returns:
        Encoded JWT refresh token
    """
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(days=settings.refresh_token_expire_days)
    to_encode.update({"exp": expire, "type": "refresh"})
    encoded_jwt = jwt.encode(
        to_encode,
        settings.secret_key,
        algorithm=settings.algorithm
    )
    return encoded_jwt


def decode_token(token: str) -> dict:
    """
    Decode and verify a JWT token.
    
    Args:
        token: JWT token to decode
        
    Returns:
        Decoded token payload
        
    Raises:
        AuthenticationError: If token is invalid or expired
    """
    try:
        payload = jwt.decode(
            token,
            settings.secret_key,
            algorithms=[settings.algorithm]
        )
        return payload
    except JWTError as e:
        logger.error(f"Token decode error: {e}")
        raise AuthenticationError("Invalid or expired token")


def register_user(db: Session, email: str, password: str, background_info: Optional[dict] = None) -> User:
    """
    Register a new user.
    
    Args:
        db: Database session
        email: User email
        password: Plain text password
        background_info: Optional user background information (software/hardware/robotics experience)
        
    Returns:
        Created user
        
    Raises:
        ValidationError: If user already exists
    """
    # Check if user already exists
    existing_user = db.query(User).filter(User.email == email).first()
    if existing_user:
        raise AuthenticationError("User with this email already exists")
    
    # Create new user
    hashed_password = get_password_hash(password)
    new_user = User(
        email=email,
        password_hash=hashed_password,
        preferences={},
        interaction_history={},
        background_info=background_info or {}
    )
    
    db.add(new_user)
    db.commit()
    db.refresh(new_user)
    
    logger.info(f"User registered: {email}")
    return new_user


def authenticate_user(db: Session, email: str, password: str) -> Optional[User]:
    """
    Authenticate a user.
    
    Args:
        db: Database session
        email: User email
        password: Plain text password
        
    Returns:
        User if authentication successful, None otherwise
    """
    user = db.query(User).filter(User.email == email).first()
    if not user:
        return None
    
    if not verify_password(password, user.password_hash):
        return None
    
    if not user.is_active:
        raise AuthenticationError("User account is inactive")
    
    return user


def create_user_session(
    db: Session,
    user_id: int,
    token_value: str,
    expiration: datetime
) -> AuthToken:
    """
    Create a user session token record.
    
    Args:
        db: Database session
        user_id: User ID
        token_value: Token string
        expiration: Token expiration timestamp
        
    Returns:
        Created auth token
    """
    auth_token = AuthToken(
        user_id=user_id,
        token_value=token_value,
        expiration_timestamp=expiration,
        is_active=True
    )
    
    db.add(auth_token)
    db.commit()
    db.refresh(auth_token)
    
    return auth_token


def deactivate_user_session(db: Session, token_value: str) -> bool:
    """
    Deactivate a user session.
    
    Args:
        db: Database session
        token_value: Token to deactivate
        
    Returns:
        True if successful
    """
    token = db.query(AuthToken).filter(
        AuthToken.token_value == token_value,
        AuthToken.is_active == True
    ).first()
    
    if token:
        token.is_active = False
        db.commit()
        return True
    
    return False


def get_current_user(db: Session, token: str) -> User:
    """
    Get current user from token.
    
    Args:
        db: Database session
        token: JWT token
        
    Returns:
        Current user
        
    Raises:
        AuthenticationError: If token is invalid
        NotFoundError: If user not found
    """
    payload = decode_token(token)
    user_id: int = payload.get("sub")
    
    if user_id is None:
        raise AuthenticationError("Invalid token payload")
    
    user = db.query(User).filter(User.id == user_id).first()
    if user is None:
        raise NotFoundError("User", str(user_id))
    
    return user



