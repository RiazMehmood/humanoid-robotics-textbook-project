"""Common error handling infrastructure."""
from typing import Optional
from fastapi import HTTPException, status


class AppError(Exception):
    """Base application error."""
    
    def __init__(
        self,
        message: str,
        status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR,
        code: Optional[str] = None
    ):
        self.message = message
        self.status_code = status_code
        self.code = code or "INTERNAL_ERROR"
        super().__init__(self.message)


class ValidationError(AppError):
    """Validation error."""
    
    def __init__(self, message: str, code: Optional[str] = None):
        super().__init__(
            message,
            status_code=status.HTTP_400_BAD_REQUEST,
            code=code or "VALIDATION_ERROR"
        )


class NotFoundError(AppError):
    """Resource not found error."""
    
    def __init__(self, resource: str, identifier: Optional[str] = None):
        message = f"{resource} not found"
        if identifier:
            message += f": {identifier}"
        super().__init__(
            message,
            status_code=status.HTTP_404_NOT_FOUND,
            code="NOT_FOUND"
        )


class AuthenticationError(AppError):
    """Authentication error."""
    
    def __init__(self, message: str = "Authentication failed"):
        super().__init__(
            message,
            status_code=status.HTTP_401_UNAUTHORIZED,
            code="AUTHENTICATION_ERROR"
        )


class AuthorizationError(AppError):
    """Authorization error."""
    
    def __init__(self, message: str = "Insufficient permissions"):
        super().__init__(
            message,
            status_code=status.HTTP_403_FORBIDDEN,
            code="AUTHORIZATION_ERROR"
        )


def handle_app_error(error: AppError) -> HTTPException:
    """Convert AppError to HTTPException."""
    return HTTPException(
        status_code=error.status_code,
        detail={
            "code": error.code,
            "message": error.message
        }
    )





