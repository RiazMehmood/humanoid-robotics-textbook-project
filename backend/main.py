"""Main FastAPI application with global routing and middleware."""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from src.config import settings
from src.utils.errors import AppError, handle_app_error
from src.utils.logger import logger

# Import routers - make auth optional if SQLAlchemy not available
try:
    from src.api.chatbot import router as chatbot_router
    CHATBOT_ROUTER_AVAILABLE = True
    logger.info("✅ Chatbot router imported successfully")
except ImportError as e:
    logger.warning(f"Chatbot router not available: {e}")
    CHATBOT_ROUTER_AVAILABLE = False
    chatbot_router = None

try:
    from src.api.auth import router as auth_router
    AUTH_ROUTER_AVAILABLE = True
except ImportError as e:
    logger.warning(f"Auth router not available (database not installed): {e}")
    AUTH_ROUTER_AVAILABLE = False
    auth_router = None

# Import content ingestion router (optional)
try:
    from src.api.content_ingestion import router as content_ingestion_router
    CONTENT_INGESTION_AVAILABLE = True
    logger.info("✅ Content ingestion router imported successfully")
except ImportError as e:
    logger.debug(f"Content ingestion router not available: {e}")
    CONTENT_INGESTION_AVAILABLE = False
    content_ingestion_router = None

# Import personalization router (optional)
try:
    from src.api.personalization import router as personalization_router
    PERSONALIZATION_AVAILABLE = True
    logger.info("✅ Personalization router imported successfully")
except ImportError as e:
    logger.debug(f"Personalization router not available: {e}")
    PERSONALIZATION_AVAILABLE = False
    personalization_router = None

# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for the online textbook platform",
    version="1.0.0"
)

# Configure CORS
cors_origins = settings.get_cors_origins_list() + ["http://localhost:3000", "http://127.0.0.1:3000"]
logger.info(f"🌐 CORS allowed origins: {cors_origins}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
    allow_headers=["*"],
    expose_headers=["*"],
    max_age=3600,  # Cache preflight requests for 1 hour
)

# Include routers (only if available)
if CHATBOT_ROUTER_AVAILABLE and chatbot_router:
    app.include_router(chatbot_router)
    logger.info("🚀 Chatbot router loaded and registered")
else:
    logger.warning("⚠️  Chatbot router NOT loaded - check import errors above")

if AUTH_ROUTER_AVAILABLE and auth_router:
    app.include_router(auth_router)
    logger.info("🚀 Auth router loaded and registered")
else:
    logger.info("ℹ️  Auth router skipped (database not required for chatbot)")

if CONTENT_INGESTION_AVAILABLE and content_ingestion_router:
    app.include_router(content_ingestion_router)
    logger.info("🚀 Content ingestion router loaded and registered")
else:
    logger.warning("⚠️  Content ingestion router NOT loaded - check import errors above")

if PERSONALIZATION_AVAILABLE and personalization_router:
    app.include_router(personalization_router)
    logger.info("🚀 Personalization router loaded and registered")
else:
    logger.warning("⚠️  Personalization router NOT loaded - check import errors above")


# Global exception handler
@app.exception_handler(AppError)
async def app_error_handler(request, exc: AppError):
    """Handle application errors."""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "code": exc.code,
            "message": exc.message
        }
    )


# Root endpoint
@app.get("/")
async def read_root():
    """Root endpoint."""
    return {"message": "Hello, Physical AI & Humanoid Robotics Textbook Backend!"}


# Health check endpoint
@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


# CORS test endpoint
@app.options("/{full_path:path}")
async def options_handler(full_path: str):
    """Handle OPTIONS preflight requests for all paths."""
    return {"message": "OK"}


# CORS info endpoint (for debugging)
@app.get("/cors-info")
async def cors_info():
    """Get CORS configuration info (for debugging)."""
    return {
        "cors_origins": cors_origins,
        "cors_origins_from_env": settings.cors_origins,
        "cors_origins_list": settings.get_cors_origins_list(),
    }


# Startup event
@app.on_event("startup")
async def startup_event():
    """Application startup event."""
    logger.info("Application starting up...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Debug mode: {settings.debug}")
    logger.info(f"CORS Origins from env: {settings.cors_origins}")
    logger.info(f"CORS Origins list: {settings.get_cors_origins_list()}")


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Application shutdown event."""
    logger.info("Application shutting down...")


