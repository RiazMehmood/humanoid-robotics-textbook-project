"""Database base configuration."""
from src.config import settings
from src.utils.logger import logger

# Try to import SQLAlchemy - optional for development
try:
    from sqlalchemy.ext.declarative import declarative_base
    from sqlalchemy.orm import sessionmaker
    from sqlalchemy import create_engine
    SQLALCHEMY_AVAILABLE = True
except ImportError:
    SQLALCHEMY_AVAILABLE = False
    logger.warning("SQLAlchemy not installed. Database features disabled.")
    # Create a dummy Base for models
    class declarative_base:
        @staticmethod
        def __call__():
            return type('Base', (), {})

# Base class for models
if SQLALCHEMY_AVAILABLE:
    Base = declarative_base()
else:
    Base = type('Base', (), {})

# Initialize engine and session as None - will be created on first use
engine = None
SessionLocal = None

def init_database():
    """Initialize database connection."""
    global engine, SessionLocal
    if not SQLALCHEMY_AVAILABLE:
        logger.info("Running in no-database mode (SQLAlchemy not installed)")
        from unittest.mock import MagicMock
        SessionLocal = MagicMock
        return
    
    if engine is None:
        try:
            # Handle both postgresql:// and postgresql+psycopg:// URLs
            db_url = settings.database_url
            # Convert postgresql:// to postgresql+psycopg:// for psycopg 3.x
            if db_url.startswith("postgresql://") and "+psycopg" not in db_url and "+psycopg2" not in db_url:
                # Replace postgresql:// with postgresql+psycopg:// to use psycopg 3.x driver
                db_url = db_url.replace("postgresql://", "postgresql+psycopg://", 1)
                logger.info("Converted database URL to use psycopg 3.x driver")
            engine = create_engine(
                db_url,
                pool_pre_ping=True,
                echo=settings.debug
            )
            SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
            logger.info("Database connection initialized")
        except Exception as e:
            logger.warning(f"Database connection failed: {e}. Running in no-database mode.")
            # Create a mock session for development
            from unittest.mock import MagicMock
            SessionLocal = MagicMock

def get_db():
    """Dependency for getting database session."""
    global SessionLocal
    if SessionLocal is None:
        init_database()
    
    # If database is not available, return a mock session
    if SessionLocal is None or hasattr(SessionLocal, '__call__') and 'MagicMock' in str(type(SessionLocal)):
        from unittest.mock import MagicMock
        db = MagicMock()
        yield db
        return
    
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Initialize on import
init_database()

