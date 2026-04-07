"""
Database Connection Module

Manages database connections and sessions using SQLModel and asyncpg.
Provides session management for FastAPI dependency injection.
Compatible with Neon PostgreSQL.
"""

from sqlmodel import SQLModel, create_engine
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
import ssl
import os

# Get database URL from environment
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://user:password@localhost:5432/neondb")

# Create synchronous engine for migrations (if needed)
sync_engine = create_engine(
    DATABASE_URL,
    echo=os.getenv("ENVIRONMENT", "development") == "development",
    pool_pre_ping=True,
    pool_size=5,
    max_overflow=10,
)

# Create async engine for FastAPI application
# Convert postgresql:// to postgresql+asyncpg://
async_database_url = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://")

# Remove sslmode parameter if present (asyncpg doesn't support it in URL)
if "sslmode=" in async_database_url:
    # Remove sslmode parameter
    async_database_url = async_database_url.split("?")[0]

# Create SSL context for asyncpg (Neon requires SSL)
ssl_context = ssl.create_default_context()
ssl_context.check_hostname = False
ssl_context.verify_mode = ssl.CERT_NONE

async_engine = create_async_engine(
    async_database_url,
    echo=os.getenv("ENVIRONMENT", "development") == "development",
    pool_pre_ping=True,
    pool_size=int(os.getenv("DB_POOL_SIZE", "10")),
    max_overflow=int(os.getenv("DB_MAX_OVERFLOW", "20")),
    connect_args={
        "ssl": ssl_context,  # asyncpg uses ssl parameter
    }
)

# Create async session factory
AsyncSessionLocal = sessionmaker(
    bind=async_engine,
    class_=AsyncSession,
    expire_on_commit=False,
)


def create_db_and_tables():
    """Create all database tables (for development only)"""
    SQLModel.metadata.create_all(sync_engine)


async def get_session() -> AsyncSession:
    """
    Dependency function to get database session.

    Usage in FastAPI endpoints:
        @app.get("/items")
        async def get_items(session: AsyncSession = Depends(get_session)):
            ...
    """
    async with AsyncSessionLocal() as session:
        yield session
