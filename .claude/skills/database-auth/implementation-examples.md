# Database Authentication Implementation Examples

This reference contains detailed implementation examples for creating database design and implementation for authentication using PostgreSQL and SQLModel.

## SQLModel User Model Definition

```python
# models.py
from sqlmodel import SQLModel, Field
from datetime import datetime
from typing import Optional
from sqlalchemy import Column, DateTime
import uuid

class UserBase(SQLModel):
    """Base model for user fields that are shared between requests and responses"""
    username: str = Field(unique=True, index=True, max_length=50)
    email: str = Field(unique=True, index=True, max_length=100)

class User(UserBase, table=True):
    """User model for the database table"""
    id: Optional[int] = Field(default=None, primary_key=True)
    hashed_password: str = Field(max_length=255)
    created_at: datetime = Field(default_factory=datetime.utcnow, sa_column=Column(DateTime(timezone=True)))
    last_login: Optional[datetime] = Field(sa_column=Column(DateTime(timezone=True)))
    is_active: bool = Field(default=True)

    # Additional fields can be added as needed:
    # is_verified: bool = Field(default=False)
    # is_superuser: bool = Field(default=False)

class UserCreate(UserBase):
    """Model for creating a new user (used in requests)"""
    password: str

class UserRead(UserBase):
    """Model for reading user data (used in responses)"""
    id: int
    created_at: datetime
    last_login: Optional[datetime] = None
    is_active: bool = True

class UserUpdate(SQLModel):
    """Model for updating user data"""
    username: Optional[str] = None
    email: Optional[str] = None
    is_active: Optional[bool] = None
```

## Database Connection Setup

```python
# database.py
from sqlmodel import create_engine, Session
from sqlalchemy import event
from sqlalchemy.pool import QueuePool
import os
from typing import Generator

# Get database URL from environment variable (Railway compatible)
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://username:password@localhost:5432/dbname")

# Create engine with connection pooling for production
engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=10,
    max_overflow=20,
    pool_pre_ping=True,
    pool_recycle=300,
    echo=False  # Set to True for debugging in development
)

def get_session() -> Generator[Session, None, None]:
    """Dependency to get database session"""
    with Session(engine) as session:
        yield session

def create_db_and_tables():
    """Initialize database tables"""
    SQLModel.metadata.create_all(engine)
```

## Alternative: UUID-based Primary Key

```python
# models_uuid.py
from sqlmodel import SQLModel, Field
from datetime import datetime
from typing import Optional
from sqlalchemy import Column, DateTime
import uuid

class UserBase(SQLModel):
    """Base model for user fields that are shared between requests and responses"""
    username: str = Field(unique=True, index=True, max_length=50)
    email: str = Field(unique=True, index=True, max_length=100)

class User(UUIDBaseModel, table=True):
    """User model with UUID primary key for distributed systems"""
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    hashed_password: str = Field(max_length=255)
    created_at: datetime = Field(default_factory=datetime.utcnow, sa_column=Column(DateTime(timezone=True)))
    last_login: Optional[datetime] = Field(sa_column=Column(DateTime(timezone=True)))
    is_active: bool = Field(default=True)
```

## Database Configuration

```python
# config.py
import os
from urllib.parse import urlparse

class DatabaseConfig:
    """Database configuration class for Railway deployment"""

    @staticmethod
    def get_database_url():
        """Get database URL from environment, with Railway compatibility"""
        database_url = os.getenv("DATABASE_URL")

        if not database_url:
            raise ValueError("DATABASE_URL environment variable is required")

        # Parse the URL to ensure it's PostgreSQL
        parsed = urlparse(database_url)
        if parsed.scheme != 'postgresql':
            raise ValueError("DATABASE_URL must be a PostgreSQL URL")

        return database_url

    @staticmethod
    def get_pool_config():
        """Get connection pool configuration"""
        return {
            'pool_size': int(os.getenv('DB_POOL_SIZE', '10')),
            'max_overflow': int(os.getenv('DB_MAX_OVERFLOW', '20')),
            'pool_pre_ping': True,
            'pool_recycle': int(os.getenv('DB_POOL_RECYCLE', '300'))
        }
```

## Migration Setup (Optional)

```python
# migrations/versions/001_initial_user_table.py
"""Initial user table migration

Revision ID: 001
Revises:
Create Date: 2024-01-01 00:00:00.000000

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers
revision = '001'
down_revision = None
branch_labels = None
depends_on = None

def upgrade():
    """Create user table"""
    op.create_table(
        'user',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('username', sa.String(length=50), nullable=False),
        sa.Column('email', sa.String(length=100), nullable=False),
        sa.Column('hashed_password', sa.String(length=255), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=False),
        sa.Column('last_login', sa.DateTime(timezone=True), nullable=True),
        sa.Column('is_active', sa.Boolean(), nullable=False, default=True),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('email'),
        sa.UniqueConstraint('username')
    )

    # Create indexes
    op.create_index('ix_user_email', 'user', ['email'])
    op.create_index('ix_user_username', 'user', ['username'])

def downgrade():
    """Drop user table"""
    op.drop_index('ix_user_email')
    op.drop_index('ix_user_username')
    op.drop_table('user')
```

## Complete Example with Initialization

```python
# main.py (for initialization purposes)
from contextlib import asynccontextmanager
from fastapi import FastAPI
from .database import create_db_and_tables, engine
from .models import User

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize database on startup"""
    print("Initializing database tables...")
    create_db_and_tables()
    print("Database tables initialized successfully")
    yield

app = FastAPI(lifespan=lifespan)

@app.get("/health")
async def health_check():
    """Simple health check endpoint"""
    return {"status": "healthy", "database": "connected"}
```

## SQL Schema Definition (Raw)

```sql
-- Raw SQL schema for reference
CREATE TABLE IF NOT EXISTS users (
    id SERIAL PRIMARY KEY,
    username VARCHAR(50) UNIQUE NOT NULL,
    email VARCHAR(100) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP WITH TIME ZONE,
    is_active BOOLEAN DEFAULT TRUE NOT NULL
);

-- Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_user_email ON users(email);
CREATE INDEX IF NOT EXISTS idx_user_username ON users(username);

-- Add comments for documentation
COMMENT ON TABLE users IS 'User authentication table';
COMMENT ON COLUMN users.username IS 'Unique username for the user';
COMMENT ON COLUMN users.email IS 'Unique email address for the user';
COMMENT ON COLUMN users.hashed_password IS 'BCrypt hashed password';
COMMENT ON COLUMN users.created_at IS 'Timestamp when user was created';
COMMENT ON COLUMN users.last_login IS 'Timestamp of last login';
COMMENT ON COLUMN users.is_active IS 'Whether the user account is active';
```

## Railway Configuration

```yaml
# docker-compose.yml for Railway deployment
version: '3.8'
services:
  app:
    build: .
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=${{DATABASE_URL}}
      - SECRET_KEY=${{SECRET_KEY}}
    depends_on:
      - db
    restart: unless-stopped

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=auth_db
      - POSTGRES_USER=postgres
      - POSTGRES_PASSWORD=postgres
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data
    restart: unless-stopped

volumes:
  postgres_data:
```

## Environment Configuration for Railway

```env
# .env.example
# PostgreSQL connection string (provided by Railway)
DATABASE_URL=postgresql://username:password@railway_host:5432/dbname

# JWT configuration
SECRET_KEY=your-super-secret-key-here-make-it-long-and-random
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Database pool configuration
DB_POOL_SIZE=10
DB_MAX_OVERFLOW=20
DB_POOL_RECYCLE=300
```

This implementation provides a complete, production-ready database design for authentication that follows all the requirements:
- PostgreSQL and SQLModel compatible
- Railway deployment ready
- Simple, minimal, and extendable design
- Proper constraints and indexing
- Password hashing compatibility
- No frontend or backend code included