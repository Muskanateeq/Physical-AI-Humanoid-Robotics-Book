# PostgreSQL SQLModel Integration References

This document provides references for implementing database authentication with PostgreSQL and SQLModel, including setup instructions, best practices, and Railway deployment guidelines.

## PostgreSQL Setup and Configuration

### 1. PostgreSQL Connection Parameters

For Railway deployment, PostgreSQL connection string format:
```
postgresql://username:password@railway_host:port/database_name
```

### 2. Required PostgreSQL Extensions

For authentication system, consider these extensions:
- `uuid-ossp` for UUID generation (if using UUID primary keys)
- `pgcrypto` for cryptographic functions (alternative to application-level hashing)

### 3. Database Initialization

Initialize the database with proper authentication tables:

```sql
-- Create the users table
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    username VARCHAR(50) UNIQUE NOT NULL,
    email VARCHAR(100) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP WITH TIME ZONE,
    is_active BOOLEAN DEFAULT TRUE NOT NULL
);

-- Create indexes for performance
CREATE INDEX idx_user_email ON users(email);
CREATE INDEX idx_user_username ON users(username);
```

## SQLModel Best Practices

### 1. Model Definition Standards

```python
from sqlmodel import SQLModel, Field
from datetime import datetime
from typing import Optional
from sqlalchemy import Column, DateTime

class User(SQLModel, table=True):
    # Primary key
    id: Optional[int] = Field(default=None, primary_key=True)

    # Unique constraints
    username: str = Field(unique=True, index=True, max_length=50)
    email: str = Field(unique=True, index=True, max_length=100)

    # Security fields
    hashed_password: str = Field(max_length=255)

    # Timestamps
    created_at: datetime = Field(default_factory=datetime.utcnow, sa_column=Column(DateTime(timezone=True)))
    last_login: Optional[datetime] = Field(sa_column=Column(DateTime(timezone=True)))

    # Status fields
    is_active: bool = Field(default=True)
```

### 2. Field Constraints and Validation

- Use `max_length` for string fields to prevent oversized data
- Use `unique=True` for fields that must be unique across the table
- Use `index=True` for fields used in queries frequently
- Use `nullable=False` for required fields (default behavior)
- Use `default=` for default values
- Use `sa_column=Column(DateTime(timezone=True))` for timezone-aware datetimes

### 3. Relationship Patterns

For future extensibility, even if no relationships are needed now:

```python
from sqlmodel import Relationship

# Example for when relationships are needed in the future
class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    # ... other fields ...

    # Sessions related to this user (when needed)
    # sessions: List[UserSession] = Relationship(back_populates="user")
```

## Connection Pooling and Performance

### 1. Connection Pool Configuration

```python
from sqlmodel import create_engine
from sqlalchemy.pool import QueuePool

engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=10,           # Number of connections to maintain
    max_overflow=20,        # Additional connections beyond pool_size
    pool_pre_ping=True,     # Verify connections before use
    pool_recycle=300,       # Recycle connections after 5 minutes
    echo=False             # Set to True for SQL logging in development
)
```

### 2. Session Management

```python
from sqlmodel import Session
from contextlib import contextmanager

@contextmanager
def get_db_session():
    """Context manager for database sessions"""
    session = Session(engine)
    try:
        yield session
        session.commit()
    except Exception:
        session.rollback()
        raise
    finally:
        session.close()
```

## Railway Deployment Considerations

### 1. Environment Variables

Set these environment variables in your Railway dashboard:
- `DATABASE_URL`: PostgreSQL connection string from Railway
- `SECRET_KEY`: JWT secret key for authentication
- `DB_POOL_SIZE`: Connection pool size (default: 10)
- `DB_MAX_OVERFLOW`: Max overflow connections (default: 20)

### 2. Railway Configuration File

```yaml
# .railway/config.yml
module: backend-auth
build:
  builder: dockerfile
  dockerfilePath: Dockerfile
  dockerContextPath: .
deploy:
  startCommand: uvicorn main:app --host 0.0.0.0 --port $PORT
  healthcheckPath: /health
  maxRestarts: 3
```

### 3. Dockerfile for Railway

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install system dependencies for PostgreSQL
RUN apt-get update && apt-get install -y \
    gcc \
    postgresql-server-dev-all \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE $PORT

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "$PORT"]
```

## Security Best Practices

### 1. Password Security

- Never store plain text passwords in the database
- Always use a strong hashing algorithm (bcrypt recommended)
- Store only the hashed password in the `hashed_password` field
- Ensure the field is large enough (VARCHAR(255) or TEXT)

### 2. SQL Injection Prevention

- Use SQLModel's parameterized queries
- Never construct SQL strings with string concatenation
- Use ORM methods for all database operations

### 3. Data Validation

- Validate input before database insertion
- Use SQLModel's field constraints
- Implement proper error handling for database operations

## Migration Strategies

### 1. Alembic for Migrations

```python
# alembic.ini
[alembic]
script_location = migrations
sqlalchemy.url = ${DATABASE_URL}

# env.py
from logging.config import fileConfig
from sqlalchemy import engine_from_config
from sqlalchemy import pool
from alembic import context
from app.models import SQLModel

config = context.config
fileConfig(config.config_file_name)
target_metadata = SQLModel.metadata

def run_migrations_online():
    connectable = engine_from_config(
        config.get_section(config.config_ini_section),
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )

    with connectable.connect() as connection:
        context.configure(
            connection=connection, target_metadata=target_metadata
        )

        with context.begin_transaction():
            context.run_migrations()
```

### 2. Manual Table Creation (Simple Approach)

```python
from sqlmodel import SQLModel

def create_db_and_tables():
    """Create all tables defined in SQLModel models"""
    SQLModel.metadata.create_all(engine)
```

## Error Handling

### 1. Database Connection Errors

```python
from sqlalchemy.exc import SQLAlchemyError
from sqlmodel import Session

def handle_database_operation(operation_func, *args, **kwargs):
    """Wrapper for database operations with error handling"""
    try:
        with Session(engine) as session:
            return operation_func(session, *args, **kwargs)
    except SQLAlchemyError as e:
        print(f"Database error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error: {e}")
        raise
```

### 2. Unique Constraint Violations

```python
from sqlalchemy.exc import IntegrityError

def create_user_safe(session, user_data):
    """Safely create a user with unique constraint handling"""
    try:
        user = User.from_orm(user_data)
        session.add(user)
        session.commit()
        session.refresh(user)
        return user
    except IntegrityError:
        session.rollback()
        raise ValueError("Username or email already exists")
```

## Performance Optimization

### 1. Indexing Strategy

```sql
-- Essential indexes for authentication
CREATE INDEX idx_user_email ON users(email);           -- For login queries
CREATE INDEX idx_user_username ON users(username);     -- For username lookups
CREATE INDEX idx_user_active ON users(is_active);      -- For active user queries

-- Composite indexes if needed
CREATE INDEX idx_user_email_active ON users(email, is_active);  -- For active user login
```

### 2. Query Optimization

```python
from sqlmodel import select

# Use select() for queries instead of raw SQL
def get_user_by_email(session: Session, email: str):
    """Get user by email with optimized query"""
    statement = select(User).where(User.email == email).where(User.is_active == True)
    return session.exec(statement).first()
```

This reference guide provides comprehensive information for implementing a secure, efficient, and scalable authentication database using PostgreSQL and SQLModel, optimized for Railway deployment.