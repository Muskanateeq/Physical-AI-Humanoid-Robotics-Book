# Authentication System Data Model

## Overview
This document defines the data models for the authentication system, following the patterns established in the database-auth skill. The system uses PostgreSQL with SQLModel for type-safe database operations.

## Database Schema

### User Table
The primary user table follows the database-auth skill patterns with proper constraints and indexing:

```sql
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
CREATE INDEX idx_user_active ON users(is_active);
```

## SQLModel Implementation

### User Model (Backend)
```python
from sqlmodel import SQLModel, Field
from datetime import datetime
from typing import Optional
from sqlalchemy import Column, DateTime

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

## Database Configuration

### Connection Pooling
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

### Session Management
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

## API Request/Response Models

### Frontend Schemas (Following frontend-auth patterns)
```javascript
// User creation request
const UserCreateRequest = {
  username: String,  // Required, unique
  email: String,     // Required, unique, valid email format
  password: String   // Required, min 8 chars with complexity
};

// Authentication response
const AuthResponse = {
  access_token: String,  // JWT token
  token_type: String     // "bearer"
};

// User profile response
const UserProfile = {
  id: Number,
  username: String,
  email: String,
  created_at: String,    // ISO date string
  last_login: String     // ISO date string or null
};
```

## Data Validation Rules

### Backend Validation (Pydantic models)
- Username: 3-50 characters, alphanumeric + underscore/hyphen
- Email: Valid email format, max 100 characters
- Password: Min 8 characters, must include uppercase, lowercase, number, and special character
- Unique constraints: username and email must be unique across all users
- Active status: defaults to true, can be set to false for account deactivation

### Frontend Validation
- Real-time validation feedback
- Password strength indicators
- Email format validation
- Username availability checking
- Form submission disabled until all validations pass

## Security Considerations

### Password Storage
- Passwords are never stored in plain text
- bcrypt hashing with 12 rounds of salting
- Hashed passwords stored in VARCHAR(255) field
- Passwords are hashed before database insertion

### Field Constraints
- Username and email have unique constraints to prevent duplicates
- All string fields have max length constraints
- Email field validates format at application level
- Created_at field is automatically set on creation

### Indexing Strategy
- Email and username fields are indexed for fast lookups
- Active status field is indexed for filtering active users
- Composite indexes created for common query patterns
- Indexes are optimized for authentication queries

## Migration Strategy

### Initial Migration
```python
# migrations/versions/001_initial_user_table.py
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

def downgrade():
    """Drop user table"""
    op.drop_table('user')
```

## Performance Optimization

### Query Optimization
- Use select() for queries instead of raw SQL
- Implement proper filtering and pagination
- Leverage SQLModel's relationship patterns for future extensions
- Use connection pooling for database operations

### Index Optimization
- Primary key index on id field
- Unique indexes on username and email
- Additional indexes on frequently queried fields
- Regular monitoring of query performance

## Error Handling

### Database Errors
- Handle unique constraint violations gracefully
- Manage connection pool exhaustion
- Handle database connection failures
- Implement proper rollback mechanisms

### Validation Errors
- Provide clear error messages to users
- Log validation failures for monitoring
- Prevent invalid data from reaching the database
- Implement rate limiting for validation attempts