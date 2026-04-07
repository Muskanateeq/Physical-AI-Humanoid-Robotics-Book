# Python FastAPI Integration References

This document provides references for implementing the backend authentication system with Python and FastAPI using PostgreSQL and SQLModel, including setup instructions, security considerations, and best practices.

## FastAPI Setup and Configuration

### 1. Project Structure

Recommended project structure for the authentication backend:

```
auth-backend/
├── main.py                 # Main application file
├── models.py              # SQLModel database models
├── schemas.py             # Pydantic schemas
├── database.py            # Database connection and setup
├── auth.py                # Authentication utilities
├── dependencies.py        # FastAPI dependencies
├── requirements.txt       # Python dependencies
└── .env                  # Environment variables (not committed)
```

### 2. Requirements.txt

```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
bcrypt==4.0.1
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
python-multipart==0.0.6
pydantic==2.5.0
pydantic-settings==2.1.0
sqlmodel==0.0.16
sqlalchemy==2.0.23
psycopg2-binary==2.9.9
```

### 3. Environment Variables

Create a `.env` file for sensitive configuration:

```env
SECRET_KEY=your-super-secret-key-here-make-it-long-and-random
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
DATABASE_URL=postgresql://username:password@localhost:5432/dbname
```

## Security Best Practices

### 1. Password Security

Always hash passwords before storing them in the database:

```python
from passlib.context import CryptContext

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)
```

### 2. JWT Token Security

- Use a strong secret key (stored in environment variables)
- Set appropriate token expiration times
- Use HTTPS in production
- Consider token blacklisting for logout functionality

### 3. Input Validation

FastAPI automatically validates Pydantic models, but you should also add custom validation:

```python
from pydantic import BaseModel, field_validator
import re

class UserCreate(BaseModel):
    username: str
    email: str
    password: str

    @field_validator('email')
    def validate_email(cls, v):
        if not re.match(r"[^@]+@[^@]+\.[^@]+", v):
            raise ValueError('Invalid email format')
        return v

    @field_validator('password')
    def validate_password(cls, v):
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters')
        return v
```

## Database Integration with PostgreSQL and SQLModel

### 1. SQLModel Setup

SQLModel combines the power of SQLAlchemy and Pydantic:

```python
from sqlmodel import SQLModel, Field, create_engine
from datetime import datetime
from typing import Optional

class UserBase(SQLModel):
    username: str = Field(unique=True, index=True)
    email: str = Field(unique=True, index=True)

class User(UserBase, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    hashed_password: str
    created_at: datetime = Field(default_factory=datetime.utcnow)

class UserCreateInternal(UserBase):
    password: str
```

### 2. PostgreSQL Connection

Setting up PostgreSQL with connection pooling:

```python
from sqlmodel import create_engine
from sqlalchemy import URL

# For Railway deployment
DATABASE_URL = "postgresql://username:password@railway_host:5432/dbname"

# Create engine with connection pooling
engine = create_engine(
    DATABASE_URL,
    pool_size=10,
    max_overflow=20,
    pool_pre_ping=True,
    pool_recycle=300
)
```

### 3. Session Management

Proper session management with dependency injection:

```python
from sqlmodel import Session
from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize database on startup."""
    SQLModel.metadata.create_all(engine)
    yield

def get_session():
    """Get database session."""
    with Session(engine) as session:
        yield session
```

## FastAPI Dependencies and Middleware

### 1. Authentication Dependency

```python
from fastapi import HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

security = HTTPBearer()

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    session: Session = Depends(get_session)
):
    token = credentials.credentials
    email = verify_token(token)

    if email is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    statement = select(User).where(User.email == email)
    user = session.exec(statement).first()

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user
```

### 2. CORS Middleware

For development and production:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## API Documentation and Testing

### 1. API Documentation

FastAPI automatically generates interactive API documentation:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

### 2. Testing with Pytest

Create test files for your authentication endpoints:

```python
# test_auth.py
import pytest
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_signup():
    response = client.post("/signup", json={
        "username": "testuser",
        "email": "test@example.com",
        "password": "testpassword123"
    })
    assert response.status_code == 201

def test_signin():
    # First signup a user
    client.post("/signup", json={
        "username": "testuser2",
        "email": "test2@example.com",
        "password": "testpassword123"
    })

    # Then try to sign in
    response = client.post("/signin", json={
        "email": "test2@example.com",
        "password": "testpassword123"
    })
    assert response.status_code == 200
    assert "access_token" in response.json()
```

## Production Deployment on Railway

### 1. Railway Configuration

For deploying on Railway, create a `Procfile`:

```
web: uvicorn main:app --host=0.0.0.0 --port=${PORT:-5000}
```

### 2. Environment Variables on Railway

Set these environment variables in your Railway dashboard:
- `SECRET_KEY`: Your JWT secret key
- `DATABASE_URL`: PostgreSQL connection string from Railway's database plugin

### 3. Dockerfile for Railway

```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

## Error Handling

### 1. Custom HTTP Exceptions

Define custom error responses:

```python
from fastapi import FastAPI, HTTPException, status

@app.post("/signup")
async def signup(user_data: UserCreate, session: Session = Depends(get_session)):
    statement = select(User).where(User.email == user_data.email)
    existing_user = session.exec(statement).first()

    if existing_user:
        raise HTTPException
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    user_create_internal = UserCreateInternal(
        username=user_data.username,
        email=user_data.email,
        password=user_data.password
    )

    user = create_user(session, user_create_internal)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username already taken"
        )

    return user
```

### 2. Global Exception Handler

Handle unexpected errors gracefully:

```python
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc):
    return JSONResponse(
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        content={"detail": exc.errors()}
    )
```

## Performance Considerations

### 1. Database Connection Pooling

SQLAlchemy provides built-in connection pooling:

```python
from sqlalchemy import create_engine

engine = create_engine(
    DATABASE_URL,
    pool_size=20,
    max_overflow=40,
    pool_pre_ping=True,
    pool_recycle=300,
    echo=False  # Set to True for debugging
)
```

### 2. Indexing Strategy

For optimal PostgreSQL performance, ensure proper indexing:

```python
class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    username: str = Field(unique=True, index=True)  # Index for unique constraint
    email: str = Field(unique=True, index=True)     # Index for unique constraint
    hashed_password: str
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

## PostgreSQL-Specific Features

### 1. UUID Primary Keys (Optional)

For enhanced security and distributed systems:

```python
from uuid import UUID, uuid4
from sqlmodel import Field

class User(SQLModel, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    username: str = Field(unique=True, index=True)
    email: str = Field(unique=True, index=True)
    hashed_password: str
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

### 2. JSON Fields (If needed)

For flexible data storage:

```python
from sqlalchemy import JSON

class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    username: str = Field(unique=True, index=True)
    email: str = Field(unique=True, index=True)
    hashed_password: str
    profile: dict = Field(default={}, sa_column=Column(JSON))  # For additional user data
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

This reference guide provides comprehensive information for implementing and deploying a secure, efficient authentication backend using Python, FastAPI, PostgreSQL, and SQLModel, specifically optimized for Railway deployment.