# Backend Authentication Implementation Examples

This reference contains detailed implementation examples for creating simple authentication backend using Python and FastAPI.

## Dependencies Setup

```bash
pip install fastapi uvicorn bcrypt python-jose[cryptography] passlib[bcrypt] python-multipart sqlmodel sqlalchemy psycopg2-binary
```

## Main Application Structure

```python
# main.py
from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from typing import Optional
from datetime import datetime, timedelta
import jwt
from passlib.context import CryptContext
from sqlmodel import SQLModel, Field, create_engine, Session, select
from sqlalchemy import Column, DateTime
from enum import Enum
import os
from contextlib import asynccontextmanager

# Database setup
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://username:password@localhost/dbname")
engine = create_engine(DATABASE_URL, echo=True)

# Security setup
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()

# Secret key for JWT (in production, use environment variable)
SECRET_KEY = os.getenv("SECRET_KEY", "your-secret-key-here")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# Pydantic models
class UserCreate(BaseModel):
    username: str
    email: str
    password: str

class UserLogin(BaseModel):
    email: str
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str

class UserResponse(BaseModel):
    id: int
    username: str
    email: str
    created_at: datetime

    class Config:
        from_attributes = True
```

## Password Hashing Utilities

```python
def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against a hashed password."""
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """Generate a hash for a plain password."""
    return pwd_context.hash(password)
```

## Database Models with SQLModel

```python
# SQLModel models for PostgreSQL
from sqlmodel import SQLModel, Field
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

## Database Setup and Session Management

```python
from sqlmodel import Session, create_engine, SQLModel, select
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

## User Database Operations

```python
from sqlmodel import Session, select
from typing import Optional

def get_user_by_email(session: Session, email: str) -> Optional[User]:
    """Retrieve a user by email."""
    statement = select(User).where(User.email == email)
    return session.exec(statement).first()

def create_user(session: Session, user_data: UserCreateInternal) -> Optional[User]:
    """Create a new user in the database."""
    hashed_password = get_password_hash(user_data.password)

    db_user = User(
        username=user_data.username,
        email=user_data.email,
        hashed_password=hashed_password
    )

    try:
        session.add(db_user)
        session.commit()
        session.refresh(db_user)
        return db_user
    except Exception:
        # Rollback in case of error
        session.rollback()
        return None
```

## JWT Token Utilities

```python
def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create a JWT access token."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    """Verify and decode a JWT token."""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            return None
        return email
    except jwt.JWTError:
        return None
```

## Authentication Dependency

```python
from fastapi import BackgroundTasks

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    session: Session = Depends(get_session)
):
    """Dependency to get current user from token."""
    token = credentials.credentials
    email = verify_token(token)

    if email is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user = get_user_by_email(session, email)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user
```

## Signup Endpoint

```python
@app.post("/signup", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    user_data: UserCreate,
    session: Session = Depends(get_session)
):
    """Create a new user account."""
    # Check if user already exists
    existing_user = get_user_by_email(session, user_data.email)
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Create new user
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

    # Return user info without password
    return UserResponse(
        id=user.id,
        username=user.username,
        email=user.email,
        created_at=user.created_at
    )
```

## Signin Endpoint

```python
@app.post("/signin", response_model=Token)
async def signin(
    user_data: UserLogin,
    session: Session = Depends(get_session)
):
    """Authenticate user and return access token."""
    user = get_user_by_email(session, user_data.email)

    if not user or not verify_password(user_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Create access token
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.email}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}
```

## Protected Route Example

```python
@app.get("/me", response_model=UserResponse)
async def get_current_user_info(
    current_user: User = Depends(get_current_user)
):
    """Get current user info (protected route)."""
    return UserResponse(
        id=current_user.id,
        username=current_user.username,
        email=current_user.email,
        created_at=current_user.created_at
    )
```

## Logout Endpoint (Optional)

```python
@app.post("/logout")
async def logout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Logout user (token invalidation would require a blacklist in production)."""
    # In a simple implementation, we just confirm the user was logged in
    # For true logout, you'd need a token blacklist system
    return {"message": "Successfully logged out"}
```

## Complete Example with Error Handling

```python
# Complete main.py file
from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from typing import Optional
from datetime import datetime, timedelta
import jwt
from passlib.context import CryptContext
from sqlmodel import SQLModel, Field, create_engine, Session, select
from contextlib import asynccontextmanager
import os

# Database setup
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://username:password@localhost/dbname")
engine = create_engine(DATABASE_URL, echo=True)

# Security setup
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()

# Secret key for JWT (in production, use environment variable)
SECRET_KEY = os.getenv("SECRET_KEY", "your-secret-key-here")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# Pydantic models
class UserCreate(BaseModel):
    username: str
    email: str
    password: str

class UserLogin(BaseModel):
    email: str
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str

class UserResponse(BaseModel):
    id: int
    username: str
    email: str
    created_at: datetime

    class Config:
        from_attributes = True

# SQLModel models for PostgreSQL
class UserBase(SQLModel):
    username: str = Field(unique=True, index=True)
    email: str = Field(unique=True, index=True)

class User(UserBase, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    hashed_password: str
    created_at: datetime = Field(default_factory=datetime.utcnow)

class UserCreateInternal(UserBase):
    password: str

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against a hashed password."""
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """Generate a hash for a plain password."""
    return pwd_context.hash(password)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize database on startup."""
    SQLModel.metadata.create_all(engine)
    yield

def get_session():
    """Get database session."""
    with Session(engine) as session:
        yield session

def get_user_by_email(session: Session, email: str):
    """Retrieve a user by email."""
    statement = select(User).where(User.email == email)
    return session.exec(statement).first()

def create_user(session: Session, user_data: UserCreateInternal):
    """Create a new user in the database."""
    hashed_password = get_password_hash(user_data.password)

    db_user = User(
        username=user_data.username,
        email=user_data.email,
        hashed_password=hashed_password
    )

    try:
        session.add(db_user)
        session.commit()
        session.refresh(db_user)
        return db_user
    except Exception:
        # Rollback in case of error
        session.rollback()
        return None

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create a JWT access token."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    """Verify and decode a JWT token."""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            return None
        return email
    except jwt.JWTError:
        return None

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    session: Session = Depends(get_session)
):
    """Dependency to get current user from token."""
    token = credentials.credentials
    email = verify_token(token)

    if email is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user = get_user_by_email(session, email)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user

app = FastAPI(title="Simple Authentication API", lifespan=lifespan)

@app.post("/signup", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    user_data: UserCreate,
    session: Session = Depends(get_session)
):
    """Create a new user account."""
    # Check if user already exists
    existing_user = get_user_by_email(session, user_data.email)
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Create new user
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

    # Return user info without password
    return UserResponse(
        id=user.id,
        username=user.username,
        email=user.email,
        created_at=user.created_at
    )

@app.post("/signin", response_model=Token)
async def signin(
    user_data: UserLogin,
    session: Session = Depends(get_session)
):
    """Authenticate user and return access token."""
    user = get_user_by_email(session, user_data.email)

    if not user or not verify_password(user_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Create access token
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.email}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}

@app.get("/me", response_model=UserResponse)
async def get_current_user_info(
    current_user: User = Depends(get_current_user)
):
    """Get current user info (protected route)."""
    return UserResponse(
        id=current_user.id,
        username=current_user.username,
        email=current_user.email,
        created_at=current_user.created_at
    )

@app.post("/logout")
async def logout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Logout user (token invalidation would require a blacklist in production)."""
    # In a simple implementation, we just confirm the user was logged in
    # For true logout, you'd need a token blacklist system
    return {"message": "Successfully logged out"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

## Running the Application

To run the authentication API:

1. Install dependencies:
```bash
pip install fastapi uvicorn bcrypt python-jose[cryptography] passlib[bcrypt] python-multipart sqlmodel sqlalchemy psycopg2-binary
```

2. Set up your PostgreSQL database (using Railway or local setup)

3. Set the DATABASE_URL environment variable:
```bash
export DATABASE_URL="postgresql://username:password@localhost/dbname"
```

4. Run the application:
```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000` with the following endpoints:
- `POST /signup` - Create a new user
- `POST /signin` - Authenticate a user
- `GET /me` - Get current user info (requires valid token)
- `POST /logout` - Logout user (optional)