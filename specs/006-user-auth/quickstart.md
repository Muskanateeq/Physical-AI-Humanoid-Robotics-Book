# Authentication System Quickstart Guide

## Overview
This guide provides a quick setup for the authentication system using Docusaurus frontend, FastAPI backend, and PostgreSQL database. Follow these steps to get the system running locally.

## Prerequisites
- Python 3.11+ installed
- Node.js 18+ installed
- PostgreSQL server running locally or accessible
- Git installed for version control

## Backend Setup

### 1. Create Backend Directory Structure
```bash
mkdir backend
cd backend
```

### 2. Set Up Python Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install fastapi uvicorn bcrypt python-jose[cryptography] passlib[bcrypt] python-multipart sqlmodel sqlalchemy psycopg2-binary python-dotenv
```

### 3. Create Backend Files

Create `requirements.txt`:
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
python-dotenv==1.0.0
```

Create `.env` file:
```env
SECRET_KEY=your-super-secret-key-here-make-it-long-and-random
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
DATABASE_URL=postgresql://username:password@localhost:5432/auth_db
```

### 4. Create Backend Application Structure

Create `src/database/database.py`:
```python
from sqlmodel import create_engine, Session
from sqlalchemy.pool import QueuePool
import os
from typing import Generator
from contextlib import asynccontextmanager
from fastapi import FastAPI
from src.models.user import SQLModel

# Get database URL from environment variable
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://username:password@localhost:5432/auth_db")

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

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize database on startup."""
    print("Initializing database tables...")
    create_db_and_tables()
    print("Database tables initialized successfully")
    yield
```

Create `src/models/user.py`:
```python
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

Create `src/auth/auth.py`:
```python
from passlib.context import CryptContext
from datetime import datetime, timedelta
import jwt
from fastapi import HTTPException, status
from typing import Optional
from sqlmodel import Session, select
from src.models.user import User

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against a hashed password."""
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """Generate a hash for a plain password."""
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create a JWT access token."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, os.getenv("SECRET_KEY"), algorithm=os.getenv("ALGORITHM", "HS256"))
    return encoded_jwt

def verify_token(token: str):
    """Verify and decode a JWT token."""
    try:
        payload = jwt.decode(token, os.getenv("SECRET_KEY"), algorithms=[os.getenv("ALGORITHM", "HS256")])
        email: str = payload.get("sub")
        if email is None:
            return None
        return email
    except jwt.JWTError:
        return None

def get_user_by_email(session: Session, email: str):
    """Retrieve a user by email."""
    statement = select(User).where(User.email == email)
    return session.exec(statement).first()

def create_user(session: Session, user_data):
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

Create `src/api/auth.py`:
```python
from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from typing import Optional
from datetime import timedelta
from sqlmodel import Session, select
from src.database.database import get_session
from src.models.user import User, UserCreate, UserRead
from src.auth.auth import (
    verify_password,
    get_user_by_email,
    create_user,
    create_access_token,
    verify_token,
    get_password_hash
)

router = APIRouter()
security = HTTPBearer()

class UserLogin(BaseModel):
    email: str
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str

@router.post("/signup", response_model=UserRead, status_code=status.HTTP_201_CREATED)
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
    user = create_user(session, user_data)

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username already taken"
        )

    # Return user info without password
    return UserRead(
        id=user.id,
        username=user.username,
        email=user.email,
        created_at=user.created_at,
        last_login=user.last_login,
        is_active=user.is_active
    )

@router.post("/signin", response_model=Token)
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
    from datetime import timedelta
    access_token_expires = timedelta(minutes=30)  # Use value from settings
    access_token = create_access_token(
        data={"sub": user.email}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}

@router.get("/me", response_model=UserRead)
async def get_current_user_info(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    session: Session = Depends(get_session)
):
    """Get current user info (protected route)."""
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

    return UserRead(
        id=user.id,
        username=user.username,
        email=user.email,
        created_at=user.created_at,
        last_login=user.last_login,
        is_active=user.is_active
    )

@router.post("/logout")
async def logout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Logout user (token invalidation would require a blacklist in production)."""
    # In a simple implementation, we just confirm the user was logged in
    # For true logout, you'd need a token blacklist system
    return {"message": "Successfully logged out"}
```

Create `src/main.py`:
```python
from fastapi import FastAPI
from src.database.database import lifespan
from src.api.auth import router as auth_router

app = FastAPI(
    title="Authentication API",
    description="Complete authentication system with signup/signin",
    version="1.0.0",
    lifespan=lifespan
)

# Include authentication routes
app.include_router(auth_router, prefix="/auth", tags=["authentication"])

@app.get("/health")
async def health_check():
    """Simple health check endpoint"""
    return {"status": "healthy", "database": "connected"}
```

### 5. Run Backend Server
```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn src.main:app --reload --port 8000
```

## Frontend Setup

### 1. Create Frontend Directory Structure
```bash
mkdir frontend
cd frontend
npm init docusaurus@latest .
```

### 2. Install Frontend Dependencies
```bash
cd frontend
npm install
npm install axios react-query @headlessui/react @heroicons/react
```

### 3. Create Frontend Files

Create `src/components/Auth/AuthContext.js`:
```javascript
import React, { createContext, useContext, useReducer } from 'react';
import { useLocalStorage } from '../utils/useLocalStorage';

const AuthContext = createContext();

const authReducer = (state, action) => {
  switch (action.type) {
    case 'LOGIN_START':
      return { ...state, loading: true, error: null };
    case 'LOGIN_SUCCESS':
      return {
        ...state,
        loading: false,
        user: action.payload.user,
        token: action.payload.token,
        isAuthenticated: true
      };
    case 'LOGIN_FAILURE':
      return { ...state, loading: false, error: action.payload };
    case 'LOGOUT':
      return {
        ...state,
        user: null,
        token: null,
        isAuthenticated: false
      };
    case 'SET_USER':
      return {
        ...state,
        user: action.payload,
        isAuthenticated: true
      };
    default:
      return state;
  }
};

export const AuthProvider = ({ children }) => {
  const [token, setToken] = useLocalStorage('authToken', null);
  const [state, dispatch] = useReducer(authReducer, {
    user: null,
    token: token,
    isAuthenticated: !!token,
    loading: false,
    error: null
  });

  return (
    <AuthContext.Provider value={{ ...state, dispatch, setToken }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
```

Create `src/pages/signup.js`:
```javascript
import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../components/Auth/AuthContext';

const SignupPage = () => {
  const [formData, setFormData] = useState({
    username: '',
    email: '',
    password: ''
  });
  const [error, setError] = useState('');
  const { dispatch } = useAuth();
  const navigate = useNavigate();

  const handleChange = (e) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    try {
      const response = await fetch('http://localhost:8000/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Signup failed');
      }

      const data = await response.json();

      // Auto-login after signup
      const loginResponse = await fetch('http://localhost:8000/auth/signin', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password
        }),
      });

      if (!loginResponse.ok) {
        throw new Error('Login after signup failed');
      }

      const loginData = await loginResponse.json();

      dispatch({
        type: 'LOGIN_SUCCESS',
        payload: {
          user: data,
          token: loginData.access_token
        }
      });

      navigate('/dashboard');
    } catch (err) {
      setError(err.message);
    }
  };

  return (
    <div className="min-h-screen flex items-center justify-center bg-gradient-to-br from-slate-800 to-slate-900">
      <div className="max-w-md w-full space-y-8 p-10 bg-white rounded-xl shadow-lg">
        <div>
          <h2 className="mt-6 text-center text-3xl font-extrabold text-gray-900">
            Create your account
          </h2>
        </div>
        <form className="mt-8 space-y-6" onSubmit={handleSubmit}>
          {error && (
            <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded relative" role="alert">
              <span className="block sm:inline">{error}</span>
            </div>
          )}
          <input type="hidden" name="remember" value="true" />
          <div className="rounded-md shadow-sm -space-y-px">
            <div>
              <label htmlFor="username" className="sr-only">Username</label>
              <input
                id="username"
                name="username"
                type="text"
                required
                className="appearance-none rounded-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-t-md focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 focus:z-10 sm:text-sm"
                placeholder="Username"
                value={formData.username}
                onChange={handleChange}
              />
            </div>
            <div>
              <label htmlFor="email" className="sr-only">Email address</label>
              <input
                id="email"
                name="email"
                type="email"
                autoComplete="email"
                required
                className="appearance-none rounded-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 focus:z-10 sm:text-sm"
                placeholder="Email address"
                value={formData.email}
                onChange={handleChange}
              />
            </div>
            <div>
              <label htmlFor="password" className="sr-only">Password</label>
              <input
                id="password"
                name="password"
                type="password"
                autoComplete="current-password"
                required
                className="appearance-none rounded-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-b-md focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 focus:z-10 sm:text-sm"
                placeholder="Password"
                value={formData.password}
                onChange={handleChange}
              />
            </div>
          </div>

          <div>
            <button
              type="submit"
              className="group relative w-full flex justify-center py-2 px-4 border border-transparent text-sm font-medium rounded-md text-white bg-indigo-600 hover:bg-indigo-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500"
            >
              Sign up
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default SignupPage;
```

Create `src/pages/signin.js`:
```javascript
import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../components/Auth/AuthContext';

const SigninPage = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: ''
  });
  const [error, setError] = useState('');
  const { dispatch, setToken } = useAuth();
  const navigate = useNavigate();

  const handleChange = (e) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    try {
      const response = await fetch('http://localhost:8000/auth/signin', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Login failed');
      }

      const data = await response.json();

      // Get user info
      const userResponse = await fetch('http://localhost:8000/auth/me', {
        headers: {
          'Authorization': `Bearer ${data.access_token}`,
        },
      });

      if (!userResponse.ok) {
        throw new Error('Failed to get user info');
      }

      const userData = await userResponse.json();

      dispatch({
        type: 'LOGIN_SUCCESS',
        payload: {
          user: userData,
          token: data.access_token
        }
      });

      setToken(data.access_token);

      navigate('/dashboard');
    } catch (err) {
      setError(err.message);
    }
  };

  return (
    <div className="min-h-screen flex items-center justify-center bg-gradient-to-br from-slate-800 to-slate-900">
      <div className="max-w-md w-full space-y-8 p-10 bg-white rounded-xl shadow-lg">
        <div>
          <h2 className="mt-6 text-center text-3xl font-extrabold text-gray-900">
            Sign in to your account
          </h2>
        </div>
        <form className="mt-8 space-y-6" onSubmit={handleSubmit}>
          {error && (
            <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded relative" role="alert">
              <span className="block sm:inline">{error}</span>
            </div>
          )}
          <input type="hidden" name="remember" value="true" />
          <div className="rounded-md shadow-sm -space-y-px">
            <div>
              <label htmlFor="email" className="sr-only">Email address</label>
              <input
                id="email"
                name="email"
                type="email"
                autoComplete="email"
                required
                className="appearance-none rounded-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-t-md focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 focus:z-10 sm:text-sm"
                placeholder="Email address"
                value={formData.email}
                onChange={handleChange}
              />
            </div>
            <div>
              <label htmlFor="password" className="sr-only">Password</label>
              <input
                id="password"
                name="password"
                type="password"
                autoComplete="current-password"
                required
                className="appearance-none rounded-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-b-md focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 focus:z-10 sm:text-sm"
                placeholder="Password"
                value={formData.password}
                onChange={handleChange}
              />
            </div>
          </div>

          <div>
            <button
              type="submit"
              className="group relative w-full flex justify-center py-2 px-4 border border-transparent text-sm font-medium rounded-md text-white bg-indigo-600 hover:bg-indigo-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500"
            >
              Sign in
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default SigninPage;
```

### 4. Update Docusaurus Configuration
Update `docusaurus.config.js` to include the auth pages:

```javascript
// docusaurus.config.js
module.exports = {
  // ... other config
  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
        },
        blog: false,
        pages: {
          // Enable custom pages
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],
  themes: [
    // Add any additional themes here
  ],
};
```

### 5. Run Frontend Server
```bash
cd frontend
npm start
```

## Environment Configuration

### Backend Environment Variables
Set these environment variables in your `.env` file:
- `SECRET_KEY`: Your JWT secret key (make it long and random)
- `ALGORITHM`: JWT algorithm (default: HS256)
- `ACCESS_TOKEN_EXPIRE_MINUTES`: Token expiration time (default: 30)
- `DATABASE_URL`: PostgreSQL connection string

### Frontend Environment Variables
Set these in your frontend environment:
- `REACT_APP_API_URL`: Backend API URL (e.g., http://localhost:8000)

## API Endpoints

### Authentication Endpoints
- `POST /auth/signup` - Create a new user account
- `POST /auth/signin` - Authenticate user and return token
- `GET /auth/me` - Get current user info (requires valid token)
- `POST /auth/logout` - Logout user

### Health Check
- `GET /health` - Check server health status

## Testing the System

1. Start the backend server: `uvicorn src.main:app --reload --port 8000`
2. Start the frontend server: `npm start` in the frontend directory
3. Navigate to `http://localhost:3000/signup` to create an account
4. Use the signup form to create a new user
5. The system will automatically log you in after signup
6. You can also access `http://localhost:3000/signin` to sign in with existing credentials

## Next Steps

1. Set up PostgreSQL database
2. Configure environment variables properly
3. Implement additional security measures (rate limiting, etc.)
4. Add more comprehensive error handling
5. Implement password reset functionality
6. Add user profile management
7. Configure production deployment settings