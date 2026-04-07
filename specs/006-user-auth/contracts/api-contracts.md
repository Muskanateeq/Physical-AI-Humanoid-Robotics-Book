# Authentication System Contracts

## Overview
This document defines the API contracts, data contracts, and interface contracts for the authentication system. These contracts ensure consistency between frontend, backend, and database layers.

## API Contracts

### Authentication API Endpoints

#### POST /auth/signup
**Purpose**: Create a new user account

**Request**:
- Method: POST
- Path: `/auth/signup`
- Content-Type: `application/json`
- Headers: `Content-Type: application/json`

**Request Body**:
```json
{
  "username": "string (3-50 chars, alphanumeric + underscore/hyphen)",
  "email": "string (valid email format, max 100 chars)",
  "password": "string (min 8 chars with complexity)"
}
```

**Success Response** (201 Created):
```json
{
  "id": "integer",
  "username": "string",
  "email": "string",
  "created_at": "ISO 8601 datetime string",
  "last_login": "ISO 8601 datetime string or null",
  "is_active": "boolean"
}
```

**Error Responses**:
- 400 Bad Request: Email already registered or username already taken
- 422 Unprocessable Entity: Validation errors

#### POST /auth/signin
**Purpose**: Authenticate user and return access token

**Request**:
- Method: POST
- Path: `/auth/signin`
- Content-Type: `application/json`
- Headers: `Content-Type: application/json`

**Request Body**:
```json
{
  "email": "string (valid email format)",
  "password": "string"
}
```

**Success Response** (200 OK):
```json
{
  "access_token": "string (JWT token)",
  "token_type": "string (always 'bearer')"
}
```

**Error Responses**:
- 401 Unauthorized: Incorrect email or password
- 422 Unprocessable Entity: Validation errors

#### GET /auth/me
**Purpose**: Get current user info (protected route)

**Request**:
- Method: GET
- Path: `/auth/me`
- Headers: `Authorization: Bearer {token}`

**Success Response** (200 OK):
```json
{
  "id": "integer",
  "username": "string",
  "email": "string",
  "created_at": "ISO 8601 datetime string",
  "last_login": "ISO 8601 datetime string or null",
  "is_active": "boolean"
}
```

**Error Responses**:
- 401 Unauthorized: Invalid or expired token
- 404 Not Found: User no longer exists

#### POST /auth/logout
**Purpose**: Logout user (optional endpoint)

**Request**:
- Method: POST
- Path: `/auth/logout`
- Headers: `Authorization: Bearer {token}`

**Success Response** (200 OK):
```json
{
  "message": "Successfully logged out"
}
```

**Error Responses**:
- 401 Unauthorized: Invalid or expired token

#### GET /health
**Purpose**: Health check endpoint

**Request**:
- Method: GET
- Path: `/health`

**Success Response** (200 OK):
```json
{
  "status": "healthy",
  "database": "connected"
}
```

## Data Contracts

### Database Schema Contracts

#### users Table
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

-- Indexes
CREATE INDEX idx_user_email ON users(email);
CREATE INDEX idx_user_username ON users(username);
CREATE INDEX idx_user_active ON users(is_active);
```

### Backend Model Contracts (SQLModel)

#### User Model
```python
class UserBase(SQLModel):
    username: str = Field(unique=True, index=True, max_length=50)
    email: str = Field(unique=True, index=True, max_length=100)

class User(UserBase, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    hashed_password: str = Field(max_length=255)
    created_at: datetime = Field(default_factory=datetime.utcnow, sa_column=Column(DateTime(timezone=True)))
    last_login: Optional[datetime] = Field(sa_column=Column(DateTime(timezone=True)))
    is_active: bool = Field(default=True)

class UserCreate(UserBase):
    password: str

class UserRead(UserBase):
    id: int
    created_at: datetime
    last_login: Optional[datetime] = None
    is_active: bool = True

class UserUpdate(SQLModel):
    username: Optional[str] = None
    email: Optional[str] = None
    is_active: Optional[bool] = None
```

### Frontend Model Contracts (JavaScript)

#### User Objects
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

## Interface Contracts

### Frontend-Backend Interface

#### Authentication API Client Interface
```javascript
interface AuthAPIClient {
  signup(userData: UserCreateRequest): Promise<UserProfile>
  signin(credentials: { email: string, password: string }): Promise<AuthResponse>
  getCurrentUser(token: string): Promise<UserProfile>
  logout(token: string): Promise<{ message: string }>
}
```

#### Auth Context Interface
```javascript
interface AuthContextType {
  user: UserProfile | null
  token: string | null
  isAuthenticated: boolean
  loading: boolean
  error: string | null
  login: (email: string, password: string) => Promise<void>
  signup: (userData: UserCreateRequest) => Promise<void>
  logout: () => void
}
```

### Backend-Database Interface

#### User Repository Interface
```python
from abc import ABC, abstractmethod
from typing import Optional
from sqlmodel import Session

class UserRepository(ABC):
    @abstractmethod
    def create_user(self, session: Session, user_data) -> Optional[User]:
        """Create a new user in the database"""
        pass

    @abstractmethod
    def get_user_by_email(self, session: Session, email: str) -> Optional[User]:
        """Retrieve a user by email"""
        pass

    @abstractmethod
    def update_user(self, session: Session, user_id: int, user_data) -> Optional[User]:
        """Update user information"""
        pass

    @abstractmethod
    def delete_user(self, session: Session, user_id: int) -> bool:
        """Delete a user by ID"""
        pass
```

## Security Contracts

### Token Contract
- Algorithm: HS256
- Expiration: 30 minutes from issue time
- Claims: `sub` (subject - user email), `exp` (expiration time)
- Format: Bearer token in Authorization header

### Password Contract
- Hashing: bcrypt with 12 rounds
- Minimum length: 8 characters
- Complexity: Must include uppercase, lowercase, number, and special character
- Storage: VARCHAR(255) in database

### Rate Limiting Contract
- Authentication endpoints: Max 5 attempts per minute per IP
- Failed attempts: Temporary lockout after 5 failed attempts

## Error Contract

### HTTP Status Codes
- 200 OK: Successful GET requests and logout
- 201 Created: Successful user creation
- 400 Bad Request: Validation errors or business logic violations
- 401 Unauthorized: Invalid or expired authentication token
- 404 Not Found: Resource not found
- 422 Unprocessable Entity: Request validation errors
- 500 Internal Server Error: Unexpected server errors

### Error Response Format
```json
{
  "detail": "Error message describing the issue"
}
```

## Configuration Contracts

### Environment Variables Contract

#### Backend Environment Variables
- `SECRET_KEY`: JWT secret key (required, min 32 chars)
- `ALGORITHM`: JWT algorithm (default: HS256)
- `ACCESS_TOKEN_EXPIRE_MINUTES`: Token expiration time (default: 30)
- `DATABASE_URL`: PostgreSQL connection string (required)
- `DB_POOL_SIZE`: Connection pool size (default: 10)
- `DB_MAX_OVERFLOW`: Max overflow connections (default: 20)
- `DB_POOL_RECYCLE`: Connection recycle time in seconds (default: 300)

#### Frontend Environment Variables
- `REACT_APP_API_URL`: Backend API URL (default: http://localhost:8000)

## Deployment Contracts

### CORS Policy Contract
- Allowed origins: Frontend domain in production
- Allowed methods: GET, POST, PUT, DELETE, OPTIONS
- Allowed headers: Content-Type, Authorization
- Credentials: Allowed (for cookie-based auth if implemented)

### Health Check Contract
- Endpoint: `/health`
- Response time: <100ms
- Status codes: 200 for healthy, 500 for unhealthy
- Database connectivity check included

## Performance Contracts

### Response Time SLA
- Authentication endpoints: <200ms p95
- User profile retrieval: <100ms p95
- Token validation: <50ms p95

### Concurrency Contract
- Support up to 1000 concurrent authenticated users
- Handle up to 100 authentication requests per second

## Validation Contracts

### Input Validation Rules
- Username: 3-50 alphanumeric characters + underscore/hyphen
- Email: Valid email format, max 100 characters
- Password: Min 8 characters with uppercase, lowercase, number, special char
- All inputs: Sanitized to prevent injection attacks

### Database Validation
- Unique constraints enforced at database level
- Field length limits enforced at database level
- Data type validation enforced at database level