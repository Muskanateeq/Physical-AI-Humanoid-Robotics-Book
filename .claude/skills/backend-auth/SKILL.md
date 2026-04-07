---
name: backend-auth
description: Create simple authentication backend using Python and FastAPI. Focus on user signup, signin, and token-based authentication. No frontend, UI/UX, or complex role-based access control. Simple, clean implementation with proper security practices.
---

# Backend Authentication Skill

This skill provides guidance for creating simple authentication backend using Python and FastAPI. The implementation focuses on user signup, signin, and token-based authentication with proper security practices.

## When to Use This Skill

Use when Claude needs to create backend authentication logic for:
- Python and FastAPI-based applications
- Simple user signup/signin functionality
- Token-based authentication system
- Basic authentication checks
- Secure password handling
- Simple user database models

## Core Requirements

### 1. Backend-Only Implementation
- **SIRF backend logic** to be written
- **No frontend, UI/UX, or styling** mention
- **No complex role-based access control**
- **No social login** implementation
- **Simple, clean, and understandable** implementation
- **No hallucination** of frontend components

### 2. Scope (Simple Auth Only)
Handle only these backend components:
- User signup
- User signin
- User logout (optional, token-based)
- Basic authentication check

## Signup Functionality

### Endpoint
- **POST /signup**

### Required Fields
- `username`
- `email`
- `password`

### Rules
- Email must be unique
- Password must be securely hashed
- User data must be saved to database

### Response
- Success message
- User id, username, email (password NEVER returned)
- No sensitive information exposed

## Signin Functionality

### Endpoint
- **POST /signin**

### Required Fields
- `email`
- `password`

### Rules
- Email must exist in database
- Password hash must be verified
- Authentication token must be generated

### Response
- Authentication success message
- Auth token (JWT or simple token)
- Basic user info (id, username, email)

## Authentication Method

### Token-Based Authentication
- Simple token-based authentication system
- Token generated on signin
- Basic middleware for token verification
- Proper token validation and security

## Database Requirements

### Simple User Model
Database table with fields:
- `id` (primary key)
- `username`
- `email` (unique)
- `hashed_password`
- `created_at`

### No Advanced Relations
- Keep relationships simple
- Focus only on user authentication needs
- No complex database schemas

## Security Basics

### Password Security
- Password hashing mandatory (using bcrypt or similar)
- Plain password NEVER stored
- Secure password handling throughout

### Input Validation
- Basic input validation
- Proper sanitization of inputs
- Clear error messages without exposing sensitive information

### Error Handling
- Clear, non-descriptive error messages
- Proper HTTP status codes
- Security-focused error responses

## Endpoints Overview

### User Registration
```
POST /signup
{
  "username": "string",
  "email": "string",
  "password": "string"
}
```

### User Authentication
```
POST /signin
{
  "email": "string",
  "password": "string"
}
```

### Authentication Check (Protected)
```
GET /me (requires valid token)
```

### User Logout (Optional)
```
POST /logout (requires valid token)
```

## Implementation Checklist

- [ ] User model with proper fields and constraints
- [ ] Password hashing implementation
- [ ] Signup endpoint with validation
- [ ] Signin endpoint with authentication
- [ ] Token-based authentication system
- [ ] Middleware for token verification
- [ ] Proper error handling
- [ ] Database integration
- [ ] Security best practices implemented
- [ ] No frontend or UI components included