---
name: database-auth
description: Create database design and implementation for authentication using PostgreSQL and SQLModel. Focus on user table/model with proper constraints, password hashing compatibility, and Railway deployment compatibility. No frontend or backend code.
---

# Database Authentication Skill

This skill provides guidance for creating database design and implementation for authentication using PostgreSQL and SQLModel. The implementation focuses on user table/model with proper constraints, password hashing compatibility, and Railway deployment compatibility.

## When to Use This Skill

Use when Claude needs to create database authentication models for:
- PostgreSQL-based authentication systems
- SQLModel ORM integration
- FastAPI authentication backends
- Railway deployment compatibility
- User table design with proper constraints
- Password hashing compatibility

## Core Requirements

### 1. Database-Only Implementation
- **SIRF database design** to be created
- **No frontend, UI/UX, or backend code** to be included
- **Simple, minimal, and production-ready** design
- **Compatible with existing auth skill** requirements
- **No hallucination** of non-database components

### 2. Scope (Database Only)
Handle only these database components:
- User table/model with required fields
- Password hashing compatible column
- Auth token storage (optional)
- Timestamps (created_at, updated_at)
- Unique constraints (email)
- Minimal required fields only

## User Table / Model Design

### Required Fields
- `id` (Primary key, auto-incrementing integer or UUID)
- `username` (string)
- `email` (string, unique)
- `hashed_password` (string)
- `created_at` (datetime, default current time)

### Optional / Future-Proof Fields
- `last_login` (datetime)
- `is_active` (boolean, default true)

## Database Rules

### Constraints and Validation
- Email must be unique (unique constraint)
- Password must never be stored in plain text
- Table structure must be compatible with FastAPI + SQLModel ORM
- Must be deployable on Railway PostgreSQL
- Design should be simple and extendable

### Security Considerations
- Password hashing compatibility (for bcrypt or similar)
- Proper indexing for performance
- Secure column types and constraints

## Relationships

### Current Design
- No relationships required for simple authentication
- Table design should be extendable for future needs (roles, permissions)

### Future Extensions
- Design should accommodate role-based relationships if needed
- Permission or session tables could be added later

## Migration and Setup

### File Structure
Recommended structure for database files:
```
database/
├── models.py      # SQLModel models
├── database.py    # Database connection setup
├── config.py      # Database configuration
└── migrations/    # Migration files (optional)
```

### SQLModel / SQLAlchemy Syntax
- Use SQLModel for model definitions
- Include proper field constraints and indices
- Follow Railway PostgreSQL compatibility

### Database Connection
- Use Railway DATABASE_URL environment variable
- Implement connection pooling
- Include error handling

### Migration / Table Creation
- Simple method for table creation
- Support for database migrations
- Safe initialization process

## Implementation Checklist

- [ ] User model with required fields (id, username, email, hashed_password, created_at)
- [ ] Email unique constraint implemented
- [ ] Password hashing compatibility ensured
- [ ] SQLModel ORM compatibility
- [ ] Railway PostgreSQL deployment compatibility
- [ ] Proper indexing for performance
- [ ] Optional fields included (last_login, is_active)
- [ ] Simple and extendable design
- [ ] No frontend or backend code included
- [ ] Migration setup method defined
- [ ] Database connection configuration