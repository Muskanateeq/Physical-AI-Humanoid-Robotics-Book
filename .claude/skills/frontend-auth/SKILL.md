---
name: frontend-auth
description: Implement frontend authentication flow for Docusaurus (React-based) environment. Focus only on frontend logic for auth state management, signup/signin pages, navbar integration, and chatbot access control. No UI design, styling, or backend implementation.
---

# Frontend Authentication Flow Skill

This skill provides guidance for creating frontend authentication flow in Docusaurus (React-based) environment. The implementation focuses only on frontend logic with no UI design, styling, or backend implementation.

## When to Use This Skill

Use when Claude needs to create frontend authentication logic for:
- Docusaurus-based websites
- Frontend-only authentication flow
- Auth state management with context/provider
- Signup/signin page behavior
- Navbar integration with authentication state
- Chatbot access control
- Frontend form validation

## Core Requirements

### 1. Frontend-Only Implementation
- **No UI design, colors, animations, or styling** to be defined
- **No backend, database, or API** implementation
- **Only Docusaurus compatible React logic** to be written
- **No hallucination** of backend functionality

### 2. Scope (Frontend Only)
Handle only these frontend components:
- Signup page behavior
- Signin page behavior
- Auth state management (frontend)
- Navbar state update
- Chatbot access gating (frontend)

## Authentication State Management

### Auth Context/Provider Implementation
Create an Auth Context/Provider with the following state data:
- `isAuthenticated` (boolean)
- `username`
- `email`

### State Persistence
- Page refresh should persist auth state (localStorage or equivalent)
- Logout should clear auth state
- State should be accessible throughout the application

### Context Structure
```jsx
// AuthContext.js
import React, { createContext, useContext, useReducer } from 'react';

// Initial state
const initialState = {
  isAuthenticated: false,
  username: null,
  email: null,
  loading: true
};

// Auth context
const AuthContext = createContext();

// Auth provider component
export const AuthProvider = ({ children }) => {
  // Implementation details
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
```

## Signup Page (Frontend Logic)

### Form Fields
- username
- email
- password
- confirmPassword

### Frontend Validation
- All fields required
- password must match confirmPassword
- Client-side validation only

### Success Behavior (Frontend Simulation)
- User should NOT be authenticated immediately after signup
- User data should be stored in temporary state
- User should be redirected to Signin page

## Signin Page (Frontend Logic)

### Form Fields
- email
- password

### Frontend Validation
- Empty fields should not be allowed
- Client-side validation only

### Success Behavior (Frontend Simulation)
- Auth state should update:
  - `isAuthenticated = true`
  - `username` and `email` should be set
- Navbar should update to reflect authenticated state
- Chatbot access should be enabled

## Navbar Frontend Integration

### Existing Docusaurus Navbar
- Use existing Docusaurus navbar
- Update navbar based on authentication state

### Navbar Behavior
- If `isAuthenticated` is false:
  - Show "Sign In" button
- If `isAuthenticated` is true:
  - Show username (as dropdown trigger)
  - Show dropdown with email and logout button

### Dropdown Behavior
- Display email
- Include logout button
- Logout functionality:
  - Clear auth state
  - Clear localStorage
  - Return navbar to unauthenticated state

## Chatbot Access Control (Frontend)

### Chatbot Button
- Floating chatbot button should always be visible
- Click behavior based on authentication state:
  - If `isAuthenticated` is false: redirect to Signup/Signin page
  - If `isAuthenticated` is true: open chatbot component

## Routing & Protection

### Route Configuration
- Signup/Signin routes should be public
- Chatbot access should be protected
- Direct chatbot page access should redirect to auth if not authenticated

## Implementation Checklist

- [ ] Auth Context/Provider with state management
- [ ] LocalStorage persistence for auth state
- [ ] Signup page with form validation
- [ ] Signin page with form validation
- [ ] Frontend-only authentication simulation
- [ ] Navbar integration with dropdown
- [ ] Chatbot access control logic
- [ ] Route protection implementation
- [ ] Docusaurus compatibility
- [ ] No UI design, styling, or backend implementation