# Docusaurus Integration References

This document provides references for integrating the frontend authentication flow with Docusaurus, including setup instructions, component integration, and best practices.

## Docusaurus Setup for Authentication

### 1. App Wrapping with Auth Provider

To make the authentication context available throughout your Docusaurus application, you need to wrap your app with the AuthProvider. This is typically done in the `src/pages/index.js` file or in a custom layout.

```jsx
// In your main layout or index page
import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';

function App({ children }) {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}
```

### 2. Using Docusaurus' Swizzling for Navbar Customization

Docusaurus allows you to customize the navbar by "swizzling" it. You can create a custom navbar component that integrates with your authentication state.

```bash
# Swizzle the navbar to customize it
npm run swizzle @docusaurus/theme-classic Navbar -- --layout=preset
```

This creates a custom navbar component in `src/theme/Navbar/index.js` where you can integrate the authentication components.

### 3. Creating Custom Pages for Auth

Create dedicated pages for signup and signin in the `src/pages` directory:

```
src/
  pages/
    signup.js
    signin.js
```

## Docusaurus-Specific Routing

### Using Docusaurus' Link Component

When creating navigation links in Docusaurus, use the built-in Link component:

```jsx
import Link from '@docusaurus/Link';

// Instead of react-router-dom's Link
<Link to="/signin">Sign In</Link>
```

### Programmatic Navigation

For programmatic navigation, you can use Docusaurus' navigate function:

```jsx
import { navigate } from '@docusaurus/reveal';

// Navigate after successful auth
navigate('/');
```

## Integrating with Docusaurus Layouts

### Using Docusaurus Layout Components

Docusaurus provides various layout components that you can use to structure your auth pages:

```jsx
import Layout from '@theme/Layout';

function SignupPage() {
  return (
    <Layout title="Sign Up" description="Create a new account">
      <main>
        <div className="container margin-vert--lg">
          {/* Your signup form here */}
        </div>
      </main>
    </Layout>
  );
}
```

## Docusaurus Styling Integration

### Using Docusaurus CSS Variables

Docusaurus uses CSS variables for theming. You can reference these in your auth components:

```css
.auth-form {
  background-color: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: var(--ifm-global-radius);
  padding: var(--ifm-global-spacing);
}
```

### Adding Custom Styles

Add custom authentication styles in your Docusaurus styles directory:

```bash
# Create a directory for custom styles
mkdir -p src/css
```

```css
/* src/css/auth.css */
.auth-dropdown {
  position: relative;
}

.auth-dropdown .dropdown-content {
  display: none;
  position: absolute;
  right: 0;
  background-color: var(--ifm-background-surface-color);
  min-width: 160px;
  box-shadow: 0px 8px 16px 0px rgba(0,0,0,0.2);
  z-index: 1;
}

.auth-dropdown:hover .dropdown-content {
  display: block;
}
```

## Docusaurus Plugin Integration

### Using Docusaurus Plugins for Enhanced Auth Features

You might want to use Docusaurus plugins to enhance your authentication experience:

```js
// docusaurus.config.js
module.exports = {
  plugins: [
    // Add plugins as needed for enhanced functionality
    // For example, a plugin for handling redirects
  ],
};
```

## Docusaurus Build Considerations

### Environment-Specific Configuration

When building for different environments, you might need to handle authentication differently:

```js
// Use environment variables for different environments
const isDevelopment = process.env.NODE_ENV === 'development';

// In development, you might want to mock auth
if (isDevelopment) {
  // Mock auth implementation
}
```

### Static Generation Considerations

Docusaurus generates static sites, so consider how authentication state will be handled:

- Auth state should be managed client-side using localStorage
- Server-side rendering should handle loading states appropriately
- Consider using useEffect for initializing auth state after component mounts

## Docusaurus SEO and Accessibility

### Adding Meta Tags for Auth Pages

Ensure your authentication pages have proper meta tags:

```jsx
import Head from '@docusaurus/Head';

function SignupPage() {
  return (
    <>
      <Head>
        <title>Sign Up - Your Site</title>
        <meta name="description" content="Create a new account to access our services" />
      </Head>
      {/* Page content */}
    </>
  );
}
```

### Accessibility Considerations

Make sure your authentication forms are accessible:

```jsx
// Use proper labels and ARIA attributes
<input
  type="email"
  id="email"
  name="email"
  aria-describedby="email-help-text"
  aria-invalid={!!errors.email}
/>
<div id="email-help-text">Enter your email address</div>
{errors.email && (
  <div role="alert" aria-live="polite">
    {errors.email}
  </div>
)}
```

## Docusaurus Deployment Considerations

### Handling Authentication in Deployed Environments

When deploying your Docusaurus site:

- Ensure localStorage-based auth state works correctly
- Test that auth redirects work in the deployed environment
- Verify that the floating chatbot button functions properly
- Check that all auth-related navigation works as expected

## Common Docusaurus Auth Patterns

### Conditionally Rendering Content

```jsx
import { useAuth } from '../contexts/AuthContext';

function MyComponent() {
  const { isAuthenticated, loading } = useAuth();

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div>
      {isAuthenticated ? (
        <div>Welcome back, user!</div>
      ) : (
        <div>Please sign in to continue</div>
      )}
    </div>
  );
}
```

### Protecting Content Based on Auth Status

```jsx
import { useAuth } from '../contexts/AuthContext';
import Link from '@docusaurus/Link';

function ProtectedContent() {
  const { isAuthenticated } = useAuth();

  if (!isAuthenticated) {
    return (
      <div>
        <p>Please sign in to access this content</p>
        <Link to="/signin">Sign In</Link>
      </div>
    );
  }

  return <div>This is protected content</div>;
}
```