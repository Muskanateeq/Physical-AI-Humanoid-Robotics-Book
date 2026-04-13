// Client-side auth configuration
// This file is safe to import in browser/client components
// In production: uses same domain (Vercel serverless function at /api/auth/*)
// In development: uses standalone auth server on port 3001

import { createAuthClient } from "better-auth/react";
import { ENV } from "../config/env";

export const authClient = createAuthClient({
  // Use ENV.AUTH_SERVER_URL if defined, otherwise fallback to localhost
  // Empty string '' means "use same domain" (for Vercel serverless)
  baseURL: ENV.AUTH_SERVER_URL !== undefined && ENV.AUTH_SERVER_URL !== null
    ? ENV.AUTH_SERVER_URL
    : "http://localhost:3001",
  fetchOptions: {
    credentials: "include", // CRITICAL: Send cookies with cross-origin requests
  },
});

export const {
  signIn,
  signUp,
  signOut,
  useSession,
  getSession
} = authClient;
