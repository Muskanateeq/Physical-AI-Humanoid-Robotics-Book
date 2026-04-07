// Client-side auth configuration
// This file is safe to import in browser/client components
// Points to standalone auth server on port 3001

import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:3001", // Standalone auth server
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
