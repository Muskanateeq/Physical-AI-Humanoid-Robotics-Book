// Better Auth Server (Standalone Node.js Server)
// Run this separately from Docusaurus

import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";
import { Pool } from "pg";
import express from "express";
import cors from "cors";
import dotenv from "dotenv";

dotenv.config();

// Create PostgreSQL connection pool for Neon
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false,
  },
});

// Initialize Better Auth
export const auth = betterAuth({
  database: pool,

  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },

  // Social providers (Google and GitHub)
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
      redirectURI: (process.env.BETTER_AUTH_URL || "http://localhost:3001") + "/api/auth/callback/google",
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID || "",
      clientSecret: process.env.GITHUB_CLIENT_SECRET || "",
      redirectURI: (process.env.BETTER_AUTH_URL || "http://localhost:3001") + "/api/auth/callback/github",
    },
  },

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
  },

  // JWT configuration
  secret: process.env.BETTER_AUTH_SECRET || "your-secret-key-here",

  // Base URL - Auth server runs on port 3001
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",

  // Trusted origins - Allow requests from Docusaurus frontend
  trustedOrigins: [process.env.FRONTEND_URL || "http://localhost:3000"],

  // Advanced options
  advanced: {
    generateId: () => {
      return crypto.randomUUID();
    },
  },
});

// Create Express app
const app = express();

// Enable CORS for Docusaurus frontend - MUST be before auth handler
app.use(cors({
  origin: process.env.FRONTEND_URL || "http://localhost:3000",
  credentials: true,
}));

// Mount Better Auth handler - CRITICAL: Must be BEFORE express.json()
// Better Auth handles body parsing internally via toNodeHandler
app.all("/api/auth/*", async (req, res, next) => {
  try {
    // Log OAuth callbacks for debugging
    if (req.path.includes('/callback/')) {
      console.log('📥 OAuth Callback:', {
        path: req.path,
        query: req.query,
        method: req.method
      });
    }

    await toNodeHandler(auth)(req, res, next);
  } catch (error) {
    console.error('❌ Better Auth Error:', error);
    res.status(500).json({
      error: 'Authentication error',
      message: error.message
    });
  }
});

// Body parsing for non-auth routes (AFTER auth handler)
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Health check
app.get("/health", (req, res) => {
  res.json({ status: "ok", service: "Better Auth Server" });
});

// Start server
const PORT = process.env.PORT || 3001;
app.listen(PORT, () => {
  console.log(`✅ Better Auth Server running on http://localhost:${PORT}`);
  console.log(`📍 Auth endpoints: http://localhost:${PORT}/api/auth/*`);
});
