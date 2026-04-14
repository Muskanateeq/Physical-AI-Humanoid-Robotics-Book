// Vercel Serverless Function for Better Auth
// Handles all authentication requests at /api/auth/*

import { betterAuth } from "better-auth";
import { Pool } from "pg";

// Create PostgreSQL connection pool (reused across invocations)
let pool;
function getPool() {
  if (!pool) {
    pool = new Pool({
      connectionString: process.env.DATABASE_URL,
      ssl: {
        rejectUnauthorized: false,
      },
      // Optimize for serverless
      max: 1, // Single connection per function instance
      idleTimeoutMillis: 30000,
      connectionTimeoutMillis: 10000,
    });
  }
  return pool;
}

// Initialize Better Auth (cached across invocations)
let auth;
function getAuth() {
  if (!auth) {
    auth = betterAuth({
      database: getPool(),

      // Email and password authentication
      emailAndPassword: {
        enabled: true,
        requireEmailVerification: false,
      },

      // Social providers (Google and GitHub)
      // Only configure if credentials are available
      socialProviders: {
        ...(process.env.GOOGLE_CLIENT_ID && process.env.GOOGLE_CLIENT_SECRET ? {
          google: {
            clientId: process.env.GOOGLE_CLIENT_ID,
            clientSecret: process.env.GOOGLE_CLIENT_SECRET,
          },
        } : {}),
        ...(process.env.GITHUB_CLIENT_ID && process.env.GITHUB_CLIENT_SECRET ? {
          github: {
            clientId: process.env.GITHUB_CLIENT_ID,
            clientSecret: process.env.GITHUB_CLIENT_SECRET,
          },
        } : {}),
      },

      // Session configuration
      session: {
        expiresIn: 60 * 60 * 24 * 7, // 7 days
        updateAge: 60 * 60 * 24, // 1 day
      },

      // JWT configuration
      secret: process.env.BETTER_AUTH_SECRET,

      // Base URL - Use production domain in production, localhost in development
      // VERCEL_ENV is "production" on production, "preview" on preview, undefined locally
      baseURL: process.env.VERCEL_ENV === "production"
        ? "https://neurobotics-ai-book.vercel.app"
        : "http://localhost:3001",

      // Trusted origins - Allow requests from same domain
      trustedOrigins: [
        "https://neurobotics-ai-book.vercel.app",
        "http://localhost:3000",
      ],

      // Advanced options
      advanced: {
        generateId: () => {
          return crypto.randomUUID();
        },
      },
    });
  }
  return auth;
}

// Vercel serverless handler
export default async function handler(req, res) {
  // Enable CORS for same-origin and development
  const origin = req.headers.origin || req.headers.referer || "*";
  res.setHeader("Access-Control-Allow-Credentials", "true");
  res.setHeader("Access-Control-Allow-Origin", origin);
  res.setHeader(
    "Access-Control-Allow-Methods",
    "GET,POST,PUT,DELETE,OPTIONS"
  );
  res.setHeader(
    "Access-Control-Allow-Headers",
    "Content-Type, Authorization, Cookie"
  );

  // Handle OPTIONS preflight
  if (req.method === "OPTIONS") {
    res.status(200).end();
    return;
  }

  try {
    // Log ALL requests for debugging
    console.log("🔍 Auth Request:", {
      method: req.method,
      url: req.url,
      path: req.url,
      headers: {
        host: req.headers.host,
        origin: req.headers.origin,
        contentType: req.headers["content-type"],
      },
      hasBody: !!req.body,
      bodyType: req.body ? typeof req.body : "none",
    });

    // Get Better Auth instance
    const authInstance = getAuth();

    // Build full URL
    const protocol = req.headers["x-forwarded-proto"] || "https";
    const host = req.headers["x-forwarded-host"] || req.headers.host;
    const url = new URL(req.url, `${protocol}://${host}`);

    // Convert Vercel request to Web Request
    const headers = new Headers();
    Object.entries(req.headers).forEach(([key, value]) => {
      if (value) {
        headers.set(key, Array.isArray(value) ? value.join(", ") : value);
      }
    });

    // Handle request body
    let body = undefined;
    if (req.method !== "GET" && req.method !== "HEAD") {
      if (req.body) {
        // Body already parsed by Vercel
        body = typeof req.body === "string" ? req.body : JSON.stringify(req.body);
      }
    }

    const request = new Request(url, {
      method: req.method,
      headers: headers,
      body: body,
    });

    // Call Better Auth handler
    const response = await authInstance.handler(request);

    // Copy response headers to Vercel response
    response.headers.forEach((value, key) => {
      res.setHeader(key, value);
    });

    // Set status code
    res.status(response.status);

    // Send response body
    if (response.body) {
      const responseText = await response.text();
      res.send(responseText);
    } else {
      res.end();
    }
  } catch (error) {
    console.error("❌ Auth Error:", error);
    res.status(500).json({
      error: "Authentication error",
      message: error.message,
    });
  }
}
