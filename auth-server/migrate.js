// Run Better Auth database migrations programmatically
import { betterAuth } from "better-auth";
import { getMigrations } from "better-auth/db/migration";
import { Pool } from "pg";
import dotenv from "dotenv";

dotenv.config();

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false,
  },
});

// Initialize Better Auth with same config as server
const auth = betterAuth({
  database: pool,
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
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
  session: {
    expiresIn: 60 * 60 * 24 * 7,
    updateAge: 60 * 60 * 24,
  },
  secret: process.env.BETTER_AUTH_SECRET || "your-secret-key-here",
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
});

async function runMigrations() {
  try {
    console.log("🔄 Running Better Auth database migrations...");

    const { toBeCreated, toBeAdded, runMigrations } = await getMigrations(auth.options);

    console.log("\n📋 Tables to be created:", toBeCreated);
    console.log("📋 Columns to be added:", toBeAdded);

    if (toBeCreated.length === 0 && toBeAdded.length === 0) {
      console.log("\n✅ Database schema is up to date!");
      await pool.end();
      process.exit(0);
    }

    console.log("\n🚀 Applying migrations...");
    await runMigrations();

    console.log("\n✅ Migrations completed successfully!");
    await pool.end();
    process.exit(0);
  } catch (error) {
    console.error("\n❌ Migration failed:", error);
    await pool.end();
    process.exit(1);
  }
}

runMigrations();
