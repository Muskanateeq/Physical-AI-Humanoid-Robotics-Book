// Environment configuration for client-side code
// Auto-detects production vs development based on hostname

// Production URLs (hardcoded)
const PRODUCTION_CONFIG = {
  // Empty string = same domain (Vercel will handle /api/auth/* via serverless function)
  AUTH_SERVER_URL: '',
  BACKEND_API_URL: 'https://physical-ai-humanoid-robotics-book-7mgx.onrender.com',
};

// Development URLs
const DEVELOPMENT_CONFIG = {
  AUTH_SERVER_URL: 'http://localhost:3001',
  BACKEND_API_URL: 'http://localhost:8001',
};

// Detect environment based on hostname
function getConfig() {
  if (typeof window === 'undefined') {
    // SSR/Build time - use development as fallback
    return DEVELOPMENT_CONFIG;
  }

  const hostname = window.location.hostname;
  const isProduction = hostname.includes('vercel.app') || hostname.includes('neurobotics-ai-book');

  return isProduction ? PRODUCTION_CONFIG : DEVELOPMENT_CONFIG;
}

// Export environment configuration
export const ENV = {
  get AUTH_SERVER_URL() {
    return getConfig().AUTH_SERVER_URL;
  },
  get BACKEND_API_URL() {
    return getConfig().BACKEND_API_URL;
  },
  get CHATKIT_API_URL() {
    return `${getConfig().BACKEND_API_URL}/chatkit`;
  }
};
