// Environment configuration for client-side code
// Uses Docusaurus customFields to access build-time environment variables

import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Hook to get environment config in React components
export function useEnv() {
  const { siteConfig } = useDocusaurusContext();
  return {
    AUTH_SERVER_URL: siteConfig.customFields?.AUTH_SERVER_URL as string,
    BACKEND_API_URL: siteConfig.customFields?.BACKEND_API_URL as string,
    get CHATKIT_API_URL() {
      return `${this.BACKEND_API_URL}/chatkit`;
    }
  };
}

// For non-React contexts (module-level initialization)
// These are set at build time from docusaurus.config.js
export const ENV = {
  AUTH_SERVER_URL: 'http://localhost:3001', // Fallback for SSR/build
  BACKEND_API_URL: 'http://localhost:8001', // Fallback for SSR/build
  get CHATKIT_API_URL() {
    return `${this.BACKEND_API_URL}/chatkit`;
  }
};

// Initialize from window if available (client-side)
if (typeof window !== 'undefined' && (window as any).docusaurus) {
  const customFields = (window as any).docusaurus.siteConfig?.customFields;
  if (customFields) {
    ENV.AUTH_SERVER_URL = customFields.AUTH_SERVER_URL || ENV.AUTH_SERVER_URL;
    ENV.BACKEND_API_URL = customFields.BACKEND_API_URL || ENV.BACKEND_API_URL;
  }
}
