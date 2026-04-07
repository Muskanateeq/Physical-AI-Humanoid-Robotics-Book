// API URL configuration for Docusaurus
const API_BASE_URL = typeof window !== 'undefined'
  ? window.ENV?.REACT_APP_API_URL || 'http://localhost:8001'
  : process.env.REACT_APP_API_URL || 'http://localhost:8001';

// Export for use in Docusaurus components
export const API_CONFIG = {
  BASE_URL: API_BASE_URL,
  CHATKIT_URL: `${API_BASE_URL}/chatkit`,
  AUTH_URL: `${API_BASE_URL}/auth`,
};