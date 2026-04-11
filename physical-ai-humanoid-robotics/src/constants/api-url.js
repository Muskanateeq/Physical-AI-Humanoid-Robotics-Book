// API URL configuration for Docusaurus
// Import from centralized env config
import { ENV } from '../config/env';

// Export for use in Docusaurus components
export const API_CONFIG = {
  BASE_URL: ENV.BACKEND_API_URL,
  CHATKIT_URL: ENV.CHATKIT_API_URL,
  AUTH_URL: ENV.AUTH_SERVER_URL,
};