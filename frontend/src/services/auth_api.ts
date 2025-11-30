/**
 * Authentication API service for sign in, sign up, and token management.
 */

// Get API base URL (browser-compatible)
// Docusaurus injects environment variables at build time via webpack DefinePlugin
function getApiBaseUrl(): string {
  // Priority 1: Check for window-injected API URL (from api-config.js script)
  if (typeof window !== 'undefined' && (window as any).__API_BASE_URL__) {
    const url = (window as any).__API_BASE_URL__;
    if (url && url !== 'API_URL_NOT_CONFIGURED') {
      return url;
    }
  }
  
  // Priority 2: Check for webpack-injected environment variable
  // This works because webpack DefinePlugin replaces process.env.REACT_APP_API_URL at build time
  if (typeof window !== 'undefined') {
    // In production build, webpack will replace this with the actual value
    const webpackApiUrl = (typeof process !== 'undefined' && (process as any).env?.REACT_APP_API_URL);
    if (webpackApiUrl && webpackApiUrl !== 'undefined') {
      return webpackApiUrl;
    }
  }
  
  // Priority 3: Check if we're in production and log error
  if (typeof window !== 'undefined') {
    const isProduction = window.location.hostname !== 'localhost' && 
                         window.location.hostname !== '127.0.0.1';
    
    if (isProduction) {
      console.error('API URL not configured! Please set REACT_APP_API_URL in Vercel environment variables and redeploy.');
      // Don't return invalid URL, return a clear error message
      return 'API_URL_NOT_CONFIGURED';
    }
  }
  
  // Default to localhost for development
  return 'http://localhost:8000';
}

const API_BASE_URL = getApiBaseUrl();

export interface SignUpRequest {
  email: string;
  password: string;
  background_info?: {
    software_experience?: string;
    hardware_experience?: string;
    robotics_experience?: string;
    programming_languages?: string;
  };
}

export interface SignInRequest {
  email: string;
  password: string;
}

export interface AuthResponse {
  message: string;
  token?: string;
  userId?: string;
}

/**
 * Sign up a new user.
 */
export async function signUp(data: SignUpRequest): Promise<AuthResponse> {
  if (API_BASE_URL === 'API_URL_NOT_CONFIGURED') {
    throw new Error('API URL not configured. Please set REACT_APP_API_URL in Vercel environment variables.');
  }
  
  const response = await fetch(`${API_BASE_URL}/auth/register`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ message: 'Failed to sign up' }));
    throw new Error(error.message || error.detail?.message || `HTTP error! status: ${response.status}`);
  }

  const result = await response.json();
  
  // Store token if provided
  if (result.token && typeof window !== 'undefined' && window.localStorage) {
    try {
      localStorage.setItem('auth_token', result.token);
      if (result.userId) {
        localStorage.setItem('user_id', result.userId);
      }
    } catch (error) {
      console.error('Error storing auth token:', error);
    }
  }

  return result;
}

/**
 * Sign in an existing user.
 */
export async function signIn(data: SignInRequest): Promise<AuthResponse> {
  if (API_BASE_URL === 'API_URL_NOT_CONFIGURED') {
    throw new Error('API URL not configured. Please set REACT_APP_API_URL in Vercel environment variables.');
  }
  
  const response = await fetch(`${API_BASE_URL}/auth/login`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ message: 'Failed to sign in' }));
    throw new Error(error.message || error.detail?.message || `HTTP error! status: ${response.status}`);
  }

  const result = await response.json();
  
  // Store token if provided
  if (result.token) {
    localStorage.setItem('auth_token', result.token);
  }

  return result;
}

/**
 * Sign out the current user.
 */
export async function signOut(): Promise<void> {
  if (typeof window === 'undefined' || !window.localStorage) {
    return;
  }

  const token = localStorage.getItem('auth_token');
  
  if (token) {
    try {
      await fetch(`${API_BASE_URL}/auth/logout`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
      });
    } catch (error) {
      // Continue with local signout even if API call fails
      console.error('Error during sign out:', error);
    }
  }

  // Clear local storage
  try {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user_id');
  } catch (error) {
    console.error('Error clearing localStorage:', error);
  }
}

/**
 * Get the current auth token.
 */
export function getAuthToken(): string | null {
  if (typeof window === 'undefined' || !window.localStorage) {
    return null;
  }
  try {
    return localStorage.getItem('auth_token');
  } catch (error) {
    console.error('Error getting auth token:', error);
    return null;
  }
}

/**
 * Check if user is authenticated.
 */
export function isAuthenticated(): boolean {
  if (typeof window === 'undefined' || !window.localStorage) {
    return false;
  }
  try {
    return !!localStorage.getItem('auth_token');
  } catch (error) {
    console.error('Error checking authentication:', error);
    return false;
  }
}

/**
 * Refresh the authentication token.
 */
export async function refreshToken(): Promise<string> {
  const token = localStorage.getItem('auth_token');
  
  if (!token) {
    throw new Error('No token to refresh');
  }

  const response = await fetch(`${API_BASE_URL}/auth/refresh-token`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`,
    },
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ message: 'Failed to refresh token' }));
    throw new Error(error.message || error.detail?.message || `HTTP error! status: ${response.status}`);
  }

  const result = await response.json();
  
  if (result.token && typeof window !== 'undefined' && window.localStorage) {
    try {
      localStorage.setItem('auth_token', result.token);
      return result.token;
    } catch (error) {
      console.error('Error storing refreshed token:', error);
    }
  }

  throw new Error('No token in refresh response');
}

