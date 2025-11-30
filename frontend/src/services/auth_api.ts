/**
 * Authentication API service for sign in, sign up, and token management.
 */

// Get API base URL (browser-compatible)
// The API URL is injected by api-config.js script which runs before React loads
function getApiBaseUrl(): string {
  if (typeof window === 'undefined') {
    // SSR - return default, will be overridden on client
    return 'http://localhost:8000';
  }
  
  // Priority 1: Check for window-injected API URL (from api-config.js script)
  const windowApiUrl = (window as any).__API_BASE_URL__;
  if (windowApiUrl) {
    // Clean the URL - remove any quotes, backticks, or whitespace
    let cleanUrl = String(windowApiUrl).trim();
    // Remove backticks, single quotes, double quotes
    cleanUrl = cleanUrl.replace(/^[`'"]+|[`'"]+$/g, '');
    // Remove URL encoding artifacts
    cleanUrl = decodeURIComponent(cleanUrl);
    
    if (cleanUrl && cleanUrl !== 'API_URL_NOT_CONFIGURED' && cleanUrl !== 'undefined' && cleanUrl !== 'null') {
      // Ensure URL has protocol
      if (!cleanUrl.startsWith('http://') && !cleanUrl.startsWith('https://')) {
        cleanUrl = 'https://' + cleanUrl;
      }
      return cleanUrl;
    }
  }
  
  // Priority 2: Check if we're in production and log error
  const isProduction = window.location.hostname !== 'localhost' && 
                       window.location.hostname !== '127.0.0.1';
  
  if (isProduction) {
    console.error('API URL not configured! Please set REACT_APP_API_URL in Vercel environment variables and redeploy.');
    console.error('Current window.__API_BASE_URL__:', windowApiUrl);
    return 'API_URL_NOT_CONFIGURED';
  }
  
  // Default to localhost for development
  return 'http://localhost:8000';
}

// Don't cache API_BASE_URL - call getApiBaseUrl() directly in each function
// This ensures we get the latest value, especially after api-config.js loads

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
  const apiUrl = getApiBaseUrl(); // Get fresh URL each time
  if (apiUrl === 'API_URL_NOT_CONFIGURED') {
    throw new Error('API URL not configured. Please set REACT_APP_API_URL in Vercel environment variables.');
  }
  
  const response = await fetch(`${apiUrl}/auth/register`, {
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
  const apiUrl = getApiBaseUrl(); // Get fresh URL each time
  if (apiUrl === 'API_URL_NOT_CONFIGURED') {
    throw new Error('API URL not configured. Please set REACT_APP_API_URL in Vercel environment variables.');
  }
  
  const response = await fetch(`${apiUrl}/auth/login`, {
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
      const apiUrl = getApiBaseUrl();
      await fetch(`${apiUrl}/auth/logout`, {
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

  const apiUrl = getApiBaseUrl();
  const response = await fetch(`${apiUrl}/auth/refresh-token`, {
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

