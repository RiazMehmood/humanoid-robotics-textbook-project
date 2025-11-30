/**
 * Frontend service for personalization API interactions.
 */
import { getAuthToken } from './auth_api';

// Get API base URL (browser-compatible) - shared logic with auth_api.ts
function getApiBaseUrl(): string {
  if (typeof window === 'undefined') {
    return 'http://localhost:8000';
  }
  
  const windowApiUrl = (window as any).__API_BASE_URL__;
  if (windowApiUrl) {
    // Clean the URL - remove any quotes, backticks, or whitespace
    let cleanUrl = String(windowApiUrl).trim();
    cleanUrl = cleanUrl.replace(/^[`'"]+|[`'"]+$/g, '');
    try {
      cleanUrl = decodeURIComponent(cleanUrl);
    } catch (e) {
      // If decoding fails, use as-is
    }
    
    if (cleanUrl && cleanUrl !== 'API_URL_NOT_CONFIGURED' && cleanUrl !== 'undefined' && cleanUrl !== 'null') {
      if (!cleanUrl.startsWith('http://') && !cleanUrl.startsWith('https://')) {
        cleanUrl = 'https://' + cleanUrl;
      }
      return cleanUrl;
    }
  }
  
  const isProduction = window.location.hostname !== 'localhost' && 
                       window.location.hostname !== '127.0.0.1';
  return isProduction ? 'API_URL_NOT_CONFIGURED' : 'http://localhost:8000';
}

export interface UserPreferences {
  language: string;
  theme: string;
}

export interface PreferencesUpdate {
  language?: string;
  theme?: string;
}

export interface TranslationResponse {
  contentId: string;
  translatedText: string;
  sourceLanguage: string;
  targetLanguage: string;
}

export interface ContentRecommendation {
  contentId: string;
  title: string;
  type?: string;
  reason?: string;
}

export interface RecommendationsResponse {
  recommendations: ContentRecommendation[];
}

/**
 * Get user content preferences.
 */
export async function getPreferences(token?: string): Promise<UserPreferences> {
  const headers: HeadersInit = {
    'Content-Type': 'application/json',
  };

  const authToken = token || getAuthToken();
  if (authToken) {
    headers['Authorization'] = `Bearer ${authToken}`;
  }

  const apiUrl = getApiBaseUrl();
  const response = await fetch(`${apiUrl}/personalization/preferences`, {
    method: 'GET',
    headers,
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ message: 'Failed to get preferences' }));
    throw new Error(error.message || `HTTP error! status: ${response.status}`);
  }

  return response.json();
}

/**
 * Update user content preferences.
 */
export async function updatePreferences(
  preferences: PreferencesUpdate,
  token?: string
): Promise<{ message: string; preferences: UserPreferences }> {
  const headers: HeadersInit = {
    'Content-Type': 'application/json',
  };

  const authToken = token || getAuthToken();
  if (authToken) {
    headers['Authorization'] = `Bearer ${authToken}`;
  }

  const apiUrl = getApiBaseUrl();
  const response = await fetch(`${apiUrl}/personalization/preferences`, {
    method: 'PUT',
    headers,
    body: JSON.stringify(preferences),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ message: 'Failed to update preferences' }));
    throw new Error(error.message || `HTTP error! status: ${response.status}`);
  }

  return response.json();
}

/**
 * Get translated content for a specific item.
 */
export async function translateContent(
  contentId: string,
  targetLanguage: string,
  sourceLanguage: string = 'en',
  token?: string
): Promise<TranslationResponse> {
  const headers: HeadersInit = {
    'Content-Type': 'application/json',
  };

  const authToken = token || getAuthToken();
  if (authToken) {
    headers['Authorization'] = `Bearer ${authToken}`;
  }

  const params = new URLSearchParams({
    targetLanguage,
    sourceLanguage,
  });

  const apiUrl = getApiBaseUrl();
  const response = await fetch(
    `${apiUrl}/personalization/content/${contentId}/translate?${params}`,
    {
      method: 'GET',
      headers,
    }
  );

  if (!response.ok) {
    const error = await response.json().catch(() => ({ message: 'Failed to translate content' }));
    throw new Error(error.message || `HTTP error! status: ${response.status}`);
  }

  return response.json();
}

/**
 * Get personalized content recommendations.
 */
export async function getRecommendations(
  limit: number = 5,
  token?: string
): Promise<RecommendationsResponse> {
  const headers: HeadersInit = {
    'Content-Type': 'application/json',
  };

  const authToken = token || getAuthToken();
  if (authToken) {
    headers['Authorization'] = `Bearer ${authToken}`;
  }

  const params = new URLSearchParams({
    limit: limit.toString(),
  });

  const response = await fetch(
    `${getApiBaseUrl()}/personalization/content/recommendations?${params}`,
    {
      method: 'GET',
      headers,
    }
  );

  if (!response.ok) {
    const error = await response.json().catch(() => ({ message: 'Failed to get recommendations' }));
    throw new Error(error.message || `HTTP error! status: ${response.status}`);
  }

  return response.json();
}

