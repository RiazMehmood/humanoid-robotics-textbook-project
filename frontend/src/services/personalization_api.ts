/**
 * Frontend service for personalization API interactions.
 */
import { getAuthToken } from './auth_api';

// Get API base URL (browser-compatible)
function getApiBaseUrl(): string {
  if (typeof window !== 'undefined' && (window as any).__API_BASE_URL__) {
    return (window as any).__API_BASE_URL__;
  }
  // Only access process.env in browser environment
  if (typeof window !== 'undefined' && typeof process !== 'undefined' && process.env) {
    return process.env.REACT_APP_API_URL || 'http://localhost:8000';
  }
  return 'http://localhost:8000';
}

const API_BASE_URL = getApiBaseUrl();

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

  const response = await fetch(`${API_BASE_URL}/personalization/preferences`, {
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

  const response = await fetch(`${API_BASE_URL}/personalization/preferences`, {
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

  const response = await fetch(
    `${API_BASE_URL}/personalization/content/${contentId}/translate?${params}`,
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
    `${API_BASE_URL}/personalization/content/recommendations?${params}`,
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

