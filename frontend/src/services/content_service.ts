/**
 * Content service for fetching textbook content from the backend API.
 * This service handles dynamic content fetching, caching, and content personalization.
 */

// Get API URL from window object or use default - shared logic with auth_api.ts
const getApiBaseUrl = (): string => {
  if (typeof window === 'undefined') {
    return 'http://localhost:8000/api/v1';
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
      return `${cleanUrl}/api/v1`;
    }
  }
  
  const isProduction = window.location.hostname !== 'localhost' && 
                       window.location.hostname !== '127.0.0.1';
  return isProduction ? 'API_URL_NOT_CONFIGURED' : 'http://localhost:8000/api/v1';
};

export interface ContentItem {
  id: string;
  title: string;
  content: string;
  module: string;
  chapter: string;
  section?: string;
  metadata?: {
    keywords?: string[];
    difficulty?: string;
    estimatedReadTime?: number;
  };
}

export interface ContentSearchParams {
  query: string;
  module?: string;
  limit?: number;
  offset?: number;
}

export interface ContentSearchResult {
  items: ContentItem[];
  total: number;
  query: string;
}

class ContentService {
  private cache: Map<string, ContentItem> = new Map();
  private cacheExpiry: Map<string, number> = new Map();
  private readonly CACHE_TTL = 5 * 60 * 1000; // 5 minutes

  /**
   * Get content by ID
   */
  async getContentById(id: string): Promise<ContentItem | null> {
    // Check cache first
    const cached = this.getCachedContent(id);
    if (cached) {
      return cached;
    }

    try {
      const apiUrl = getApiBaseUrl();
      const response = await fetch(`${apiUrl}/content/${id}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        if (response.status === 404) {
          return null;
        }
        throw new Error(`Failed to fetch content: ${response.statusText}`);
      }

      const content: ContentItem = await response.json();
      this.cacheContent(id, content);
      return content;
    } catch (error) {
      console.error('Error fetching content:', error);
      // Fallback to static content if API fails
      return this.getStaticContent(id);
    }
  }

  /**
   * Search content
   */
  async searchContent(params: ContentSearchParams): Promise<ContentSearchResult> {
    try {
      const queryParams = new URLSearchParams({
        q: params.query,
        ...(params.module && { module: params.module }),
        ...(params.limit && { limit: params.limit.toString() }),
        ...(params.offset && { offset: params.offset.toString() }),
      });

      const apiUrl = getApiBaseUrl();
      const response = await fetch(`${apiUrl}/content/search?${queryParams}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`Search failed: ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error searching content:', error);
      // Return empty results on error
      return {
        items: [],
        total: 0,
        query: params.query,
      };
    }
  }

  /**
   * Get personalized content recommendations
   */
  async getPersonalizedContent(userId?: string): Promise<ContentItem[]> {
    try {
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
      };

      if (userId) {
        // Add authentication token if available
        const token = localStorage.getItem('auth_token');
        if (token) {
          headers['Authorization'] = `Bearer ${token}`;
        }
      }

      const apiUrl = getApiBaseUrl();
      const response = await fetch(
        `${apiUrl}/content/personalized${userId ? `?user_id=${userId}` : ''}`,
        {
          method: 'GET',
          headers,
        }
      );

      if (!response.ok) {
        throw new Error(`Failed to fetch personalized content: ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error fetching personalized content:', error);
      return [];
    }
  }

  /**
   * Get content by module and chapter
   */
  async getContentByPath(module: string, chapter: string): Promise<ContentItem | null> {
    const id = `${module}/${chapter}`;
    return this.getContentById(id);
  }

  /**
   * Cache management
   */
  private cacheContent(id: string, content: ContentItem): void {
    this.cache.set(id, content);
    this.cacheExpiry.set(id, Date.now() + this.CACHE_TTL);
  }

  private getCachedContent(id: string): ContentItem | null {
    const expiry = this.cacheExpiry.get(id);
    if (expiry && expiry > Date.now()) {
      return this.cache.get(id) || null;
    }
    // Remove expired cache
    this.cache.delete(id);
    this.cacheExpiry.delete(id);
    return null;
  }

  /**
   * Fallback to static content (for offline/development)
   */
  private async getStaticContent(id: string): Promise<ContentItem | null> {
    // This would load from static markdown files
    // For now, return null to indicate content not found
    return null;
  }

  /**
   * Clear cache
   */
  clearCache(): void {
    this.cache.clear();
    this.cacheExpiry.clear();
  }
}

// Export singleton instance
export const contentService = new ContentService();

