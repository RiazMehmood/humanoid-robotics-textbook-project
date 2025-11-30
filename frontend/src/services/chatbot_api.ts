/**
 * Chatbot API service for interacting with the backend chatbot endpoint.
 * Handles query submission, context passing, and response handling.
 */

// Get API URL from window object or use default
const getApiBaseUrl = (): string => {
  // Check if running in browser and if API URL is set in window
  if (typeof window !== 'undefined' && (window as any).__API_BASE_URL__) {
    return (window as any).__API_BASE_URL__;
  }
  // Default to localhost for development
  return 'http://localhost:8000';
};

const API_BASE_URL = getApiBaseUrl();

export interface ChatbotQueryRequest {
  queryText: string;
  context?: {
    currentPageUrl?: string;
    selectedText?: string;
  };
}

export interface ChatbotResponse {
  responseText: string;
  sourceReferences: string[];
  confidenceScore?: number;
}

class ChatbotAPI {
  private getAuthToken(): string | null {
    return localStorage.getItem('auth_token');
  }

  private async makeRequest<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const token = this.getAuthToken();
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      ...options.headers,
    };

    if (token) {
      headers['Authorization'] = `Bearer ${token}`;
    }

    try {
      const response = await fetch(`${API_BASE_URL}${endpoint}`, {
        ...options,
        headers,
      });

      if (!response.ok) {
        const error = await response.json().catch(() => ({
          code: 'UNKNOWN_ERROR',
          message: `HTTP ${response.status}: ${response.statusText}`,
        }));
        throw new Error(error.message || `Server error: ${response.status}`);
      }

      return response.json();
    } catch (error) {
      // Handle network errors (backend not running, CORS, etc.)
      if (error instanceof TypeError && error.message.includes('fetch')) {
        throw new Error(
          `Cannot connect to backend server at ${API_BASE_URL}. ` +
          `Please make sure the backend is running on port 8000. ` +
          `Start it with: cd backend && uvicorn main:app --reload`
        );
      }
      throw error;
    }
  }

  /**
   * Submit a query to the chatbot
   */
  async submitQuery(
    queryText: string,
    context?: {
      currentPageUrl?: string;
      selectedText?: string;
    }
  ): Promise<ChatbotResponse> {
    const request: ChatbotQueryRequest = {
      queryText,
      context,
    };

    return this.makeRequest<ChatbotResponse>('/chatbot/query', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

  /**
   * Get context from current page
   */
  getPageContext(): {
    currentPageUrl: string;
    selectedText?: string;
  } {
    const currentPageUrl = window.location.href;
    const selectedText = window.getSelection()?.toString().trim();

    return {
      currentPageUrl,
      ...(selectedText && { selectedText }),
    };
  }

  /**
   * Submit query with automatic context from current page
   */
  async submitQueryWithContext(queryText: string): Promise<ChatbotResponse> {
    const context = this.getPageContext();
    return this.submitQuery(queryText, context);
  }
}

// Export singleton instance
export const chatbotAPI = new ChatbotAPI();

