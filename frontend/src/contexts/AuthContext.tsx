/**
 * Authentication context for managing user authentication state.
 */
import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { signIn, signUp, signOut, isAuthenticated, getAuthToken, AuthResponse, SignInRequest, SignUpRequest } from '../services/auth_api';

interface AuthContextType {
  isAuthenticated: boolean;
  isLoading: boolean;
  token: string | null;
  signIn: (data: SignInRequest) => Promise<AuthResponse>;
  signUp: (data: SignUpRequest) => Promise<AuthResponse>;
  signOut: () => Promise<void>;
  error: string | null;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function useAuth(): AuthContextType {
  const context = useContext(AuthContext);
  if (!context) {
    // Return a default context instead of throwing to prevent crashes
    return {
      isAuthenticated: false,
      isLoading: false,
      token: null,
      signIn: async () => ({ message: 'Not available' }),
      signUp: async () => ({ message: 'Not available' }),
      signOut: async () => {},
      error: null,
    };
  }
  return context;
}

interface AuthProviderProps {
  children: ReactNode;
}

export function AuthProvider({ children }: AuthProviderProps): JSX.Element {
  const [authenticated, setAuthenticated] = useState<boolean>(false);
  const [loading, setLoading] = useState<boolean>(true);
  const [token, setToken] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  // Check authentication status on mount
  useEffect(() => {
    const checkAuth = () => {
      try {
        const isAuth = isAuthenticated();
        const authToken = getAuthToken();
        setAuthenticated(isAuth);
        setToken(authToken);
      } catch (error) {
        console.error('Error checking auth status:', error);
        setAuthenticated(false);
        setToken(null);
      } finally {
        setLoading(false);
      }
    };

    // Only check auth in browser environment
    if (typeof window !== 'undefined') {
      checkAuth();
    } else {
      setLoading(false);
    }
  }, []);

  const handleSignIn = async (data: SignInRequest): Promise<AuthResponse> => {
    setError(null);
    try {
      const result = await signIn(data);
      setAuthenticated(true);
      setToken(getAuthToken());
      return result;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to sign in';
      setError(errorMessage);
      throw err;
    }
  };

  const handleSignUp = async (data: SignUpRequest): Promise<AuthResponse> => {
    setError(null);
    try {
      const result = await signUp(data);
      setAuthenticated(true);
      setToken(getAuthToken());
      return result;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to sign up';
      setError(errorMessage);
      throw err;
    }
  };

  const handleSignOut = async (): Promise<void> => {
    setError(null);
    try {
      await signOut();
      setAuthenticated(false);
      setToken(null);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to sign out';
      setError(errorMessage);
      // Still clear local state even if API call fails
      setAuthenticated(false);
      setToken(null);
    }
  };

  const value: AuthContextType = {
    isAuthenticated: authenticated,
    isLoading: loading,
    token,
    signIn: handleSignIn,
    signUp: handleSignUp,
    signOut: handleSignOut,
    error,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

