/**
 * Root component that wraps all pages in Docusaurus.
 * This allows us to add global components like the Chatbot and AuthProvider.
 */
import React, { useState, useEffect } from 'react';
import Chatbot from '../components/Chatbot';
import TextSelectionHandler from '../components/TextSelectionHandler';
import AuthButton from '../components/AuthButton';
import { AuthProvider, useAuth } from '../contexts/AuthContext';
import styles from './Root.module.css';

function AuthGuard({ children }: { children: React.ReactNode }): JSX.Element {
  const { isAuthenticated, isLoading } = useAuth();
  const [isClient, setIsClient] = useState(false);
  const [showLoading, setShowLoading] = useState(false);

  // Only run on client to avoid hydration mismatch
  useEffect(() => {
    setIsClient(true);
  }, []);

  useEffect(() => {
    // Only redirect if not on auth page and not loading (client-side only)
    if (isClient && !isLoading && !isAuthenticated) {
      const currentPath = window.location.pathname;
      // Allow access to /auth, /settings, and /signout pages
      const allowedPaths = ['/auth', '/settings', '/signout'];
      const isAllowed = allowedPaths.some(path => currentPath === path || currentPath.startsWith(path));
      if (!isAllowed) {
        window.location.href = '/auth';
      }
    }
  }, [isClient, isAuthenticated, isLoading]);

  // Show loading only on client and only when needed
  useEffect(() => {
    if (isClient && isLoading && !isAuthenticated) {
      const currentPath = window.location.pathname;
      const allowedPaths = ['/auth', '/settings', '/signout'];
      const isAllowed = allowedPaths.some(path => currentPath === path || currentPath.startsWith(path));
      setShowLoading(!isAllowed);
    } else {
      setShowLoading(false);
    }
  }, [isClient, isLoading, isAuthenticated]);

  // Always render children to avoid hydration mismatch
  // Use CSS or state to control visibility
  if (showLoading) {
    return <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}>Loading...</div>;
  }

  return <>{children}</>;
}

export default function Root({children}: {children: React.ReactNode}): JSX.Element {
  const [isClient, setIsClient] = useState(false);

  // Only run on client to avoid hydration mismatch
  useEffect(() => {
    setIsClient(true);
    
    // The api-config.js script should have already set this
    // If not set, wait a bit for the script to load
    if (!(window as any).__API_BASE_URL__) {
      // Wait for api-config.js to load
      setTimeout(() => {
        if (!(window as any).__API_BASE_URL__) {
          const isProduction = window.location.hostname !== 'localhost' && 
                               window.location.hostname !== '127.0.0.1';
          if (isProduction) {
            console.error('API URL not configured. Please set REACT_APP_API_URL in Vercel environment variables and redeploy.');
          } else {
            (window as any).__API_BASE_URL__ = 'http://localhost:8000';
          }
        }
      }, 100);
    }
  }, []);

  const handleTextSelected = (text: string) => {
    // Pass to Chatbot via a custom event
    if (isClient) {
      window.dispatchEvent(new CustomEvent('textSelected', { detail: text }));
    }
  };

  // Always render the same structure to avoid hydration mismatch
  // Use CSS to hide/show on client
  return (
    <AuthProvider>
      <AuthGuard>
        {/* Add AuthButton to navbar via CSS injection - always render to avoid hydration issues */}
        <div className={styles.authButtonContainer} style={{ display: isClient ? 'block' : 'none' }}>
          <AuthButton />
        </div>
        {children}
        {/* Always render but hide on server to avoid hydration mismatch */}
        <div style={{ display: isClient ? 'block' : 'none' }}>
          <TextSelectionHandler onTextSelected={handleTextSelected} />
          <Chatbot />
        </div>
      </AuthGuard>
    </AuthProvider>
  );
}

