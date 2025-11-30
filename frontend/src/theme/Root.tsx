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

  useEffect(() => {
    // Only redirect if not on auth page and not loading
    if (!isLoading && !isAuthenticated && typeof window !== 'undefined') {
      const currentPath = window.location.pathname;
      // Allow access to /auth, /settings, and /signout pages
      const allowedPaths = ['/auth', '/settings', '/signout'];
      const isAllowed = allowedPaths.some(path => currentPath === path || currentPath.startsWith(path));
      if (!isAllowed) {
        window.location.href = '/auth';
      }
    }
  }, [isAuthenticated, isLoading]);

  // Show nothing while checking auth (prevents flash)
  if (isLoading && !isAuthenticated && typeof window !== 'undefined') {
    const currentPath = window.location.pathname;
    const allowedPaths = ['/auth', '/settings', '/signout'];
    const isAllowed = allowedPaths.some(path => currentPath === path || currentPath.startsWith(path));
    if (!isAllowed) {
      return <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}>Loading...</div>;
    }
  }

  return <>{children}</>;
}

export default function Root({children}: {children: React.ReactNode}): JSX.Element {
  const handleTextSelected = (text: string) => {
    // Pass to Chatbot via a custom event
    if (typeof window !== 'undefined') {
      window.dispatchEvent(new CustomEvent('textSelected', { detail: text }));
    }
  };

  return (
    <AuthProvider>
      <AuthGuard>
        {/* Add AuthButton to navbar via CSS injection */}
        {typeof window !== 'undefined' && (
          <div className={styles.authButtonContainer}>
            <AuthButton />
          </div>
        )}
        {children}
        {typeof window !== 'undefined' && (
          <>
            <TextSelectionHandler onTextSelected={handleTextSelected} />
            <Chatbot />
          </>
        )}
      </AuthGuard>
    </AuthProvider>
  );
}

