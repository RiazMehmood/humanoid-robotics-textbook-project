/**
 * Sign out page - allows users to sign out directly.
 */
import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';

export default function SignOutPage(): JSX.Element {
  const { signOut, isAuthenticated } = useAuth();

  useEffect(() => {
    const handleSignOut = async () => {
      if (isAuthenticated) {
        try {
          await signOut();
          // Redirect to home after sign out
          if (typeof window !== 'undefined') {
            setTimeout(() => {
              window.location.href = '/';
            }, 1000);
          }
        } catch (error) {
          console.error('Sign out error:', error);
          // Still redirect even if API call fails
          if (typeof window !== 'undefined') {
            setTimeout(() => {
              window.location.href = '/';
            }, 1000);
          }
        }
      } else {
        // Already signed out, redirect to home
        if (typeof window !== 'undefined') {
          window.location.href = '/';
        }
      }
    };

    handleSignOut();
  }, [isAuthenticated, signOut]);

  return (
    <Layout title="Signing Out" description="Signing out...">
      <main style={{ padding: '2rem', textAlign: 'center' }}>
        <h1>Signing Out...</h1>
        <p>Please wait while we sign you out.</p>
      </main>
    </Layout>
  );
}


