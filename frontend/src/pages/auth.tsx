/**
 * Authentication page with Sign In and Sign Up.
 */
import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import SignIn from '../components/SignIn';
import SignUp from '../components/SignUp';
import { useAuth } from '../contexts/AuthContext';

export default function AuthPage(): JSX.Element {
  const [isSignUp, setIsSignUp] = useState(false);
  const { isAuthenticated } = useAuth();

  // Redirect if already authenticated
  useEffect(() => {
    if (isAuthenticated && typeof window !== 'undefined') {
      window.location.href = '/';
    }
  }, [isAuthenticated]);

  const handleSignUpSuccess = () => {
    // After sign up, switch to sign in page
    if (typeof window !== 'undefined') {
      setIsSignUp(false);
      // Show a message that they should sign in
      // The sign in will handle redirect to home
    }
  };

  const handleSignInSuccess = () => {
    // Redirect to home after successful sign in
    if (typeof window !== 'undefined') {
      window.location.href = '/';
    }
  };

  return (
    <Layout title="Sign In / Sign Up" description="Sign in or create an account to personalize your learning experience">
      <main style={{ padding: '2rem 0' }}>
        {isSignUp ? (
          <SignUp
            onSuccess={handleSignUpSuccess}
            onSwitchToSignIn={() => setIsSignUp(false)}
          />
        ) : (
          <SignIn
            onSuccess={handleSignInSuccess}
            onSwitchToSignUp={() => setIsSignUp(true)}
          />
        )}
      </main>
    </Layout>
  );
}

