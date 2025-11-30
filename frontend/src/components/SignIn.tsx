/**
 * Sign In component for user authentication.
 */
import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './SignIn.module.css';

interface SignInProps {
  onSuccess?: () => void;
  onSwitchToSignUp?: () => void;
}

export default function SignIn({ onSuccess, onSwitchToSignUp }: SignInProps): JSX.Element {
  const { signIn, error: authError } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsLoading(true);

    try {
      await signIn({ email, password });
      if (onSuccess) {
        onSuccess();
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to sign in');
    } finally {
      setIsLoading(false);
    }
  };

  const displayError = error || authError;

  return (
    <div className={styles.signInContainer}>
      <h2>Sign In</h2>
      <p className={styles.subtitle}>Sign in to access personalized features</p>

      {displayError && (
        <div className={styles.errorMessage}>
          <strong>Error:</strong> {displayError}
        </div>
      )}

      <form onSubmit={handleSubmit} className={styles.form}>
        <div className={styles.formGroup}>
          <label htmlFor="email">Email:</label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            disabled={isLoading}
            className={styles.input}
            placeholder="your.email@example.com"
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password:</label>
          <input
            id="password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            disabled={isLoading}
            className={styles.input}
            placeholder="Enter your password"
            minLength={6}
          />
        </div>

        <button
          type="submit"
          disabled={isLoading}
          className={styles.submitButton}
        >
          {isLoading ? 'Signing in...' : 'Sign In'}
        </button>
      </form>

      {onSwitchToSignUp && (
        <div className={styles.switchAuth}>
          <p>
            Don't have an account?{' '}
            <button
              type="button"
              onClick={onSwitchToSignUp}
              className={styles.switchButton}
            >
              Sign Up
            </button>
          </p>
        </div>
      )}
    </div>
  );
}


