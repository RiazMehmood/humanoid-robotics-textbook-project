/**
 * Sign Up component for user registration.
 */
import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './SignUp.module.css';

interface SignUpProps {
  onSuccess?: () => void;
  onSwitchToSignIn?: () => void;
}

export default function SignUp({ onSuccess, onSwitchToSignIn }: SignUpProps): JSX.Element {
  const { signUp, error: authError } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  
  // Background questions
  const [softwareExp, setSoftwareExp] = useState('');
  const [hardwareExp, setHardwareExp] = useState('');
  const [roboticsExp, setRoboticsExp] = useState('');
  const [programmingLanguages, setProgrammingLanguages] = useState('');
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    // Validation
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (password.length < 6) {
      setError('Password must be at least 6 characters long');
      return;
    }

    setIsLoading(true);

    try {
      await signUp({ 
        email, 
        password,
        background_info: {
          software_experience: softwareExp,
          hardware_experience: hardwareExp,
          robotics_experience: roboticsExp,
          programming_languages: programmingLanguages
        }
      });
      // Show success message and then switch to sign in
      setSuccessMessage('Account created successfully! Please sign in.');
      setTimeout(() => {
        if (onSuccess) {
          onSuccess();
        }
      }, 2000); // Wait 2 seconds before switching to sign in
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to sign up');
    } finally {
      setIsLoading(false);
    }
  };

  const displayError = error || authError;

  return (
    <div className={styles.signUpContainer}>
      <h2>Sign Up</h2>
      <p className={styles.subtitle}>Create an account to personalize your learning experience</p>

      {displayError && (
        <div className={styles.errorMessage}>
          <strong>Error:</strong> {displayError}
        </div>
      )}

      {successMessage && (
        <div style={{ 
          padding: '1rem', 
          backgroundColor: '#d4edda', 
          color: '#155724', 
          borderRadius: '4px', 
          marginBottom: '1rem',
          border: '1px solid #c3e6cb'
        }}>
          <strong>Success:</strong> {successMessage}
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
            placeholder="At least 6 characters"
            minLength={6}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="confirmPassword">Confirm Password:</label>
          <input
            id="confirmPassword"
            type="password"
            value={confirmPassword}
            onChange={(e) => setConfirmPassword(e.target.value)}
            required
            disabled={isLoading}
            className={styles.input}
            placeholder="Re-enter your password"
            minLength={6}
          />
        </div>

        <div className={styles.sectionDivider}>
          <h3>Background Information</h3>
          <p className={styles.sectionDescription}>Help us personalize your learning experience</p>
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="softwareExp">Software Experience:</label>
          <textarea
            id="softwareExp"
            value={softwareExp}
            onChange={(e) => setSoftwareExp(e.target.value)}
            disabled={isLoading}
            className={styles.textarea}
            placeholder="Describe your software development experience (e.g., web development, mobile apps, etc.)"
            rows={3}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="hardwareExp">Hardware Experience:</label>
          <textarea
            id="hardwareExp"
            value={hardwareExp}
            onChange={(e) => setHardwareExp(e.target.value)}
            disabled={isLoading}
            className={styles.textarea}
            placeholder="Describe your hardware experience (e.g., Arduino, Raspberry Pi, embedded systems, etc.)"
            rows={3}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="roboticsExp">Robotics Experience:</label>
          <textarea
            id="roboticsExp"
            value={roboticsExp}
            onChange={(e) => setRoboticsExp(e.target.value)}
            disabled={isLoading}
            className={styles.textarea}
            placeholder="Describe your robotics experience (e.g., ROS, robot programming, etc.)"
            rows={3}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="programmingLanguages">Programming Languages:</label>
          <input
            id="programmingLanguages"
            type="text"
            value={programmingLanguages}
            onChange={(e) => setProgrammingLanguages(e.target.value)}
            disabled={isLoading}
            className={styles.input}
            placeholder="e.g., Python, C++, JavaScript, etc."
          />
        </div>

        <button
          type="submit"
          disabled={isLoading}
          className={styles.submitButton}
        >
          {isLoading ? 'Creating account...' : 'Sign Up'}
        </button>
      </form>

      {onSwitchToSignIn && (
        <div className={styles.switchAuth}>
          <p>
            Already have an account?{' '}
            <button
              type="button"
              onClick={onSwitchToSignIn}
              className={styles.switchButton}
            >
              Sign In
            </button>
          </p>
        </div>
      )}
    </div>
  );
}

