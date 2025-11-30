/**
 * Authentication button component for header/navigation.
 * Shows Sign In button when not authenticated, or user menu when authenticated.
 */
import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './AuthButton.module.css';

export default function AuthButton(): JSX.Element {
  const { isAuthenticated, signOut } = useAuth();
  const [showMenu, setShowMenu] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setShowMenu(false);
      }
    };

    if (showMenu) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [showMenu]);

  const handleSignIn = () => {
    if (typeof window !== 'undefined') {
      window.location.href = '/auth';
    }
  };

  const handleSignOut = async () => {
    try {
      setShowMenu(false);
      // Navigate to sign out page for better UX
      if (typeof window !== 'undefined') {
        window.location.href = '/signout';
      }
    } catch (error) {
      console.error('Sign out error:', error);
      // Fallback: clear localStorage and redirect
      if (typeof window !== 'undefined') {
        localStorage.removeItem('auth_token');
        window.location.href = '/';
      }
    }
  };

  const handleSettings = () => {
    setShowMenu(false);
    if (typeof window !== 'undefined') {
      window.location.href = '/settings';
    }
  };

  // Get user email from token if available
  const getUserEmail = (): string | null => {
    if (typeof window === 'undefined') return null;
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) return null;
      // Decode JWT token to get email (simple base64 decode)
      const payload = JSON.parse(atob(token.split('.')[1]));
      return payload.email || null;
    } catch (error) {
      return null;
    }
  };

  const userEmail = getUserEmail();

  if (!isAuthenticated) {
    return (
      <button onClick={handleSignIn} className={styles.signInButton}>
        <span className={styles.buttonIcon}>🔐</span>
        Sign In
      </button>
    );
  }

  return (
    <div className={styles.authMenu} ref={menuRef}>
      <button
        onClick={() => setShowMenu(!showMenu)}
        className={styles.userButton}
        aria-label="User menu"
        aria-expanded={showMenu}
      >
        <span className={styles.userIcon}>👤</span>
        <span className={styles.userText}>Account</span>
        <span className={styles.dropdownIcon}>{showMenu ? '▲' : '▼'}</span>
      </button>
      {showMenu && (
        <div className={styles.menu}>
          <div className={styles.menuHeader}>
            <span className={styles.menuHeaderIcon}>✓</span>
            <span className={styles.menuHeaderText}>Signed In</span>
          </div>
          {userEmail && (
            <>
              <div className={styles.menuEmail}>{userEmail}</div>
              <div className={styles.menuDivider}></div>
            </>
          )}
          <button onClick={handleSettings} className={styles.menuItem}>
            <span className={styles.menuItemIcon}>⚙️</span>
            Settings
          </button>
          <button onClick={handleSignOut} className={styles.menuItem}>
            <span className={styles.menuItemIcon}>🚪</span>
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}

