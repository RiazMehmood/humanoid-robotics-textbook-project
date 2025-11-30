/**
 * User Settings component for managing preferences.
 * Allows users to update language, theme, and other preferences.
 */
import React, { useState, useEffect } from 'react';
import {
  getPreferences,
  updatePreferences,
  UserPreferences,
  PreferencesUpdate,
} from '../services/personalization_api';
import { useAuth } from '../contexts/AuthContext';
import styles from './UserSettings.module.css';

export default function UserSettings(): JSX.Element {
  const [preferences, setPreferences] = useState<UserPreferences>({
    language: 'en',
    theme: 'light',
  });
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  // Get auth token from context
  const { token } = useAuth();
  
  const getAuthToken = (): string | null => {
    return token;
  };

  // Load preferences on mount
  useEffect(() => {
    loadPreferences();
  }, []);

  const loadPreferences = async () => {
    setIsLoading(true);
    setError(null);
    try {
      const token = getAuthToken();
      const prefs = await getPreferences(token || undefined);
      setPreferences(prefs);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load preferences');
    } finally {
      setIsLoading(false);
    }
  };

  const handleUpdate = async (updates: PreferencesUpdate) => {
    setIsSaving(true);
    setError(null);
    setSuccess(null);
    try {
      const token = getAuthToken();
      const result = await updatePreferences(updates, token || undefined);
      setPreferences(result.preferences);
      setSuccess('Preferences updated successfully!');
      // Clear success message after 3 seconds
      setTimeout(() => setSuccess(null), 3000);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to update preferences');
    } finally {
      setIsSaving(false);
    }
  };

  const handleLanguageChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newLanguage = e.target.value;
    handleUpdate({ language: newLanguage });
  };

  const handleThemeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newTheme = e.target.value;
    handleUpdate({ theme: newTheme });
    // Apply theme to document
    document.documentElement.setAttribute('data-theme', newTheme);
  };

  if (isLoading) {
    return (
      <div className={styles.settingsContainer}>
        <p>Loading preferences...</p>
      </div>
    );
  }

  return (
    <div className={styles.settingsContainer}>
      <h2>User Preferences</h2>
      
      {error && (
        <div className={styles.errorMessage}>
          <strong>Error:</strong> {error}
        </div>
      )}
      
      {success && (
        <div className={styles.successMessage}>
          {success}
        </div>
      )}

      <div className={styles.preferenceGroup}>
        <label htmlFor="language">
          <strong>Preferred Language:</strong>
        </label>
        <select
          id="language"
          value={preferences.language}
          onChange={handleLanguageChange}
          disabled={isSaving}
          className={styles.select}
        >
          <option value="en">English</option>
          <option value="es">Spanish (Español)</option>
          <option value="fr">French (Français)</option>
          <option value="de">German (Deutsch)</option>
          <option value="zh">Chinese (中文)</option>
          <option value="ja">Japanese (日本語)</option>
        </select>
        <p className={styles.helpText}>
          Content will be translated to your preferred language when available.
        </p>
      </div>

      <div className={styles.preferenceGroup}>
        <label htmlFor="theme">
          <strong>Theme:</strong>
        </label>
        <select
          id="theme"
          value={preferences.theme}
          onChange={handleThemeChange}
          disabled={isSaving}
          className={styles.select}
        >
          <option value="light">Light</option>
          <option value="dark">Dark</option>
          <option value="auto">Auto (System)</option>
        </select>
        <p className={styles.helpText}>
          Choose your preferred color theme for the textbook.
        </p>
      </div>

      {isSaving && (
        <div className={styles.savingIndicator}>
          Saving preferences...
        </div>
      )}
    </div>
  );
}

