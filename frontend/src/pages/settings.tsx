/**
 * User Settings page.
 */
import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import UserSettings from '../components/UserSettings';

export default function SettingsPage(): JSX.Element {
  const { isAuthenticated } = useAuth();

  // Redirect if not authenticated
  useEffect(() => {
    if (!isAuthenticated && typeof window !== 'undefined') {
      window.location.href = '/auth';
    }
  }, [isAuthenticated]);

  if (!isAuthenticated) {
    return null;
  }

  return (
    <Layout title="User Settings" description="Manage your preferences and personalize your learning experience">
      <main style={{ padding: '2rem 0' }}>
        <UserSettings />
      </main>
    </Layout>
  );
}

