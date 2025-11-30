import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './index.module.css';
import HomepageFeatures from '../components/HomepageFeatures';
import { getRecommendations, ContentRecommendation } from '../services/personalization_api';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <a className="button button--secondary button--lg" href="/intro">
            Start Learning - Begin Your Journey 🚀
          </a>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const [recommendations, setRecommendations] = useState<ContentRecommendation[]>([]);
  const [isLoadingRecommendations, setIsLoadingRecommendations] = useState(false);

  // Get auth token from context
  const { token } = useAuth();
  
  const getAuthToken = (): string | null => {
    return token;
  };

  // Load personalized recommendations on mount
  useEffect(() => {
    loadRecommendations();
  }, []);

  const loadRecommendations = async () => {
    setIsLoadingRecommendations(true);
    try {
      const token = getAuthToken();
      const result = await getRecommendations(5, token || undefined);
      setRecommendations(result.recommendations);
    } catch (err) {
      // Silently fail - recommendations are optional
      console.debug('Failed to load recommendations:', err);
    } finally {
      setIsLoadingRecommendations(false);
    }
  };

  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Learn Physical AI and Humanoid Robotics - A comprehensive textbook covering ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems">
      <HomepageHeader />
      <main>
        {recommendations.length > 0 && (
          <section className={styles.recommendationsSection}>
            <div className="container">
              <Heading as="h2" className={styles.recommendationsTitle}>
                Recommended for You
              </Heading>
              <div className={styles.recommendationsGrid}>
                {recommendations.map((rec) => (
                  <a
                    key={rec.contentId}
                    href={`/docs/${rec.contentId}`}
                    className={styles.recommendationCard}
                  >
                    <h3>{rec.title}</h3>
                    {rec.reason && <p className={styles.recommendationReason}>{rec.reason}</p>}
                  </a>
                ))}
              </div>
            </div>
          </section>
        )}
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
