/**
 * Content Renderer component for displaying translated content.
 * Integrates with translation API to show content in user's preferred language.
 */
import React, { useState, useEffect } from 'react';
import { translateContent, TranslationResponse } from '../services/personalization_api';
import { useAuth } from '../contexts/AuthContext';
import styles from './ContentRenderer.module.css';

interface ContentRendererProps {
  contentId: string;
  originalContent: string;
  sourceLanguage?: string;
  targetLanguage?: string;
  autoTranslate?: boolean;
}

export default function ContentRenderer({
  contentId,
  originalContent,
  sourceLanguage = 'en',
  targetLanguage,
  autoTranslate = false,
}: ContentRendererProps): JSX.Element {
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationError, setTranslationError] = useState<string | null>(null);
  const [currentLanguage, setCurrentLanguage] = useState(sourceLanguage);

  // Get auth token from context
  const { token } = useAuth();
  
  const getAuthToken = (): string | null => {
    return token;
  };

  // Auto-translate on mount if enabled
  useEffect(() => {
    if (autoTranslate && targetLanguage && targetLanguage !== sourceLanguage) {
      handleTranslate(targetLanguage);
    }
  }, [autoTranslate, targetLanguage, sourceLanguage]);

  const handleTranslate = async (lang: string) => {
    if (lang === currentLanguage) {
      return; // Already in this language
    }

    setIsTranslating(true);
    setTranslationError(null);

    try {
      const token = getAuthToken();
      const result: TranslationResponse = await translateContent(
        contentId,
        lang,
        sourceLanguage,
        token || undefined
      );

      setTranslatedContent(result.translatedText);
      setCurrentLanguage(lang);
    } catch (err) {
      setTranslationError(
        err instanceof Error ? err.message : 'Failed to translate content'
      );
      // Fall back to original content
      setTranslatedContent(null);
      setCurrentLanguage(sourceLanguage);
    } finally {
      setIsTranslating(false);
    }
  };

  const displayContent = translatedContent || originalContent;
  const isTranslated = translatedContent !== null;

  return (
    <div className={styles.contentRenderer}>
      {isTranslated && (
        <div className={styles.translationBadge}>
          <span>Translated to {currentLanguage.toUpperCase()}</span>
          <button
            onClick={() => {
              setTranslatedContent(null);
              setCurrentLanguage(sourceLanguage);
            }}
            className={styles.showOriginalButton}
          >
            Show Original
          </button>
        </div>
      )}

      {isTranslating && (
        <div className={styles.translatingIndicator}>
          Translating content...
        </div>
      )}

      {translationError && (
        <div className={styles.errorMessage}>
          <strong>Translation Error:</strong> {translationError}
        </div>
      )}

      <div
        className={styles.content}
        dangerouslySetInnerHTML={{ __html: displayContent }}
      />

      {!isTranslated && targetLanguage && targetLanguage !== sourceLanguage && (
        <div className={styles.translateButtonContainer}>
          <button
            onClick={() => handleTranslate(targetLanguage)}
            disabled={isTranslating}
            className={styles.translateButton}
          >
            Translate to {targetLanguage.toUpperCase()}
          </button>
        </div>
      )}
    </div>
  );
}

