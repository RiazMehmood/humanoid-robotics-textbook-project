/**
 * Enhanced search component for textbook content.
 * This component provides a custom search interface that can integrate
 * with the backend API for advanced search capabilities.
 */
import React, { useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './Search.module.css';

interface SearchResult {
  id: string;
  title: string;
  content: string;
  url: string;
  module?: string;
  score?: number;
}

interface SearchProps {
  onResultSelect?: (result: SearchResult) => void;
  placeholder?: string;
}

export default function Search({
  onResultSelect,
  placeholder = 'Search textbook content...',
}: SearchProps): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<SearchResult[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [showResults, setShowResults] = useState(false);

  // Use Docusaurus built-in search if available
  useEffect(() => {
    // This component can be enhanced to call backend API for advanced search
    // For now, it provides a UI wrapper around Docusaurus search
    if (query.length > 2) {
      // Trigger search (Docusaurus handles this internally)
      setShowResults(true);
    } else {
      setShowResults(false);
      setResults([]);
    }
  }, [query]);

  const handleSearch = (e: React.ChangeEvent<HTMLInputElement>) => {
    setQuery(e.target.value);
  };

  const handleResultClick = (result: SearchResult) => {
    if (onResultSelect) {
      onResultSelect(result);
    }
    // Navigate to result
    window.location.href = result.url;
  };

  return (
    <div className={styles.searchContainer}>
      <div className={styles.searchInputWrapper}>
        <input
          type="text"
          className={styles.searchInput}
          placeholder={placeholder}
          value={query}
          onChange={handleSearch}
          onFocus={() => query.length > 2 && setShowResults(true)}
          onBlur={() => setTimeout(() => setShowResults(false), 200)}
        />
        <span className={styles.searchIcon}>🔍</span>
      </div>
      {showResults && results.length > 0 && (
        <div className={styles.searchResults}>
          {results.map((result) => (
            <div
              key={result.id}
              className={styles.searchResultItem}
              onClick={() => handleResultClick(result)}
            >
              <div className={styles.resultTitle}>{result.title}</div>
              {result.module && (
                <div className={styles.resultModule}>{result.module}</div>
              )}
              <div className={styles.resultSnippet}>
                {result.content.substring(0, 150)}...
              </div>
            </div>
          ))}
        </div>
      )}
      {isLoading && (
        <div className={styles.searchLoading}>Searching...</div>
      )}
    </div>
  );
}





