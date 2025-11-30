/**
 * Chatbot UI component for textbook Q&A.
 * Provides an interactive chat interface for asking questions about the textbook content.
 */
import React, { useState, useRef, useEffect } from 'react';
import { chatbotAPI, ChatbotResponse } from '../services/chatbot_api';
import styles from './Chatbot.module.css';

interface Message {
  id: string;
  text: string;
  isUser: boolean;
  timestamp: Date;
  sourceReferences?: string[];
  confidenceScore?: number;
}

export default function Chatbot(): JSX.Element {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Extract query handling logic
  const handleQuery = async (queryText: string) => {
    setInputText('');
    setError(null);

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: queryText,
      isUser: true,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Submit query with context from current page
      const response: ChatbotResponse = await chatbotAPI.submitQueryWithContext(
        queryText
      );

      // Add bot response
      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: response.responseText,
        isUser: false,
        timestamp: new Date(),
        sourceReferences: response.sourceReferences,
        confidenceScore: response.confidenceScore,
      };
      setMessages((prev) => [...prev, botMessage]);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to get response';
      setError(errorMessage);
      
      // Add error message
      const errorMsg: Message = {
        id: (Date.now() + 1).toString(),
        text: `Sorry, I encountered an error: ${errorMessage}`,
        isUser: false,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, errorMsg]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle text selection from TextSelectionHandler via custom event
  useEffect(() => {
    const handleTextSelectedEvent = (e: CustomEvent) => {
      const selectedText = e.detail as string;
      // Open chatbot if closed
      setIsOpen(true);
      
      // Auto-submit query about selected text after a delay
      setTimeout(() => {
        const queryText = `What is "${selectedText}"? Please explain or define it.`;
        handleQuery(queryText);
      }, 300);
    };

    window.addEventListener('textSelected', handleTextSelectedEvent as EventListener);
    return () => {
      window.removeEventListener('textSelected', handleTextSelectedEvent as EventListener);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []); // Empty deps - handleQuery is stable

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!inputText.trim() || isLoading) {
      return;
    }

    const queryText = inputText.trim();
    await handleQuery(queryText);
  };

  const handleClear = () => {
    setMessages([]);
    setError(null);
  };

  return (
    <div className={styles.chatbotContainer}>
      {/* Chatbot Toggle Button */}
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chatbot' : 'Open chatbot'}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chatbot Panel */}
      {isOpen && (
        <div className={styles.chatbotPanel}>
          <div className={styles.chatbotHeader}>
            <h3>Textbook AI Assistant</h3>
            <div className={styles.headerActions}>
              <button
                className={styles.clearButton}
                onClick={handleClear}
                title="Clear conversation"
              >
                🗑️
              </button>
              <button
                className={styles.closeButton}
                onClick={() => setIsOpen(false)}
                title="Close"
              >
                ✕
              </button>
            </div>
          </div>

          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook.</p>
                <p>Ask me anything about the course content!</p>
              </div>
            )}
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${
                  message.isUser ? styles.userMessage : styles.botMessage
                }`}
              >
                <div className={styles.messageContent}>
                  <p>{message.text}</p>
                  {!message.isUser && message.sourceReferences && message.sourceReferences.length > 0 && (
                    <div className={styles.sources}>
                      <strong>Sources:</strong>
                      <ul>
                        {message.sourceReferences.map((ref, idx) => (
                          <li key={idx}>
                            <a href={ref} target="_blank" rel="noopener noreferrer">
                              {ref}
                            </a>
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                  {!message.isUser && message.confidenceScore !== undefined && (
                    <div className={styles.confidence}>
                      Confidence: {(message.confidenceScore * 100).toFixed(0)}%
                    </div>
                  )}
                </div>
                <div className={styles.messageTime}>
                  {message.timestamp.toLocaleTimeString()}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.messageContent}>
                  <div className={styles.loadingDots}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            {error && (
              <div className={styles.errorMessage}>
                ⚠️ {error}
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className={styles.inputForm} onSubmit={handleSubmit}>
            <input
              ref={inputRef}
              type="text"
              className={styles.inputField}
              placeholder="Ask a question about the textbook..."
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputText.trim()}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
}

