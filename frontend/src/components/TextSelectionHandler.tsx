/**
 * Component to handle text selection and provide quick chatbot actions.
 */
import React, { useState, useEffect, useRef } from 'react';
import styles from './TextSelectionHandler.module.css';

interface TextSelectionHandlerProps {
  onTextSelected: (selectedText: string) => void;
}

export default function TextSelectionHandler({
  onTextSelected,
}: TextSelectionHandlerProps): JSX.Element | null {
  const [selectedText, setSelectedText] = useState<string>('');
  const [position, setPosition] = useState<{ top: number; left: number } | null>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim() || '';

      if (text.length > 0) {
        setSelectedText(text);
        
        // Get selection position
        const range = selection?.getRangeAt(0);
        if (range) {
          const rect = range.getBoundingClientRect();
          setPosition({
            top: rect.top + window.scrollY - 50,
            left: rect.left + window.scrollX + rect.width / 2,
          });
        }
      } else {
        setSelectedText('');
        setPosition(null);
      }
    };

    const handleClick = (e: MouseEvent) => {
      // Don't hide if clicking on the button itself
      if (buttonRef.current?.contains(e.target as Node)) {
        return;
      }
      
      // Small delay to allow selection to be processed
      setTimeout(() => {
        const selection = window.getSelection();
        if (!selection?.toString().trim()) {
          setSelectedText('');
          setPosition(null);
        }
      }, 100);
    };

    document.addEventListener('selectionchange', handleSelection);
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('click', handleClick);

    return () => {
      document.removeEventListener('selectionchange', handleSelection);
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('click', handleClick);
    };
  }, []);

  const handleAskQuestion = () => {
    if (selectedText) {
      onTextSelected(selectedText);
      setSelectedText('');
      setPosition(null);
      // Clear selection
      window.getSelection()?.removeAllRanges();
    }
  };

  if (!selectedText || !position) {
    return null;
  }

  return (
    <button
      ref={buttonRef}
      className={styles.selectionButton}
      style={{
        top: `${position.top}px`,
        left: `${position.left}px`,
      }}
      onClick={handleAskQuestion}
      title={`Ask about: "${selectedText.substring(0, 50)}${selectedText.length > 50 ? '...' : ''}"`}
    >
      💬 Ask AI about this
    </button>
  );
}





