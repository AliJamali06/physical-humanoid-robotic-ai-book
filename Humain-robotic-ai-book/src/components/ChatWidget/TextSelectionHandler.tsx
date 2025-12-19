import React, { useEffect, useState } from 'react';
import styles from './TextSelectionPopup.module.css';

interface TextSelectionHandlerProps {
  onAskQuestion: (selectedText: string) => void;
}

export default function TextSelectionHandler({ onAskQuestion }: TextSelectionHandlerProps): JSX.Element | null {
  const [selectedText, setSelectedText] = useState('');
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [show, setShow] = useState(false);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 10) {
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setSelectedText(text);
          setPosition({
            x: rect.left + rect.width / 2,
            y: rect.top - 10
          });
          setShow(true);
        }
      } else {
        setShow(false);
      }
    };

    const handleClickOutside = () => {
      setShow(false);
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('click', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  const handleAskClick = (e: React.MouseEvent) => {
    e.stopPropagation();
    onAskQuestion(selectedText);
    setShow(false);
  };

  if (!show) return null;

  return (
    <div 
      className={styles.selectionPopup}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
        transform: 'translate(-50%, -100%)'
      }}
    >
      <button onClick={handleAskClick} className={styles.askButton}>
        💬 Ask about this
      </button>
    </div>
  );
}
