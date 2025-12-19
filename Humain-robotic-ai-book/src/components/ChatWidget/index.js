import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

// Use window.location to dynamically set backend URL
const BACKEND_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:8001'
  : process.env.REACT_APP_BACKEND_URL || 'http://localhost:8001';

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: '👋 Hi! I\'m your AI assistant for the Physical AI & Humanoid Robotics textbook. Ask me anything about ROS 2, Isaac Sim, Gazebo, Unity, or Vision-Language-Action pipelines!'
    }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch(`${BACKEND_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: input,
          selected_text: selectedText || null,
          conversation_history: messages.slice(-6).map(m => ({
            role: m.role,
            content: m.content
          }))
        })
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();

      const assistantMessage = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources
      };

      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText(''); // Clear selected text after use

    } catch (error) {
      const errorMessage = {
        role: 'assistant',
        content: '❌ Sorry, I encountered an error. Please make sure the backend is running or try again later.'
      };
      setMessages(prev => [...prev, errorMessage]);
    }

    setLoading(false);
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className={styles.chatWidget}>
      {/* Chat Button */}
      <button 
        className={styles.chatButton} 
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <h3>🤖 Robotics AI Assistant</h3>
              <p>Ask me about the textbook</p>
            </div>
            <button 
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messages}>
            {messages.map((msg, idx) => (
              <div key={idx} className={styles[msg.role]}>
                <div className={styles.messageContent}>
                  {msg.content}
                </div>
                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sources}>
                    <details>
                      <summary>📚 Sources ({msg.sources.length})</summary>
                      <ul>
                        {msg.sources.map((source, i) => (
                          <li key={i}>
                            <strong>{source.module}</strong> - {source.chapter}
                            <span className={styles.score}>{(source.score * 100).toFixed(0)}% match</span>
                            <p>{source.text}</p>
                          </li>
                        ))}
                      </ul>
                    </details>
                  </div>
                )}
              </div>
            ))}
            {loading && (
              <div className={styles.assistant}>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Selected Text Preview */}
          {selectedText && (
            <div className={styles.selectedTextPreview}>
              <span>📌 Selected: "{selectedText.substring(0, 50)}..."</span>
              <button onClick={() => setSelectedText('')}>✕</button>
            </div>
          )}

          {/* Input Area */}
          <div className={styles.inputArea}>
            <textarea
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the textbook..."
              rows={2}
              disabled={loading}
            />
            <button 
              onClick={sendMessage}
              disabled={loading || !input.trim()}
              className={styles.sendButton}
            >
              {loading ? '⏳' : '➤'}
            </button>
          </div>

          {/* Quick Actions */}
          <div className={styles.quickActions}>
            <button onClick={() => setInput('What is ROS 2?')}>
              What is ROS 2?
            </button>
            <button onClick={() => setInput('Explain Vision-Language-Action')}>
              VLA Pipeline
            </button>
            <button onClick={() => setInput('How does Isaac Sim work?')}>
              Isaac Sim
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
