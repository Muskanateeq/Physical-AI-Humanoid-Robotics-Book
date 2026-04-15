import React, { useState, KeyboardEvent } from 'react';

interface MessageInputProps {
  onSend: (message: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

export default function MessageInput({
  onSend,
  disabled = false,
  placeholder = "Ask anything about Physical AI and Robotics...",
}: MessageInputProps) {
  const [input, setInput] = useState('');

  const handleSend = () => {
    if (input.trim() && !disabled) {
      onSend(input.trim());
      setInput('');
    }
  };

  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div
      style={{
        padding: '1rem',
        borderTop: '1px solid #5848b8',
        background: '#5848b8',
        display: 'flex',
        gap: '0.5rem',
        alignItems: 'flex-end',
      }}
    >
      <textarea
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder={placeholder}
        disabled={disabled}
        rows={1}
        style={{
          flex: 1,
          padding: '0.75rem',
          background: 'rgba(15, 23, 42, 0.95)',
          color: '#ffffff',
          border: '1px solid #6a5acd',
          borderRadius: '0.5rem',
          fontSize: '0.9rem',
          resize: 'none',
          outline: 'none',
          fontFamily: 'inherit',
          maxHeight: '120px',
          overflowY: 'auto',
        }}
      />
      <button
        onClick={handleSend}
        disabled={disabled || !input.trim()}
        style={{
          padding: '0.75rem 1.25rem',
          background: disabled ? '#4a4a5c' : '#b8860b',
          color: '#ffffff',
          border: 'none',
          borderRadius: '0.5rem',
          cursor: disabled ? 'not-allowed' : 'pointer',
          fontSize: '0.9rem',
          fontWeight: '500',
          transition: 'all 0.2s',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
        }}
        onMouseOver={(e) => {
          if (!disabled) {
            e.currentTarget.style.background = '#daa520';
          }
        }}
        onMouseOut={(e) => {
          if (!disabled) {
            e.currentTarget.style.background = '#b8860b';
          }
        }}
      >
        <svg
          width="20"
          height="20"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <line x1="22" y1="2" x2="11" y2="13"></line>
          <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
        </svg>
      </button>
    </div>
  );
}
