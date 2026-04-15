import React, { useEffect, useRef } from 'react';
import { Message } from '../../hooks/useCustomChat';

interface MessageListProps {
  messages: Message[];
  isLoading: boolean;
}

export default function MessageList({ messages, isLoading }: MessageListProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Check if last message is being streamed (empty or partial content)
  const lastMessage = messages[messages.length - 1];
  const isStreaming = isLoading && lastMessage?.role === 'assistant';

  return (
    <div
      style={{
        flex: 1,
        overflowY: 'auto',
        padding: '1rem',
        display: 'flex',
        flexDirection: 'column',
        gap: '1rem',
        background: 'rgba(15, 23, 42, 0.95)',
      }}
    >
      {messages.map((message) => (
        <div
          key={message.id}
          style={{
            display: 'flex',
            justifyContent: message.role === 'user' ? 'flex-end' : 'flex-start',
          }}
        >
          <div
            style={{
              maxWidth: '80%',
              padding: '0.75rem 1rem',
              borderRadius: '0.75rem',
              background: message.role === 'user' ? '#6a5acd' : '#5848b8',
              color: '#ffffff',
              fontSize: '0.9rem',
              lineHeight: '1.5',
              wordWrap: 'break-word',
              whiteSpace: 'pre-wrap',
            }}
          >
            {message.content || (
              // Show typing indicator only for empty assistant messages
              message.role === 'assistant' && (
                <div
                  style={{
                    display: 'flex',
                    gap: '0.25rem',
                    alignItems: 'center',
                  }}
                >
                  <span
                    style={{
                      width: '8px',
                      height: '8px',
                      borderRadius: '50%',
                      background: '#6a5acd',
                      animation: 'pulse 1.5s ease-in-out infinite',
                    }}
                  />
                  <span
                    style={{
                      width: '8px',
                      height: '8px',
                      borderRadius: '50%',
                      background: '#6a5acd',
                      animation: 'pulse 1.5s ease-in-out 0.2s infinite',
                    }}
                  />
                  <span
                    style={{
                      width: '8px',
                      height: '8px',
                      borderRadius: '50%',
                      background: '#6a5acd',
                      animation: 'pulse 1.5s ease-in-out 0.4s infinite',
                    }}
                  />
                </div>
              )
            )}
          </div>
        </div>
      ))}

      <div ref={messagesEndRef} />

      <style>{`
        @keyframes pulse {
          0%, 100% {
            opacity: 0.3;
          }
          50% {
            opacity: 1;
          }
        }
      `}</style>
    </div>
  );
}
