import React, { useState } from 'react';

interface Conversation {
  id: string;
  title: string;
  timestamp: number;
}

interface HistoryViewProps {
  conversations: Conversation[];
  onSelectConversation: (id: string) => void;
  onDeleteConversation: (id: string) => void;
  onNewChat: () => void;
  onBack: () => void;
}

export default function HistoryView({
  conversations,
  onSelectConversation,
  onDeleteConversation,
  onNewChat,
  onBack,
}: HistoryViewProps) {
  const [deleteConfirmId, setDeleteConfirmId] = useState<string | null>(null);

  const handleDelete = (id: string) => {
    onDeleteConversation(id);
    setDeleteConfirmId(null);
  };

  return (
    <div
      style={{
        flex: 1,
        display: 'flex',
        flexDirection: 'column',
        background: 'rgba(15, 23, 42, 0.95)',
        overflow: 'hidden',
      }}
    >
      {/* History Header (Transparent) */}
      <div
        style={{
          padding: '1rem 1.25rem',
          background: 'transparent',
          borderBottom: '1px solid #5848b8',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
        }}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
          <button
            onClick={onBack}
            style={{
              background: 'transparent',
              border: 'none',
              color: '#ffffff',
              cursor: 'pointer',
              padding: '0.25rem',
              display: 'flex',
              alignItems: 'center',
            }}
            title="Back to chat"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="19" y1="12" x2="5" y2="12"></line>
              <polyline points="12 19 5 12 12 5"></polyline>
            </svg>
          </button>
          <h2 style={{ color: '#ffffff', fontSize: '1rem', fontWeight: '600', margin: 0 }}>
            Conversation History
          </h2>
        </div>
        <button
          onClick={onNewChat}
          style={{
            padding: '0.5rem 1rem',
            background: '#b8860b',
            color: '#ffffff',
            border: 'none',
            borderRadius: '0.375rem',
            cursor: 'pointer',
            fontSize: '0.875rem',
            fontWeight: '500',
            transition: 'all 0.2s',
          }}
          onMouseOver={(e) => {
            e.currentTarget.style.background = '#daa520';
          }}
          onMouseOut={(e) => {
            e.currentTarget.style.background = '#b8860b';
          }}
        >
          New Chat
        </button>
      </div>

      {/* Conversation List */}
      <div
        style={{
          flex: 1,
          overflowY: 'auto',
          padding: '1rem',
        }}
      >
        {conversations.length === 0 ? (
          <div style={{ textAlign: 'center', padding: '3rem 1rem', color: '#9ca3af' }}>
            <p>No conversation history yet.</p>
            <p style={{ fontSize: '0.875rem', marginTop: '0.5rem' }}>
              Start a new conversation to see it here.
            </p>
          </div>
        ) : (
          <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
            {conversations.map((conv) => (
              <div
                key={conv.id}
                style={{
                  background: '#5848b8',
                  borderRadius: '0.5rem',
                  padding: '1rem',
                  border: '1px solid #6a5acd',
                  transition: 'all 0.2s',
                  cursor: 'pointer',
                }}
                onMouseOver={(e) => {
                  e.currentTarget.style.background = '#6a5acd';
                }}
                onMouseOut={(e) => {
                  e.currentTarget.style.background = '#5848b8';
                }}
              >
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start' }}>
                  <div
                    onClick={() => onSelectConversation(conv.id)}
                    style={{ flex: 1, minWidth: 0 }}
                  >
                    <h3
                      style={{
                        color: '#ffffff',
                        fontSize: '0.95rem',
                        fontWeight: '500',
                        margin: '0 0 0.5rem 0',
                        overflow: 'hidden',
                        textOverflow: 'ellipsis',
                        whiteSpace: 'nowrap',
                      }}
                    >
                      {conv.title}
                    </h3>
                    <p style={{ color: '#d1d5db', fontSize: '0.75rem', margin: 0 }}>
                      {new Date(conv.timestamp).toLocaleDateString()} {new Date(conv.timestamp).toLocaleTimeString()}
                    </p>
                  </div>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      setDeleteConfirmId(conv.id);
                    }}
                    style={{
                      background: 'transparent',
                      border: 'none',
                      color: '#ef4444',
                      cursor: 'pointer',
                      padding: '0.25rem',
                      marginLeft: '0.5rem',
                    }}
                    title="Delete conversation"
                  >
                    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                      <polyline points="3 6 5 6 21 6"></polyline>
                      <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
                    </svg>
                  </button>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Delete Confirmation Overlay */}
      {deleteConfirmId && (
        <>
          <div
            onClick={() => setDeleteConfirmId(null)}
            style={{
              position: 'absolute',
              top: 0,
              left: 0,
              right: 0,
              bottom: 0,
              background: 'rgba(0, 0, 0, 0.7)',
              zIndex: 10,
            }}
          />
          <div
            style={{
              position: 'absolute',
              top: '50%',
              left: '50%',
              transform: 'translate(-50%, -50%)',
              width: '90%',
              maxWidth: '350px',
              background: 'rgba(15, 23, 42, 0.98)',
              borderRadius: '0.75rem',
              padding: '1.5rem',
              zIndex: 11,
              border: '1px solid #5848b8',
            }}
          >
            <h3 style={{ color: '#ffffff', fontSize: '1.125rem', fontWeight: '600', margin: '0 0 1rem 0' }}>
              Delete Conversation?
            </h3>
            <p style={{ color: '#d1d5db', fontSize: '0.875rem', margin: '0 0 1.5rem 0' }}>
              Are you sure you want to permanently delete this conversation? This action cannot be undone.
            </p>
            <div style={{ display: 'flex', gap: '0.75rem', justifyContent: 'flex-end' }}>
              <button
                onClick={() => setDeleteConfirmId(null)}
                style={{
                  padding: '0.5rem 1rem',
                  background: 'transparent',
                  color: '#d1d5db',
                  border: '1px solid #6a5acd',
                  borderRadius: '0.375rem',
                  cursor: 'pointer',
                  fontSize: '0.875rem',
                  fontWeight: '500',
                }}
              >
                Cancel
              </button>
              <button
                onClick={() => handleDelete(deleteConfirmId)}
                style={{
                  padding: '0.5rem 1rem',
                  background: '#ef4444',
                  color: '#ffffff',
                  border: 'none',
                  borderRadius: '0.375rem',
                  cursor: 'pointer',
                  fontSize: '0.875rem',
                  fontWeight: '500',
                }}
              >
                Delete Permanently
              </button>
            </div>
          </div>
        </>
      )}
    </div>
  );
}
