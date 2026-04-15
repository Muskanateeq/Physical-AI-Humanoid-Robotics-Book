import React, { useState, useEffect } from "react";
import { useSession } from "../../lib/auth-client";
import { useCustomChat } from "../../hooks/useCustomChat";
import StartScreen from "./StartScreen";
import MessageList from "./MessageList";
import MessageInput from "./MessageInput";
import HistoryView from "./HistoryView";

export default function Chatbot() {
  const [isReady, setIsReady] = useState(false);
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [showHistory, setShowHistory] = useState(false);

  // Use Better Auth React hook for reactive session state
  const { data: session, isPending } = useSession();

  // Use custom chat hook
  const { messages, isLoading, sendMessage, clearChat, threadId } = useCustomChat();

  // Conversation history management
  const [conversations, setConversations] = useState<Array<{
    id: string;
    title: string;
    timestamp: number;
  }>>([]);

  // Load conversations from localStorage
  useEffect(() => {
    const loadConversations = () => {
      try {
        const saved = localStorage.getItem('chat-conversations-list');
        if (saved) {
          setConversations(JSON.parse(saved));
        }
      } catch (e) {
        console.error('Failed to load conversations:', e);
      }
    };
    loadConversations();
  }, []);

  // Save current conversation to history when messages change
  useEffect(() => {
    if (messages.length > 0 && threadId) {
      const firstUserMessage = messages.find(m => m.role === 'user');
      const title = firstUserMessage
        ? (firstUserMessage.content.length > 50
            ? firstUserMessage.content.substring(0, 50) + '...'
            : firstUserMessage.content)
        : 'New Conversation';

      setConversations(prev => {
        const existing = prev.find(c => c.id === threadId);
        const updated = existing
          ? prev.map(c => c.id === threadId ? { ...c, title, timestamp: Date.now() } : c)
          : [{ id: threadId, title, timestamp: Date.now() }, ...prev];

        localStorage.setItem('chat-conversations-list', JSON.stringify(updated));
        return updated;
      });
    }
  }, [messages, threadId]);

  // Set ready state on mount
  useEffect(() => {
    setIsReady(true);
  }, []);

  // Listen for custom event to open chatbot from other components
  useEffect(() => {
    const handleOpenChatbotEvent = () => {
      handleOpenChat();
    };

    window.addEventListener('openChatbot', handleOpenChatbotEvent);

    return () => {
      window.removeEventListener('openChatbot', handleOpenChatbotEvent);
    };
  }, [session]); // Re-attach listener when session changes

  // Check authentication before opening chatbot
  const handleOpenChat = () => {
    // If user is not authenticated, redirect to signin
    if (!session?.user) {
      const currentPath = window.location.pathname;
      window.location.href = `/signin?callbackUrl=${encodeURIComponent(currentPath)}`;
      return;
    }

    // User is authenticated, open chat
    setIsChatOpen(true);
  };

  // Handle new chat
  const handleNewChat = () => {
    clearChat();
    setShowHistory(false); // Go back to chat view
  };

  // Handle select conversation from history
  const handleSelectConversation = (conversationId: string) => {
    try {
      const saved = localStorage.getItem('custom-chat-thread');
      if (saved) {
        const parsed = JSON.parse(saved);
        if (parsed.threadId === conversationId) {
          // Already loaded, just close history
          setShowHistory(false);
          return;
        }
      }

      // Load the selected conversation
      const convKey = `chat-thread-${conversationId}`;
      const convData = localStorage.getItem(convKey);
      if (convData) {
        localStorage.setItem('custom-chat-thread', convData);
        setShowHistory(false);
        window.location.reload(); // Reload to load the conversation
      }
    } catch (e) {
      console.error('Failed to load conversation:', e);
    }
  };

  // Handle delete conversation
  const handleDeleteConversation = (conversationId: string) => {
    try {
      // Remove from list
      setConversations(prev => {
        const updated = prev.filter(c => c.id !== conversationId);
        localStorage.setItem('chat-conversations-list', JSON.stringify(updated));
        return updated;
      });

      // Remove conversation data
      localStorage.removeItem(`chat-thread-${conversationId}`);

      // If deleting current conversation, clear it
      const current = localStorage.getItem('custom-chat-thread');
      if (current) {
        const parsed = JSON.parse(current);
        if (parsed.threadId === conversationId) {
          clearChat();
        }
      }
    } catch (e) {
      console.error('Failed to delete conversation:', e);
    }
  };

  // Handle prompt click from start screen
  const handlePromptClick = (prompt: string) => {
    sendMessage(prompt);
  };

  if (!isReady || isPending) {
    return null;
  }

  return (
    <div>
      {/* Floating Chat Button (bottom-right) - Always visible, redirects to signin if not authenticated */}
      {!isChatOpen && (
        <button
          onClick={handleOpenChat}
          style={{
            position: "fixed",
            bottom: "2rem",
            right: "2rem",
            width: "60px",
            height: "60px",
            borderRadius: "50%",
            background: "linear-gradient(135deg, #6a5acd, #1e1b4b)",
            border: "none",
            cursor: "pointer",
            boxShadow: "0 4px 20px rgba(180, 165, 80, 0.4)",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            transition: "transform 0.2s",
            zIndex: 100,
          }}
          onMouseOver={(e) => (e.currentTarget.style.transform = "scale(1.1)")}
          onMouseOut={(e) => (e.currentTarget.style.transform = "scale(1)")}
        >
          <svg
            width="28"
            height="28"
            viewBox="0 0 24 24"
            fill="none"
            stroke="white"
            strokeWidth="2"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}

      {/* Chat Popup (bottom-right, above the button) */}
      {isChatOpen && (
        <>
          {/* Backdrop */}
          <div
            onClick={() => setIsChatOpen(false)}
            style={{
              position: "fixed",
              top: 0,
              left: 0,
              right: 0,
              bottom: 0,
              background: "rgba(0, 0, 0, 0.3)",
              zIndex: 999,
            }}
          />

          {/* Popup Window */}
          <div
            style={{
              position: "fixed",
              bottom: "2rem",
              right: "2rem",
              width: "420px",
              height: "600px",
              maxWidth: "calc(100vw - 4rem)",
              maxHeight: "calc(100vh - 4rem)",
              background: "rgba(15, 23, 42, 0.95)",
              borderRadius: "1rem",
              boxShadow: "0 10px 50px rgba(0, 0, 0, 0.5)",
              display: "flex",
              flexDirection: "column",
              overflow: "hidden",
              zIndex: 1000,
              animation: "popupIn 0.25s ease-out",
            }}
          >
            {/* Chat Header */}
            <div
              style={{
                padding: "1rem 1.25rem",
                background: "#5848b8",
                borderBottom: "1px solid #6a5acd",
                display: "flex",
                justifyContent: "space-between",
                alignItems: "center",
                flexShrink: 0,
              }}
            >
              <span
                style={{
                  color: "#ffffff",
                  fontWeight: "bold",
                  fontSize: "1rem",
                }}
              >
                NeuroBotics Assistant
              </span>
              <div style={{ display: "flex", gap: "0.5rem" }}>
                {/* History Button */}
                <button
                  onClick={() => setShowHistory(true)}
                  title="History"
                  style={{
                    padding: "0.4rem",
                    background: "transparent",
                    color: "#ffffff",
                    border: "1px solid #6a5acd",
                    borderRadius: "0.375rem",
                    cursor: "pointer",
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                    transition: "all 0.2s",
                  }}
                  onMouseOver={(e) => {
                    e.currentTarget.style.background = "#6a5acd";
                  }}
                  onMouseOut={(e) => {
                    e.currentTarget.style.background = "transparent";
                  }}
                >
                  <svg
                    width="16"
                    height="16"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    strokeWidth="2"
                  >
                    <circle cx="12" cy="12" r="10"></circle>
                    <polyline points="12 6 12 12 16 14"></polyline>
                  </svg>
                </button>
                {/* Close Button */}
                <button
                  onClick={() => setIsChatOpen(false)}
                  style={{
                    padding: "0.4rem",
                    background: "transparent",
                    color: "#a0a0a0",
                    border: "1px solid #a0a0a0",
                    borderRadius: "0.375rem",
                    cursor: "pointer",
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                  }}
                >
                  <svg
                    width="16"
                    height="16"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    strokeWidth="2"
                  >
                    <line x1="18" y1="6" x2="6" y2="18"></line>
                    <line x1="6" y1="6" x2="18" y2="18"></line>
                  </svg>
                </button>
              </div>
            </div>

            {/* Chat Content */}
            <div
              style={{
                flex: 1,
                overflow: "hidden",
                display: "flex",
                flexDirection: "column",
              }}
            >
              {showHistory ? (
                <HistoryView
                  conversations={conversations}
                  onSelectConversation={handleSelectConversation}
                  onDeleteConversation={handleDeleteConversation}
                  onNewChat={handleNewChat}
                  onBack={() => setShowHistory(false)}
                />
              ) : (
                <>
                  {messages.length === 0 ? (
                    <StartScreen onPromptClick={handlePromptClick} />
                  ) : (
                    <MessageList messages={messages} isLoading={isLoading} />
                  )}

                  {/* Input always visible at bottom */}
                  <MessageInput
                    onSend={sendMessage}
                    disabled={isLoading}
                    placeholder="Ask anything about Physical AI and Robotics..."
                  />
                </>
              )}
            </div>
          </div>
        </>
      )}

      {/* CSS Animation */}
      <style>{`
        @keyframes popupIn {
          from {
            opacity: 0;
            transform: scale(0.9) translateY(20px);
          }
          to {
            opacity: 1;
            transform: scale(1) translateY(0);
          }
        }
      `}</style>
    </div>
  );
}
