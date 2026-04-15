import { ChatKit, useChatKit } from "@openai/chatkit-react";
import React, { useState, useEffect } from "react";
import { API_CONFIG } from "../../constants/api-url";
import { useSession } from "../../lib/auth-client";

export default function Chatbot() {
  const [initialThread, setInitialThread] = useState<string | null>(null);
  const [isReady, setIsReady] = useState(false);
  const [isChatOpen, setIsChatOpen] = useState(false);

  // Use Better Auth React hook for reactive session state
  const { data: session, isPending } = useSession();

  // Load saved thread ID on mount
  useEffect(() => {
    const savedThread = localStorage.getItem("chatkit-thread-id");
    setInitialThread(savedThread);
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

  const { control } = useChatKit({
    api: {
      url: API_CONFIG.CHATKIT_URL,
      // Dynamic domainKey: localhost for development, production domain for production
      domainKey: typeof window !== 'undefined' &&
                 (window.location.hostname.includes('vercel.app') ||
                  window.location.hostname.includes('neurobotics-ai-book'))
        ? "neurobotics-ai-book.vercel.app"
        : "localhost",
    },
    initialThread: initialThread,
    theme: {
      colorScheme: "dark",
      color: {
        grayscale: { hue: 220, tint: 6, shade: -1 },
        accent: { primary: "#6a5acd", level: 1 },
      },
      radius: "round",
    },
    startScreen: {
      greeting: "Welcome to NeuroBotics AI Assistant!",
      prompts: [
        {
          label: "About ROS2",
          prompt:
            "What is ROS2 and how does it work as the nervous system for robots?",
        },
        {
          label: "Simulation Help",
          prompt: "Explain Gazebo and Unity in robotics",
        },
        {
          label: "AI Robot Brain",
          prompt: "How does Nvidia Isaac help create an AI Robot Brain?",
        },
        {
          label: "Vision Language Action",
          prompt: "What is Vision Language Action in robotics?",
        },
      ],
    },
    composer: {
      placeholder: "Ask anything about Physical AI and Robotics...",
    },
    onThreadChange: ({ threadId }) => {
      console.log("Thread changed:", threadId);
      if (threadId) {
        localStorage.setItem("chatkit-thread-id", threadId);
      }
    },
    onError: ({ error }) => {
      console.error("ChatKit error:", error);
    },
    onReady: () => {
      console.log("ChatKit is ready!");
    },
  });

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
            boxShadow: "0 4px 20px rgba(76, 201, 240, 0.4)",
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
              background: "#16213e",
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
                background: "#0f3460",
                borderBottom: "1px solid #1a1a2e",
                display: "flex",
                justifyContent: "space-between",
                alignItems: "center",
                flexShrink: 0,
              }}
            >
              <span
                style={{
                  color: "#6a5acd",
                  fontWeight: "bold",
                  fontSize: "1rem",
                }}
              >
                NeuroBotics Assistant
              </span>
              <div style={{ display: "flex", gap: "0.5rem" }}>
                <button
                  onClick={() => {
                    localStorage.removeItem("chatkit-thread-id");
                    window.location.reload();
                  }}
                  style={{
                    padding: "0.4rem 0.6rem",
                    background: "#6a5acd",
                    color: "white",
                    border: "none",
                    borderRadius: "0.375rem",
                    cursor: "pointer",
                    fontSize: "0.7rem",
                  }}
                >
                  New Chat
                </button>
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
            <div style={{ flex: 1, overflow: "hidden" }}>
              <ChatKit control={control} className="h-full w-full" />
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
