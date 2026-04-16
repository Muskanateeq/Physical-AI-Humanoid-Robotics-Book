import { useState, useCallback, useEffect } from 'react';
import { API_CONFIG } from '../constants/api-url';
import { useSession } from '../lib/auth-client';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: number;
}

export interface ChatState {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  threadId: string | null;
}

export interface UseCustomChatReturn {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sendMessage: (content: string) => Promise<void>;
  clearChat: () => void;
  threadId: string | null;
}

// Helper to get user-specific storage key
function getUserStorageKey(userEmail: string | null | undefined, baseKey: string): string {
  if (!userEmail) return baseKey; // Fallback for non-authenticated users
  return `${baseKey}-${userEmail}`;
}

export function useCustomChat(): UseCustomChatReturn {
  // Get current user session
  const { data: session } = useSession();
  const userEmail = session?.user?.email;

  const [state, setState] = useState<ChatState>({
    messages: [],
    isLoading: false,
    error: null,
    threadId: null,
  });

  // Load messages from localStorage on mount or when user changes
  useEffect(() => {
    const STORAGE_KEY = getUserStorageKey(userEmail, 'custom-chat-thread');
    const savedThread = localStorage.getItem(STORAGE_KEY);

    if (savedThread) {
      try {
        const parsed = JSON.parse(savedThread);
        setState(prev => ({
          ...prev,
          messages: parsed.messages || [],
          threadId: parsed.threadId || generateThreadId(),
        }));
      } catch (e) {
        console.error('Failed to load chat history:', e);
        setState(prev => ({
          ...prev,
          threadId: generateThreadId(),
        }));
      }
    } else {
      setState(prev => ({
        ...prev,
        threadId: generateThreadId(),
      }));
    }
  }, [userEmail]); // Re-run when user changes

  // Save messages to localStorage whenever they change
  useEffect(() => {
    if (state.threadId && state.messages.length > 0 && userEmail) {
      const threadData = {
        threadId: state.threadId,
        messages: state.messages,
      };

      const STORAGE_KEY = getUserStorageKey(userEmail, 'custom-chat-thread');
      const THREAD_KEY = getUserStorageKey(userEmail, `chat-thread-${state.threadId}`);

      // Save to main storage (current conversation)
      localStorage.setItem(STORAGE_KEY, JSON.stringify(threadData));

      // Also save to thread-specific storage (for history)
      localStorage.setItem(THREAD_KEY, JSON.stringify(threadData));
    }
  }, [state.messages, state.threadId, userEmail]);

  const sendMessage = useCallback(async (content: string) => {
    if (!content.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: generateMessageId(),
      role: 'user',
      content: content.trim(),
      timestamp: Date.now(),
    };

    setState(prev => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      isLoading: true,
      error: null,
    }));

    try {
      // Call backend chat endpoint with streaming
      const response = await fetch(`${API_CONFIG.BASE_URL}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: content.trim(),
          // Let backend use its default model
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      // Handle streaming response
      const reader = response.body?.getReader();
      const decoder = new TextDecoder();

      if (!reader) {
        throw new Error('No response body');
      }

      // Create assistant message placeholder
      const assistantMessageId = generateMessageId();
      let assistantContent = '';

      setState(prev => ({
        ...prev,
        messages: [
          ...prev.messages,
          {
            id: assistantMessageId,
            role: 'assistant',
            content: '',
            timestamp: Date.now(),
          },
        ],
      }));

      // Read stream
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value);
        const lines = chunk.split('\n');

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const data = line.slice(6);
            if (data.trim() === '[DONE]') continue;

            try {
              const parsed = JSON.parse(data);

              // Try OpenRouter format first
              const delta = parsed.choices?.[0]?.delta?.content ||
                           parsed.choices?.[0]?.text ||
                           parsed.content;

              if (delta) {
                assistantContent += delta;

                // Update assistant message with accumulated content
                setState(prev => ({
                  ...prev,
                  messages: prev.messages.map(msg =>
                    msg.id === assistantMessageId
                      ? { ...msg, content: assistantContent }
                      : msg
                  ),
                }));
              }
            } catch (e) {
              // Skip invalid JSON
              console.warn('Failed to parse SSE data:', data, e);
            }
          }
        }
      }

      setState(prev => ({ ...prev, isLoading: false }));
    } catch (error) {
      console.error('Chat error:', error);
      setState(prev => ({
        ...prev,
        isLoading: false,
        error: error instanceof Error ? error.message : 'Failed to send message',
      }));
    }
  }, []);

  const clearChat = useCallback(() => {
    const STORAGE_KEY = getUserStorageKey(userEmail, 'custom-chat-thread');
    localStorage.removeItem(STORAGE_KEY);
    setState({
      messages: [],
      isLoading: false,
      error: null,
      threadId: generateThreadId(),
    });
  }, [userEmail]);

  return {
    messages: state.messages,
    isLoading: state.isLoading,
    error: state.error,
    sendMessage,
    clearChat,
    threadId: state.threadId,
  };
}

// Helper functions
function generateThreadId(): string {
  return `thread_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

function generateMessageId(): string {
  return `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}
