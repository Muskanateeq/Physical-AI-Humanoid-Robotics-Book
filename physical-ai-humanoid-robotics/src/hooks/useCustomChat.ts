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

      // A browser ReadableStream chunk is not the same thing as an SSE event. An
      // event (and its JSON payload) may be split across multiple chunks, so keep
      // incomplete data until the SSE blank-line delimiter arrives.
      let sseBuffer = '';

      const processSseEvent = (event: string) => {
        const data = event
          .split(/\r?\n/)
          .filter(line => line.startsWith('data:'))
          .map(line => line.slice(5).trimStart())
          .join('\n');

        if (!data || data === '[DONE]') return;

        try {
          const parsed = JSON.parse(data);
          const choice = parsed.choices?.[0];

          if (parsed.type === 'retry') {
            assistantContent = '';
            setState(prev => ({
              ...prev,
              messages: prev.messages.map(msg =>
                msg.id === assistantMessageId ? { ...msg, content: '' } : msg
              ),
            }));
            return;
          }

          if (parsed.type === 'error') {
            setState(prev => ({
              ...prev,
              error: parsed.message || 'The AI response was interrupted. Please try again.',
            }));
            return;
          }

          if (choice?.finish_reason === 'error') {
            console.error('Chat provider ended the stream with an error:', parsed);
            setState(prev => ({
              ...prev,
              error: 'The AI response was interrupted. Please try again.',
            }));
            return;
          }

          // Try OpenRouter format first
          const delta = choice?.delta?.content || choice?.text || parsed.content;

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
          // A complete SSE event should always contain valid JSON. Keep the
          // partial assistant response instead of crashing the chat UI.
          console.warn('Failed to parse complete SSE event:', data, e);
        }
      };

      // Read stream
      while (true) {
        const { done, value } = await reader.read();
        if (done) {
          sseBuffer += decoder.decode();
          break;
        }

        sseBuffer += decoder.decode(value, { stream: true });
        const events = sseBuffer.split(/\r?\n\r?\n/);
        sseBuffer = events.pop() ?? '';

        for (const event of events) {
          processSseEvent(event);
        }
      }

      // Some proxies omit the final blank line. At EOF it is safe to process
      // the final event; malformed provider data is reported without losing the
      // valid text already received.
      if (sseBuffer.trim()) {
        processSseEvent(sseBuffer);
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
