import React from "react";
import Layout from "@theme/Layout";
import Chatbot from "../../components/Chatbot";
import { useSession } from "../../lib/auth-client";
import { useHistory } from "@docusaurus/router";

export default function ChatbotPage() {
  const { data: session, isPending } = useSession();
  const history = useHistory();

  // Show loading state
  if (isPending) {
    return (
      <Layout title="AI Chatbot" description="Physical AI & Humanoid Robotics AI Assistant">
        <div
          style={{
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            minHeight: "calc(100vh - 60px)",
            background: "#1a1a2e",
            color: "#6A5ACD",
          }}
        >
          <div style={{ textAlign: "center" }}>
            <div
              style={{
                width: "48px",
                height: "48px",
                border: "4px solid rgba(106, 90, 205, 0.2)",
                borderTopColor: "#6A5ACD",
                borderRadius: "50%",
                animation: "spin 1s linear infinite",
                margin: "0 auto 1rem",
              }}
            />
            <p>Loading...</p>
          </div>
          <style>{`
            @keyframes spin {
              to { transform: rotate(360deg); }
            }
          `}</style>
        </div>
      </Layout>
    );
  }

  // Redirect to signin if not authenticated
  if (!session?.user) {
    if (typeof window !== "undefined") {
      history.push("/signin?callbackUrl=/chatbot");
    }
    return (
      <Layout title="AI Chatbot" description="Physical AI & Humanoid Robotics AI Assistant">
        <div
          style={{
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            minHeight: "calc(100vh - 60px)",
            background: "#1a1a2e",
            color: "#6A5ACD",
          }}
        >
          <p>Redirecting to sign in...</p>
        </div>
      </Layout>
    );
  }

  // Render chatbot for authenticated users
  return (
    <Layout title="AI Chatbot" description="Physical AI & Humanoid Robotics AI Assistant">
      <div
        style={{
          minHeight: "calc(100vh - 60px)",
          background: "#1a1a2e",
        }}
      >
        <Chatbot />
      </div>
    </Layout>
  );
}
