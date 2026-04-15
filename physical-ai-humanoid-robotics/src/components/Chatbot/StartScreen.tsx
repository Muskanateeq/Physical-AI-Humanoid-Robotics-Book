import React from 'react';

interface StartScreenProps {
  onPromptClick: (prompt: string) => void;
}

const PROMPTS = [
  {
    label: "About ROS2",
    prompt: "What is ROS2 and how does it work as the nervous system for robots?",
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
];

export default function StartScreen({ onPromptClick }: StartScreenProps) {
  return (
    <div
      style={{
        flex: 1,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
        padding: "2rem 1.5rem",
        textAlign: "center",
        overflowY: "auto",
        background: "rgba(15, 23, 42, 0.95)",
      }}
    >
      <h2
        style={{
          color: "#ffffff",
          fontSize: "1.25rem",
          fontWeight: "600",
          marginBottom: "0.5rem",
          lineHeight: "1.4",
        }}
      >
        Welcome to NeuroBotics AI Assistant!
      </h2>
      <p
        style={{
          color: "#d1d5db",
          fontSize: "0.875rem",
          marginBottom: "2rem",
          lineHeight: "1.5",
        }}
      >
        Ask me anything about Physical AI and Robotics
      </p>
      <div
        style={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr",
          gap: "0.75rem",
          width: "100%",
          maxWidth: "360px",
        }}
      >
        {PROMPTS.map((item, index) => (
          <button
            key={index}
            onClick={() => onPromptClick(item.prompt)}
            style={{
              padding: "0.875rem 1rem",
              background: "#5848b8",
              color: "#ffffff",
              border: "1px solid #6a5acd",
              borderRadius: "0.5rem",
              cursor: "pointer",
              fontSize: "0.875rem",
              fontWeight: "500",
              transition: "all 0.2s",
              textAlign: "left",
              lineHeight: "1.4",
            }}
            onMouseOver={(e) => {
              e.currentTarget.style.background = "#5648b6";
              e.currentTarget.style.borderColor = "#daa520";
              e.currentTarget.style.transform = "translateY(-2px)";
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.background = "#5848b8";
              e.currentTarget.style.borderColor = "#6a5acd";
              e.currentTarget.style.transform = "translateY(0)";
            }}
          >
            {item.label}
          </button>
        ))}
      </div>
    </div>
  );
}
