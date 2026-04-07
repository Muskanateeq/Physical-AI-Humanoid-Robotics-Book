import React from "react";
import Translate, { translate } from "@docusaurus/Translate";
import styles from "./styles.module.css";

// SVGs for icons - in a real app, these might be imported from separate files
const RosIcon = () => (
  <svg
    fill="slateblue"
    width="48px"
    height="48px"
    viewBox="0 0 24 24"
    xmlns="http://www.w3.org/2000/svg"
  >
    <path d="M12,2A10,10,0,1,0,22,12,10,10,0,0,0,12,2Zm0,18a8,8,0,1,1,8-8A8,8,0,0,1,12,20ZM12,5a7,7,0,0,0-7,7,2,2,0,0,0,4,0,3,3,0,0,1,3-3,2,2,0,0,0,0-4Z" />
  </svg>
);
const SimulationIcon = () => (
  <svg
    fill="slateblue"
    width="48px"
    height="48px"
    viewBox="0 0 24 24"
    xmlns="http://www.w3.org/2000/svg"
  >
    <path d="M21.71,11.29l-9-9a1,1,0,0,0-1.42,0l-9,9a1,1,0,0,0,0,1.42l9,9a1,1,0,0,0,1.42,0l9-9a1,1,0,0,0,0-1.42ZM12,20.59L3.41,12,12,3.41,20.59,12Z" />
  </svg>
);
const BrainIcon = () => (
  <svg
    fill="slateblue"
    width="48px"
    height="48px"
    viewBox="0 0 24 24"
    xmlns="http://www.w3.org/2000/svg"
  >
    <path d="M17.82,8.44A5.24,5.24,0,0,0,13,5.25a5.42,5.42,0,0,0-5.16,4,4.24,4.24,0,0,0-3.34,4.5,4.42,4.42,0,0,0,4.5,3.75h9a4,4,0,0,0,1.08-7.87,5.23,5.23,0,0,0-1.26-.19ZM17,15.5H8.5a2.42,2.42,0,0,1-2.5-2.25A2.24,2.24,0,0,1,8.34,11a.79.79,0,0,0,.78-.63,3.42,3.42,0,0,1,3.2-2.37A3.24,3.24,0,0,1,15.58,11a.75.75,0,0,0,.74.65h.28a2,2,0,0,1,2,2A2,2,0,0,1,17,15.5Z" />
  </svg>
);
const VlaIcon = () => (
  <svg
    fill="slateblue"
    width="48px"
    height="48px"
    viewBox="0 0 24 24"
    xmlns="http://www.w3.org/2000/svg"
  >
    <path d="M13,12a1,1,0,0,0-1-1H4a1,1,0,0,0,0,2h8A1,1,0,0,0,13,12Zm5-6H4A1,1,0,0,0,4,8H18a1,1,0,0,0,0-2Zm-5,8H4a1,1,0,0,0,0,2h9a1,1,0,0,0,0-2Z" />
  </svg>
);

const COURSE_MODULES = [
  {
    badgeText: translate({
      id: "homepage.courseRoadmap.module1.badge",
      message: "FOUNDATION",
    }),
    icon: <RosIcon />,
    title: translate({
      id: "homepage.courseRoadmap.module1.title",
      message: "The Robotic Nervous System (ROS 2)",
    }),
    description: translate({
      id: "homepage.courseRoadmap.module1.description",
      message: "Middleware for robot control and bridging Python agents.",
    }),
    keyTopics: [
      translate({
        id: "homepage.courseRoadmap.module1.topic1",
        message: "ROS 2 Nodes, Topics, and Services",
      }),
      translate({
        id: "homepage.courseRoadmap.module1.topic2",
        message: "Python Agents with rclpy",
      }),
      translate({
        id: "homepage.courseRoadmap.module1.topic3",
        message: "URDF for Humanoids",
      }),
    ],
    timeframe: translate({
      id: "homepage.courseRoadmap.module1.timeframe",
      message: "Weeks 1-5",
    }),
  },
  {
    badgeText: translate({
      id: "homepage.courseRoadmap.module2.badge",
      message: "SIMULATION",
    }),
    icon: <SimulationIcon />,
    title: translate({
      id: "homepage.courseRoadmap.module2.title",
      message: "The Digital Twin",
    }),
    description: translate({
      id: "homepage.courseRoadmap.module2.description",
      message: "Physics simulation, high-fidelity environments, and sensor integration.",
    }),
    keyTopics: [
      translate({
        id: "homepage.courseRoadmap.module2.topic1",
        message: "Gazebo Physics & Gravity",
      }),
      translate({
        id: "homepage.courseRoadmap.module2.topic2",
        message: "Unity HRI Visualization",
      }),
      translate({
        id: "homepage.courseRoadmap.module2.topic3",
        message: "Sensors: LiDAR, Depth, IMU",
      }),
    ],
    timeframe: translate({
      id: "homepage.courseRoadmap.module2.timeframe",
      message: "Weeks 6-7",
    }),
  },
  {
    badgeText: translate({
      id: "homepage.courseRoadmap.module3.badge",
      message: "AI-POWERED",
    }),
    icon: <BrainIcon />,
    title: translate({
      id: "homepage.courseRoadmap.module3.title",
      message: "The AI-Robot Brain",
    }),
    description: translate({
      id: "homepage.courseRoadmap.module3.description",
      message: "Advanced perception, navigation, and reinforcement learning.",
    }),
    keyTopics: [
      translate({
        id: "homepage.courseRoadmap.module3.topic1",
        message: "NVIDIA Isaac Sim & Synthetic Data",
      }),
      translate({
        id: "homepage.courseRoadmap.module3.topic2",
        message: "Isaac ROS & VSLAM",
      }),
      translate({
        id: "homepage.courseRoadmap.module3.topic3",
        message: "Nav2 Path Planning",
      }),
    ],
    timeframe: translate({
      id: "homepage.courseRoadmap.module3.timeframe",
      message: "Weeks 8-10",
    }),
  },
  {
    badgeText: translate({
      id: "homepage.courseRoadmap.module4.badge",
      message: "CAPSTONE",
    }),
    icon: <VlaIcon />,
    title: translate({
      id: "homepage.courseRoadmap.module4.title",
      message: "Vision-Language-Action (VLA)",
    }),
    description: translate({
      id: "homepage.courseRoadmap.module4.description",
      message: "Convergence of LLMs and Robotics for conversational control.",
    }),
    keyTopics: [
      translate({
        id: "homepage.courseRoadmap.module4.topic1",
        message: "Voice-to-Action (Whisper)",
      }),
      translate({
        id: "homepage.courseRoadmap.module4.topic2",
        message: "LLM Cognitive Planning",
      }),
      translate({
        id: "homepage.courseRoadmap.module4.topic3",
        message: "Project: Autonomous Humanoid",
      }),
    ],
    timeframe: translate({
      id: "homepage.courseRoadmap.module4.timeframe",
      message: "Weeks 11-13",
    }),
  },
];

function CourseCard({
  badgeText,
  icon,
  title,
  description,
  keyTopics,
  timeframe,
}) {
  return (
    <div className={styles.courseCard}>
      <div className={styles.badge}>{badgeText}</div>
      <div className={styles.cardHeader}>
        <div className={styles.cardIcon}>{icon}</div>
      </div>
      <div className={styles.cardBody}>
        <h3 className={styles.cardTitle}>{title}</h3>
        <p className={styles.cardDescription}>{description}</p>
        <ul className={styles.cardTopics}>
          {keyTopics.map((topic, index) => (
            <li key={index}>{topic}</li>
          ))}
        </ul>
      </div>
      <div className={styles.cardFooter}>
        <p className={styles.cardTimeframe}>{timeframe}</p>
      </div>
    </div>
  );
}

export default function CourseRoadmap() {
  return (
    <section className={styles.roadmapSection}>
      <div className="container">
        <h2 className={styles.roadmapTitle}>
          <Translate id="homepage.courseRoadmap.title">
            From Foundations to Thinking Machines
          </Translate>
        </h2>
        <p className={styles.roadmapSubtitle}>
          <Translate id="homepage.courseRoadmap.subtitle">
            Course Overview: Master Physical AI. Design, simulate, and deploy
            humanoid robots capable of natural human interactions using ROS 2,
            Gazebo, and NVIDIA Isaac.
          </Translate>
        </p>
        <div className={styles.cardsGrid}>
          {COURSE_MODULES.map((module, index) => (
            <CourseCard key={index} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}
