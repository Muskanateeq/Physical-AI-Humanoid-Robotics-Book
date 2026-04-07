import React from "react";
import clsx from "clsx"; // Import clsx
import styles from "./PhysicalAISchematic.module.css"; // Import as a module

export default function PhysicalAISchematic() {
  return (
    <div className={styles["pa-stage"]}>
      {/* SVG CONNECTION LINES */}
      <svg
        className={styles["pa-connections"]}
        viewBox="0 0 920 580"
        preserveAspectRatio="none"
      >
        <defs>
          <filter id="paGlow">
            <feGaussianBlur stdDeviation="6" result="blur" />
            <feMerge>
              <feMergeNode in="blur" />
              <feMergeNode in="SourceGraphic" />
            </feMerge>
          </filter>

          <linearGradient id="paLineGrad" x1="0" x2="1">
            <stop offset="0" stopColor="#35e0d7" />
            <stop offset="1" stopColor="#6a5acd" />
          </linearGradient>
        </defs>

        {/* New looping square paths */}
        <path className={styles["pa-loop-path"]} d="M 460 90 L 840 290" />
        <path className={styles["pa-loop-path"]} d="M 840 290 L 460 490" />
        <path className={styles["pa-loop-path"]} d="M 460 490 L 80 290" />
        <path className={styles["pa-loop-path"]} d="M 80 290 L 460 90" />

        {/* The four main node dots */}
        <circle cx="460" cy="90" r="4" fill="#6a5acd" />
        <circle cx="840" cy="290" r="4" fill="#6a5acd" />
        <circle cx="460" cy="490" r="4" fill="#6a5acd" />
        <circle cx="80" cy="290" r="4" fill="#6a5acd" />
      </svg>

      {/* CENTER BODY */}
      <div className={styles["pa-center-card"]}>
        <div className={styles["pa-core"]}>
          <div className={styles["pa-inner"]} />
        </div>
      </div>

      {/* TOP NODE */}
      <div className={clsx(styles["pa-node"], styles["pa-top"])}>
        <span className={styles["pa-title"]}>ROS2</span>
        <span className={styles["pa-sub"]}>Robot Middleware</span>
      </div>

      {/* RIGHT NODE */}
      <div className={clsx(styles["pa-node"], styles["pa-right"])}>
        <span className={styles["pa-title"]}>DIGITAL TWIN</span>
        <span className={styles["pa-sub"]}>Simulation Layer</span>
      </div>

      {/* LEFT NODE */}
      <div className={clsx(styles["pa-node"], styles["pa-left"])}>
        <span className={styles["pa-title"]}>AI BRAIN</span>
        <span className={styles["pa-sub"]}>Decision Engine</span>
      </div>

      {/* BOTTOM NODE */}
      <div className={clsx(styles["pa-node"], styles["pa-bottom"])}>
        <span className={styles["pa-title"]}>VLA MODELS</span>
        <span className={styles["pa-sub"]}>Vision & Locomotion</span>
      </div>
    </div>
  );
}
