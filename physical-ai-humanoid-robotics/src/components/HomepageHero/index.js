// src/components/HomepageHero/index.js
import React, { useState } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import Translate from "@docusaurus/Translate";
import styles from "./styles.module.css";
import PhysicalAISchematic from "./PhysicalAISchematic";
import { useSession } from "@site/src/lib/auth-client";
import BrowserOnly from "@docusaurus/BrowserOnly";

function HomepageHero() {
  const { data: session } = useSession();
  const [isChatOpen, setIsChatOpen] = useState(false);

  const handleTryAIAssistant = () => {
    // Check if user is authenticated
    if (!session?.user) {
      // Redirect to signin with callback to home
      window.location.href = '/signin?callbackUrl=/';
      return;
    }

    // User is authenticated, open chatbot
    // We'll trigger the chatbot by dispatching a custom event
    window.dispatchEvent(new CustomEvent('openChatbot'));
  };
  return (
    <div className="dark-theme-override">
      <header className={clsx("hero hero--primary", styles.heroSection)}>
        <div className={clsx("container", styles.heroContainer)}>
          <div className={clsx("row", styles.heroRow)}>
            {/* Left Column: Typography & Content */}
            <div className={clsx("col col--6", styles.heroTextColumn)}>
              <h1 className={styles.mainHeadline}>
                <Translate id="homepage.hero.title">
                  Build the Future of Physical Intelligence with Humanoid Robotics.
                </Translate>
                <span className={styles.animatedUnderline}></span>{" "}
                {/* Animated underline */}
              </h1>
              <p className={styles.subHeadline}>
                <Translate id="homepage.hero.subtitle">
                  Physical AI & Humanoid Robotics is a complete, hands-on learning
                  journey. From ROS 2 and simulation to Vision-Language-Action
                  models.
                </Translate>
              </p>
              <div className={styles.buttons}>
                <Link
                  className={clsx(
                    "button button--primary button--lg",
                    styles.heroButton,
                  )}
                  to="/docs/"
                >
                  <Translate id="homepage.hero.button.startLearning">
                    Start Learning
                  </Translate>
                </Link>
                <button
                  className={clsx(
                    "button button--secondary button--lg",
                    styles.heroButton,
                  )}
                  onClick={handleTryAIAssistant}
                  style={{ cursor: 'pointer' }}
                >
                  <Translate id="homepage.hero.button.tryAI">
                    Try AI Assistant
                  </Translate>
                </button>
              </div>
              <div className={styles.techStack}>
                <span className={styles.goldenrodBullet}>•</span>
                <span>ROS 2</span>{" "}
                <span className={styles.goldenrodBullet}>•</span>
                <span>Isaac Sim</span>{" "}
                <span className={styles.goldenrodBullet}>•</span>
                <span>AI Brain</span>{" "}
                <span className={styles.goldenrodBullet}>•</span>
                <span>VLA Models</span>
              </div>
            </div>

            {/* Right Column: Schematic Illustration */}
            <div className={clsx("col col--6", styles.heroDiagramColumn)}>
              <PhysicalAISchematic />
            </div>
          </div>
        </div>
      </header>
    </div>
  );
}

export default HomepageHero;
