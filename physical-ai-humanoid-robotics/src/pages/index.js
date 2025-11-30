import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";

import Heading from "@theme/Heading";
import styles from "./index.module.css";
function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx("hero hero--primary", styles.heroBanner)}>
      <div className={clsx("container", styles.heroContainer)}>
        <div className={styles.heroImageContent}>
          <img
            src="/img/physical-ai.png"
            alt="Physical AI Humanoid Robotics Book"
            className={styles.bookVisual}
          />
        </div>
        <div className={styles.heroTextContent}>
          <h1 className={styles.bookTitle}>Physical AI Humanoid Robotics</h1>
          <p className={styles.bookDescription}>
            Dive deep into the world of physical AI and humanoid robotics. Learn
            to integrate ROS2, build digital twins, and develop AI brains for
            advanced robotic systems.
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/module-1-ros2/ros2-basics"
            >
              Start Reading →
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
