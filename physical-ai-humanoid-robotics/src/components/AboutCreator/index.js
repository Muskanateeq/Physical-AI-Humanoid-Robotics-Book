// src/components/AboutCreator/index.js
import React from "react";
import clsx from "clsx";
import Heading from "@theme/Heading";
import Translate from "@docusaurus/Translate";
import styles from "./styles.module.css";
import founderImage from "@site/static/img/founder.jpeg";

function AboutCreator() {
  return (
    <section className={styles.aboutCreatorSection}>
      <div className="container text--center">
        <h2 className={clsx("text--center", styles.sectionTitle)}>
          <Translate id="homepage.aboutCreator.title">
            About the Creator
          </Translate>
        </h2>
        <div className={styles.creatorContent}>
          <img
            src={founderImage}
            alt="Creator Avatar"
            className={styles.creatorAvatar}
          />
          <p className={styles.creatorParagraph}>
            <Translate id="homepage.aboutCreator.description">
              Meet Muskan Ateeq, the visionary behind "Physical AI & Humanoid
              Robotics." With a passion for intelligent machines and a deep
              understanding of cutting-edge AI, Muskan has dedicated years to
              exploring the convergence of robotics and artificial intelligence.
              This course is born from a desire to share practical knowledge and
              inspire the next generation of innovators to build the future of
              physical intelligence.
            </Translate>
          </p>
        </div>
      </div>
    </section>
  );
}

export default AboutCreator;