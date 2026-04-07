// src/components/HomepageFeatures/index.js
import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'ROS 2 Fundamentals',
    Svg: require('@site/static/img/icons/ros.svg').default,
    description: (
      <>
        Master the Robotics Operating System 2 to build robust and scalable
        robot applications, from basic concepts to advanced navigation.
      </>
    ),
  },
  {
    title: 'Digital Twins & Simulation',
    Svg: require('@site/static/img/icons/simulation.svg').default,
    description: (
      <>
        Learn to create high-fidelity digital twins and simulate complex
        robot behaviors in virtual environments for efficient development.
      </>
    ),
  },
  {
    title: 'NVIDIA Isaac Ecosystem',
    Svg: require('@site/static/img/icons/brain.svg').default,
    description: (
      <>
        Explore NVIDIA Isaac Sim and Isaac ROS to accelerate your robotics
        projects with powerful simulation, perception, and AI tools.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action (VLA)',
    Svg: require('@site/static/img/icons/vla.svg').default,
    description: (
      <>
        Integrate cutting-edge Vision-Language-Action models to enable robots
        to understand, reason, and interact with the world like never before.
      </>
    ),
  },
];

function Feature({ Svg, title, description }) {
  return (
    <div className={clsx('col col--3', styles.featureItem)}>
      <div className={clsx('card', styles.featureCard)}>
        <div className="card__header">
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className="card__body">
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={clsx('text--center', styles.sectionTitle)}>What You Will Learn</h2>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}