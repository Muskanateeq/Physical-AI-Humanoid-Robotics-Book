// src/components/TransformationRoadmap/index.js
import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import { translate } from '@docusaurus/Translate';
import Translate from '@docusaurus/Translate';
import styles from './styles.module.css';

const roadmapSteps = [
  {
    title: translate({
      id: 'homepage.transformationRoadmap.step1.title',
      message: 'Foundations',
    }),
    description: translate({
      id: 'homepage.transformationRoadmap.step1.description',
      message: 'Establish core understanding of robotics principles and AI basics.',
    }),
  },
  {
    title: translate({
      id: 'homepage.transformationRoadmap.step2.title',
      message: 'Sensors',
    }),
    description: translate({
      id: 'homepage.transformationRoadmap.step2.description',
      message: 'Dive into various sensor technologies and their integration for perception.',
    }),
  },
  {
    title: translate({
      id: 'homepage.transformationRoadmap.step3.title',
      message: 'Motion & Control',
    }),
    description: translate({
      id: 'homepage.transformationRoadmap.step3.description',
      message: 'Learn to program robot movement, inverse kinematics, and control systems.',
    }),
  },
  {
    title: translate({
      id: 'homepage.transformationRoadmap.step4.title',
      message: 'AI Brain',
    }),
    description: translate({
      id: 'homepage.transformationRoadmap.step4.description',
      message: 'Develop intelligent decision-making, path planning, and advanced AI behaviors.',
    }),
  },
  {
    title: translate({
      id: 'homepage.transformationRoadmap.step5.title',
      message: 'Human-Robot Interaction (HRI)',
    }),
    description: translate({
      id: 'homepage.transformationRoadmap.step5.description',
      message: 'Focus on natural and intuitive ways for humans to interact with robots.',
    }),
  },
  {
    title: translate({
      id: 'homepage.transformationRoadmap.step6.title',
      message: 'Final Humanoid Project',
    }),
    description: translate({
      id: 'homepage.transformationRoadmap.step6.description',
      message: 'Integrate all learned concepts into a comprehensive humanoid robotics project.',
    }),
  },
];

function RoadmapStep({ title, description, index }) {
  return (
    <div className={styles.roadmapItem}>
      <div className={styles.roadmapDot}></div>
      <div className={styles.roadmapContent}>
        <Heading as="h3" className={styles.roadmapTitle}>{title}</Heading>
        <p className={styles.roadmapDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function TransformationRoadmap() {
  return (
    <section className={styles.roadmapSection}>
      <div className="container">
        <h2 className={clsx('text--center', styles.sectionTitle)}>
          <Translate id="homepage.transformationRoadmap.title">
            Your Transformation Roadmap
          </Translate>
        </h2>
        <div className={styles.roadmapContainer}>
          {roadmapSteps.map((step, idx) => (
            <RoadmapStep key={idx} {...step} index={idx} />
          ))}
        </div>
      </div>
    </section>
  );
}
