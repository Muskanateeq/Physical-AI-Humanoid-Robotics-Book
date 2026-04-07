// src/components/FinalCTABanner/index.js
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import Translate from '@docusaurus/Translate';
import styles from './styles.module.css';

function FinalCTABanner() {
  return (
    <section className={styles.ctaBanner}>
      <div className="container text--center">
        <Heading as="h2" className={styles.ctaTitle}>
          <Translate id="homepage.finalCTA.title">
            Join The AI & Robotics Revolution
          </Translate>
        </Heading>
        <div className={styles.ctaButtons}>
          <Link
            className={clsx('button button--primary button--lg', styles.ctaButton)}
            to="/docs/module-1-ros2/robotic-system">
            <Translate id="homepage.finalCTA.button">
              🚀 Enter The Future
            </Translate>
          </Link>
        </div>
      </div>
    </section>
  );
}

export default FinalCTABanner;