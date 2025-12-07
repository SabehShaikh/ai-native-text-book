import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>

        {/* Course Badges */}
        <div className={styles.badges}>
          <span className={styles.badge}>13 Weeks</span>
          <span className={styles.badge}>4 Modules</span>
          <span className={styles.badge}>Open Source</span>
        </div>

        {/* CTA Button */}
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/module-1-physical-ai/week-1/">
            Start Reading →
          </Link>
        </div>
      </div>
    </header>
  );
}

function WhatYouWillLearn() {
  return (
    <section className={styles.whatYouWillLearn}>
      <div className="container">
        <h2>What You Will Learn</h2>
        <p className={styles.learningIntro}>
          This comprehensive course covers the complete spectrum of Physical AI and humanoid robotics,
          from foundational concepts to cutting-edge Vision-Language-Action systems.
        </p>
        <ul className={styles.learningList}>
          <li>Master ROS 2 architecture and develop real-world robotics applications</li>
          <li>Build and simulate robots in Gazebo, Unity, and NVIDIA Isaac Sim</li>
          <li>Implement reinforcement learning for robot control and sim-to-real transfer</li>
          <li>Design humanoid robot kinematics and bipedal locomotion systems</li>
          <li>Integrate vision-language models with robotic action planning (VLA)</li>
        </ul>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive 13-week course covering Physical AI, ROS 2, robot simulation, NVIDIA Isaac platform, and Vision-Language-Action systems">
      <HomepageHeader />
      <main>
        <WhatYouWillLearn />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
