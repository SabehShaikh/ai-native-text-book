import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type ModuleItem = {
  title: string;
  icon: string;
  description: JSX.Element;
  weekRange: string;
  href: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: Physical AI & ROS 2',
    icon: '🤖',
    description: (
      <>
        Explore the foundations of embodied intelligence and master ROS 2 for robotics development.
        Learn core concepts, communication patterns, and package development.
      </>
    ),
    weekRange: 'Weeks 1-5',
    href: '/module-1-physical-ai/week-1/',
  },
  {
    title: 'Module 2: Robot Simulation',
    icon: '🎮',
    description: (
      <>
        Build realistic robot simulations using Gazebo and Unity. Master URDF modeling,
        physics simulation, and ROS-Unity integration for testing robotics systems.
      </>
    ),
    weekRange: 'Weeks 6-7',
    href: '/module-2-simulation/week-6/',
  },
  {
    title: 'Module 3: NVIDIA Isaac Platform',
    icon: '⚡',
    description: (
      <>
        Dive into Isaac SDK and Isaac Sim for advanced perception and reinforcement learning.
        Implement sim-to-real transfer for real-world robot deployment.
      </>
    ),
    weekRange: 'Weeks 8-10',
    href: '/module-3-isaac/week-8/',
  },
  {
    title: 'Module 4: Humanoid Robotics & VLA',
    icon: '🦾',
    description: (
      <>
        Design humanoid robot systems with kinematics, bipedal locomotion, and balance control.
        Integrate Vision-Language-Action models for conversational robotics.
      </>
    ),
    weekRange: 'Weeks 11-13',
    href: '/module-4-humanoid-vla/week-11/',
  },
];

function ModuleCard({title, icon, description, weekRange, href}: ModuleItem) {
  return (
    <div className={clsx('col col--6', styles.moduleCard)}>
      <div className={styles.card}>
        <div className={styles.cardIcon}>{icon}</div>
        <h3>{title}</h3>
        <p className={styles.weekRange}>{weekRange}</p>
        <p className={styles.description}>{description}</p>
        <Link className={styles.cardLink} to={href}>
          Start Module →
        </Link>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={styles.featuresTitle}>Course Modules</h2>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
