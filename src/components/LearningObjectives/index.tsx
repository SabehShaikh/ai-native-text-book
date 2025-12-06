import React from 'react';
import styles from './styles.module.css';

interface LearningObjectivesProps {
  children: React.ReactNode;
}

export default function LearningObjectives({children}: LearningObjectivesProps): JSX.Element {
  return (
    <div className={styles.objectivesContainer}>
      <div className={styles.objectivesContent}>
        {children}
      </div>
    </div>
  );
}
