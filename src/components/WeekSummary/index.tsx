import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface WeekSummaryProps {
  children: React.ReactNode;
  nextWeek?: {
    title: string;
    href: string;
  };
}

export default function WeekSummary({children, nextWeek}: WeekSummaryProps): JSX.Element {
  return (
    <div className={styles.summaryContainer}>
      <div className={styles.summaryContent}>
        {children}
      </div>
      {nextWeek && (
        <div className={styles.nextWeekLink}>
          <Link to={nextWeek.href} className={styles.nextWeekButton}>
            Next: {nextWeek.title} →
          </Link>
        </div>
      )}
    </div>
  );
}
