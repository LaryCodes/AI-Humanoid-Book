import type {ReactNode} from 'react';
import styles from './styles.module.css';

const stats = [
  { number: '13', label: 'Comprehensive Chapters', icon: '📚' },
  { number: '4', label: 'Major Modules', icon: '🎯' },
  { number: '100+', label: 'Code Examples', icon: '💻' },
  { number: '∞', label: 'Learning Possibilities', icon: '🚀' },
];

export default function StatsSection(): ReactNode {
  return (
    <section className={styles.stats}>
      <div className="container">
        <div className={styles.statsGrid}>
          {stats.map((stat, idx) => (
            <div key={idx} className={styles.statCard}>
              <div className={styles.statIcon}>{stat.icon}</div>
              <div className={styles.statNumber}>{stat.number}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
