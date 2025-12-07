import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function CTASection(): ReactNode {
  return (
    <section className={styles.cta}>
      <div className="container">
        <div className={styles.ctaContent}>
          <h2 className={styles.ctaTitle}>Ready to Build the Future?</h2>
          <p className={styles.ctaDescription}>
            Join thousands of developers learning to create intelligent robots that can see, think, and act.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning Now 🚀
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="https://github.com/specifykit/ai-native-book">
              View on GitHub ⭐
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}
