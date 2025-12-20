import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './404.module.css';

export default function NotFound(): React.ReactElement {
  return (
    <Layout
      title="Page Not Found"
      description="The page you are looking for does not exist">
      <main className={styles.notFoundContainer}>
        <div className={styles.notFoundContent}>
          <div className={styles.errorCode}>404</div>
          <Heading as="h1" className={styles.errorTitle}>
            Page Not Found
          </Heading>
          <p className={styles.errorDescription}>
            Oops! The page you're looking for seems to have wandered off into the robot void.
            Let's get you back on track.
          </p>
          <div className={styles.buttonGroup}>
            <Link
              className="button button--primary button--lg"
              to="/">
              🏠 Go Home
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro">
              📚 Browse Docs
            </Link>
          </div>
          <div className={styles.robotAnimation}>
            <div className={styles.robot}>🤖</div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
