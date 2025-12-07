import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  gradient: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Simulation to Reality',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    gradient: 'linear-gradient(135deg, #0ea5e9 0%, #14b8a6 100%)',
    description: (
      <>
        Master the complete pipeline from virtual environments to physical robots.
        Learn ROS2, Gazebo, Unity, and NVIDIA Isaac Sim for realistic simulation.
      </>
    ),
  },
  {
    title: 'AI-Native Robotics',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    gradient: 'linear-gradient(135deg, #14b8a6 0%, #a855f7 100%)',
    description: (
      <>
        Integrate cutting-edge AI with robotics. Implement perception, navigation,
        voice control, and LLM-powered cognitive planning for autonomous systems.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    gradient: 'linear-gradient(135deg, #a855f7 0%, #ec4899 100%)',
    description: (
      <>
        Build real projects with complete code examples. From basic concepts to
        advanced humanoid robots, every chapter includes practical implementations.
      </>
    ),
  },
];

function Feature({title, Svg, description, gradient}: FeatureItem) {
  return (
    <div className={clsx('col col--4', styles.featureCol)}>
      <div className={styles.featureCard}>
        <div className={styles.featureIconWrapper} style={{background: gradient}}>
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className={styles.featureContent}>
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Why This Book?
          </Heading>
          <p className={styles.sectionSubtitle}>
            A comprehensive, modern approach to building intelligent embodied systems
          </p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
