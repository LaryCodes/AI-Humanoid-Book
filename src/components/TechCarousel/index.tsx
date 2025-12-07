import type {ReactNode} from 'react';
import styles from './styles.module.css';

const technologies = [
  { name: 'ROS2', icon: '🤖' },
  { name: 'Python', icon: '🐍' },
  { name: 'Gazebo', icon: '🌐' },
  { name: 'Unity', icon: '🎮' },
  { name: 'NVIDIA Isaac', icon: '🚀' },
  { name: 'TensorFlow', icon: '🧠' },
  { name: 'PyTorch', icon: '🔥' },
  { name: 'OpenCV', icon: '👁️' },
  { name: 'Docker', icon: '🐳' },
  { name: 'Linux', icon: '🐧' },
];

export default function TechCarousel(): ReactNode {
  return (
    <section className={styles.carousel}>
      <div className="container">
        <h2 className={styles.carouselTitle}>Technologies You'll Master</h2>
        <div className={styles.carouselTrack}>
          <div className={styles.carouselContent}>
            {[...technologies, ...technologies].map((tech, idx) => (
              <div key={idx} className={styles.techCard}>
                <span className={styles.techIcon}>{tech.icon}</span>
                <span className={styles.techName}>{tech.name}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}
