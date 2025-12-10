import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Robotic Nervous System',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn ROS 2 fundamentals: nodes, topics, services, and URDF for
        distributed robot control systems. Master the foundation of modern robotics.
      </>
    ),
    link: '/docs/module-1/chapter-1-ros2-nodes-topics',
  },
  {
    title: 'Digital Twin (Gazebo & Unity)',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Master physics simulation with Gazebo, photorealistic rendering with Unity,
        and sensor simulation (LiDAR, depth cameras, IMU) for humanoid robots.
      </>
    ),
    link: '/docs/module-2/chapter-1-gazebo',
  },
  {
    title: 'Perception & Navigation',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Explore visual SLAM with Isaac ROS, autonomous navigation with Nav2,
        and perception pipelines for humanoid robot spatial awareness.
      </>
    ),
    link: '/docs/module-3/chapter-1-ros2-introduction',
  },
  {
    title: 'Vision-Language-Action (VLA)',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Integrate Whisper for voice commands, LLM-based cognitive planning,
        and end-to-end VLA pipelines for natural human-robot interaction.
      </>
    ),
    link: '/docs/module-4/chapter-1-whisper',
  },
];

function Feature({title, Svg, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--6')}>
      <div className={styles.featureCard}>
        <div className="text--center">
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className="text--center">
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
          <a href={link} className={styles.featureLink}>
            Start Learning
          </a>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
