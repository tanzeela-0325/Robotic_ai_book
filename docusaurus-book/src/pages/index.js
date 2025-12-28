import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}

function Card({title, description, to}) {
  return (
    <div className="col col--3 margin-bottom--lg">
      <Link to={to} className={styles.cardLink}>
        <div className={clsx('card', styles.card)}>
          <div className="card__body">
            <h3>{title}</h3>
            <p>{description}</p>
          </div>
          <div className="card__footer">
            <button className="button button--primary button--block">Learn More</button>
          </div>
        </div>
      </Link>
    </div>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="AI-Native Physical Humanoid Robotics Book">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <Card
                title="Module 1: The Robotic Nervous System (ROS 2)"
                description="Learn the fundamentals of AI-native humanoid robotics"
                to="/docs/modules/module-1-ros-nervous-system"
              />
              <Card
                title="Module 2: The Digital Twin (Gazebo & Unity)"
                description="Deep dive into humanoid robot architecture and design"
                to="/docs/modules/module-2-digital-twin"
              />
              <Card
                title="Module 3: The AI-Robot Brain (NVIDIA Isaac™)"
                description="Master simulation environments for humanoid robots"
                to="/docs/modules/module-3-ai-brain"
              />
              <Card
                title="Module 4: Vision-Language-Action (VLA)"
                description="Explore AI cognition and decision making in robotics"
                to="/docs/modules/module-4-vla"
              />
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}