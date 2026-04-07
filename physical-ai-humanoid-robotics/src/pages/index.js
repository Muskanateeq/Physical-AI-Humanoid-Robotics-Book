import React from "react";
import Layout from "@theme/Layout";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import HomepageHero from "../components/HomepageHero";
import CourseRoadmap from "../components/CourseCardContent";
import TransformationRoadmap from "../components/TransformationRoadmap";
import AboutCreator from "../components/AboutCreator";
import FinalCTABanner from "../components/FinalCTABanner";
export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Home - ${siteConfig.title}`}
      description="Build the Future of Physical Intelligence with Humanoid Robotics. A complete, hands-on learning journey from ROS2 to Vision-Language-Action models."
    >
      <HomepageHero />
      <main>
        <CourseRoadmap />
        <TransformationRoadmap />
        <AboutCreator />
        <FinalCTABanner />
      </main>
    </Layout>
  );
}
