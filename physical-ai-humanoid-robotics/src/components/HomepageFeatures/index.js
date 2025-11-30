import clsx from "clsx";
import Heading from "@theme/Heading";
import styles from "./styles.module.css";

const FeatureList = [
  {
    title: "Foundations of Robotics",
    Svg: require("@site/static/img/icons/ros.svg").default,
    description: (
      <>
        Begin your journey with core concepts of robotics, laying a strong
        foundation for understanding complex humanoid systems.
      </>
    ),
  },
  {
    title: "Simulation & Digital Twins",
    Svg: require("@site/static/img/icons/simulation.svg").default,
    description: (
      <>
        Explore virtual environments and master the creation of digital twins
        for realistic development and testing of AI-powered humanoids.
      </>
    ),
  },
  {
    title: "AI-Driven Autonomy",
    Svg: require("@site/static/img/icons/brain.svg").default,
    description: (
      <>
        Delve into the exciting world of AI brains, learning how to empower
        robots with cognitive abilities, perception, and intelligent
        decision-making.
      </>
    ),
  },
];

function Feature({ Svg, title, description }) {
  return (
    <div className={clsx("col col--4")}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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
