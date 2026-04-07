// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import { themes as prismThemes } from "prism-react-renderer";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "Bridging the Digital Brain and the Physical Body",
  favicon: "img/favicon.ico",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://neurobotics-book.vercel.app",
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: "/",
  trailingSlash: true,

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "Muskanateeq",
  projectName: "Physical-AI-Humanoid-Robotics-Book",
  deploymentBranch: "gh-pages",
  onBrokenLinks: "warn",

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: "./sidebars.js",
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ["rss", "atom"],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/Muskanateeq/Physical-AI-Humanoid-Robotics-Book/tree/main/",
          // Useful options to enforce blogging best practices
          onInlineTags: "warn",
          onInlineAuthors: "warn",
          onUntruncatedBlogPosts: "warn",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      }),
    ],
  ],

  scripts: [
    {
      src: "https://cdn.platform.openai.com/deployments/chatkit/chatkit.js",
      async: true,
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      clientModules: [
        './src/constants/api-url.js',
      ],
      // Replace with your project's social card
      image: "img/docusaurus-social-card.jpg",
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: "NEUROBOTICS",
        items: [
          {
            type: "doc",
            docId: "neurobotics-overview/intro-physical-ai",
            position: "left",
            label: "Course Modules",
          },

          {
            href: "https://github.com/Muskanateeq/Physical-AI-Humanoid-Robotics-Book",
            label: "GitHub",
            position: "right",
          },
        ],
      },
      footer: {
        style: "dark",
        links: [
          {
            title: "Course Content",
            items: [
              {
                label: "Course Modules1",
                to: "/docs/module-1-ros2/robotic-system",
              },
              {
                label: "Course Modules2",
                to: "/docs/module-2-simulation/digital-twin-concepts-gazebo-humanoid",
              },
              {
                label: "Course Modules3",
                to: "/docs/module-3-aibrain/isaac-sim-environment",
              },
              {
                label: "Course Modules4",
                to: "/docs/module-4-vla/vla-introduction-and-voice",
              },
            ],
          },
          {
            title: "Projects",
            items: [
              {
                label: "Module1 Mini Project",
                to: "/docs/module-1-ros2/urdf-humanoids",
              },
              {
                label: "Module2 Mini Project",
                to: "/docs/module-2-simulation/Unity-and-mini-project",
              },
              {
                label: "Module3 Mini Project",
                to: "/docs/module-3-aibrain/nav2-and-mini-project",
              },
              {
                label: "Module4 Capstone Project",
                to: "/docs/module-4-vla/ros2-execution-and-capstone",
              },
            ],
          },
          {
            title: "Resources",
            items: [
              {
                label: "GitHub",
                href: "https://github.com/Muskanateeq/Physical-AI-Humanoid-Robotics-Book",
              },
              {
                label: "ROS 2",
                href: "https://docs.ros.org/en/humble/",
              },
              {
                label: "NVIDIA Isaac",
                href: "https://developer.nvidia.com/isaac/sim",
              },
              {
                label: "Gazebo",
                href: "https://gazebosim.org/home",
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Neurobotics. Physical AI & Humanoid Robotics. Built By Muskan Atiq.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
