import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Humanoid Robotics AI',
  tagline: 'Physical AI & Robotics - Bridging the gap between digital brain and physical body',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'facebook', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.

  onBrokenLinks: 'warn',

  markdown: {
    mermaid: true,
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoid Robotics AI',
      logo: {
        alt: 'Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          type: 'dropdown',
          label: 'Quick Links',
          position: 'left',
          items: [
            {
              label: 'ROS 2 Basics',
              to: '/docs/module-1/chapter-1-ros2-nodes-topics',
            },
            {
              label: 'Digital Twin',
              to: '/docs/module-2/chapter-1-gazebo',
            },
            {
              label: 'Perception & Navigation',
              to: '/docs/module-3/chapter-1-isaac-sim',
            },
            {
              label: 'VLA Pipeline',
              to: '/docs/module-4/chapter-1-whisper',
            },
          ],
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/yourusername/humanoid-robotics-ai',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learning',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Learning Outcomes',
              to: '/docs/learning-outcomes',
            },
            {
              label: 'ROS 2 Basics',
              to: '/docs/module-1/chapter-1-ros2-nodes-topics',
            },
            {
              label: 'Digital Twin',
              to: '/docs/module-2/chapter-1-gazebo',
            },
          ],
        },
        {
          title: 'Advanced Topics',
          items: [
            {
              label: 'NVIDIA Isaac',
              to: '/docs/module-3/chapter-1-isaac-sim',
            },
            {
              label: 'Vision-Language-Action',
              to: '/docs/module-4/chapter-1-whisper',
            },
            {
              label: 'Navigation & SLAM',
              to: '/docs/module-3/chapter-3-nav2',
            },
            {
              label: 'Capstone Project',
              to: '/docs/module-4/chapter-3-vla-capstone',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'ROS Discourse',
              href: 'https://discourse.ros.org/',
            },
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/ros2',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/robotics',
            },
            {
              label: 'X (Twitter)',
              href: 'https://x.com/ros',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/AliJamali06/physical-humanoid-robotic-ai-book',
            },
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'NVIDIA Isaac',
              href: 'https://developer.nvidia.com/isaac',
            },
            {
              label: 'Gazebo',
              href: 'https://gazebosim.org/',
            },
          ],
        },
      ],
      copyright: `
        <div style="margin-top: 1rem;">
          <strong>Humanoid Robotics AI</strong> - Physical AI & Robotics Education
        </div>
        <div style="margin-top: 0.5rem; font-size: 0.85rem;">
          Copyright © ${new Date().getFullYear()} All Rights Reserved. Built with Docusaurus.
        </div>
        <div style="margin-top: 0.75rem; font-size: 0.8rem; opacity: 0.7;">
          Learn ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action systems
        </div>
      `,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
