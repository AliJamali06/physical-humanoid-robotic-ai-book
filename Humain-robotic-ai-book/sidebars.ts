import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2) - Weeks 3-5',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 3: ROS 2 Fundamentals',
          collapsed: false,
          items: [
            'module-1/chapter-1-ros2-nodes-topics',
          ],
        },
        {
          type: 'category',
          label: 'Week 4: ROS 2 Core Concepts',
          collapsed: false,
          items: [
            'module-1/chapter-2-services',
            'module-1/chapter-4-packages',
            'module-1/chapter-3-urdf',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity) - Weeks 6-7',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 6: Gazebo Simulation Environment',
          collapsed: false,
          items: [
            'module-2/chapter-1-gazebo',
          ],
        },
        {
          type: 'category',
          label: 'Week 7: Unity & Sensor Simulation',
          collapsed: false,
          items: [
            'module-2/chapter-2-unity',
            'module-2/chapter-3-sensors',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac) - Weeks 8-10',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 8: Overview of the AI-Robot Brain',
          collapsed: false,
          items: [
            'module-3/chapter-0-overview',
          ],
        },
        {
          type: 'category',
          label: 'Week 9: NVIDIA Isaac Sim',
          collapsed: false,
          items: [
            'module-3/chapter-1-isaac-sim',
          ],
        },
        {
          type: 'category',
          label: 'Week 10: Isaac ROS & Navigation',
          collapsed: false,
          items: [
            'module-3/chapter-2-isaac-ros-vslam',
            'module-3/chapter-3-nav2',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) - Weeks 11-13',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 11: Overview of Vision-Language-Action Systems',
          collapsed: false,
          items: [
            'module-4/chapter-0-overview',
          ],
        },
        {
          type: 'category',
          label: 'Week 12: Voice-to-Action - Speech Interfaces',
          collapsed: false,
          items: [
            'module-4/chapter-1-whisper',
          ],
        },
        {
          type: 'category',
          label: 'Week 13: Vision-Language Perception and Action Execution',
          collapsed: false,
          items: [
            'module-4/chapter-2-llm-planning',
            'module-4/chapter-3-vla-capstone',
          ],
        },
      ],
    },
    'learning-outcomes',
  ],
};

export default sidebars;
