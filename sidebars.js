/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Course Overview',
    },
    {
      type: 'category',
      label: 'Module 1: Introduction to Physical AI & ROS 2',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 1: Physical AI Foundations',
          items: ['module-1-physical-ai/week-1/week-1'],
        },
        {
          type: 'category',
          label: 'Week 2: Embodied Intelligence',
          items: ['module-1-physical-ai/week-2/week-2'],
        },
        {
          type: 'category',
          label: 'Week 3: ROS 2 Basics',
          items: ['module-1-physical-ai/week-3/week-3'],
        },
        {
          type: 'category',
          label: 'Week 4: ROS 2 Communication',
          items: ['module-1-physical-ai/week-4/week-4'],
        },
        {
          type: 'category',
          label: 'Week 5: ROS 2 Packages',
          items: ['module-1-physical-ai/week-5/week-5'],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Robot Simulation',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Week 6: Gazebo Simulation',
          items: ['module-2-simulation/week-6/week-6'],
        },
        {
          type: 'category',
          label: 'Week 7: Unity Robotics',
          items: ['module-2-simulation/week-7/week-7'],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Week 8: Isaac SDK',
          items: ['module-3-isaac/week-8/week-8'],
        },
        {
          type: 'category',
          label: 'Week 9: Isaac Sim & Perception',
          items: ['module-3-isaac/week-9/week-9'],
        },
        {
          type: 'category',
          label: 'Week 10: RL & Sim-to-Real',
          items: ['module-3-isaac/week-10/week-10'],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Humanoid Robotics & VLA',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Week 11: Humanoid Kinematics',
          items: ['module-4-humanoid-vla/week-11/week-11'],
        },
        {
          type: 'category',
          label: 'Week 12: Bipedal Locomotion',
          items: ['module-4-humanoid-vla/week-12/week-12'],
        },
        {
          type: 'category',
          label: 'Week 13: VLA Systems',
          items: ['module-4-humanoid-vla/week-13/week-13'],
        },
      ],
    },
  ],
};

module.exports = sidebars;
