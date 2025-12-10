// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        'ros2-module-1/ros2-basics',
        'ros2-module-1/python-agents',
        'ros2-module-1/urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Environments',
      items: [
        'digital-twin-module/gazebo-physics',
        'digital-twin-module/unity-for-robotics',
        'digital-twin-module/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI Brain',
      items: [
        'ai-robot-brain-module/isaac-sim-basics',
        'ai-robot-brain-module/isaac-ros-vslam-nav',
        'ai-robot-brain-module/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Capstone Project',
      items: [
        'capstone-autonomous-humanoid/voice-to-action',
        'capstone-autonomous-humanoid/cognitive-planning-llm',
        'capstone-autonomous-humanoid/mini-capstone',
      ],
    },
  ],
};

export default sidebars;