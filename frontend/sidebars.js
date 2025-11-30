/**
 * Creating a sidebar enables you to:
 - Create an ordered group of docs
 - Render a sidebar for each doc of that group
 - Say "hello" to Docusaurus once more!

  `module.exports` is an ES5 export, it must be used instead of `export default`
  because Docusaurus's Node API cannot currently handle ES6 exports when
  resolving sidebars from a file.
*/

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/introduction',
        'module1/ros2-fundamentals',
        'module1/nodes-topics-services',
        'module1/python-agents-ros',
        'module1/urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/introduction',
        'module2/gazebo-simulation',
        'module2/unity-rendering',
        'module2/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module3/introduction',
        'module3/isaac-sim',
        'module3/isaac-ros',
        'module3/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/introduction',
        'module4/voice-to-action',
        'module4/cognitive-planning',
        'module4/capstone-project',
      ],
    },
  ],
};

module.exports = sidebars;
