/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'modules/module-1-ros-nervous-system',
        'modules/module-1-ros-nervous-system/chapter-1-architecture',
        'modules/module-1-ros-nervous-system/chapter-2-nodes-topics-services',
        'modules/module-1-ros-nervous-system/chapter-3-python-ai-agents',
        'modules/module-1-ros-nervous-system/chapter-4-urdf-xacro-morphology',
        'modules/module-1-ros-nervous-system/chapter-5-biological-mapping',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'modules/module-2-digital-twin',
        'modules/module-2-digital-twin/chapter-1-physics-engines',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/module-3-ai-brain',
        'modules/module-3-ai-brain/chapter-1-synthetic-data',
        'modules/module-3-ai-brain/chapter-2-isaac-sim-architecture',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'modules/module-4-vla',
        'modules/module-4-vla/chapter-1-multimodal-cognition',
        'modules/module-4-vla/chapter-2-speech-to-text',
        'modules/module-4-vla/chapter-3-language-to-plan',
        'modules/module-4-vla/chapter-4-plan-to-action',
        'modules/module-4-vla/chapter-5-safety-boundaries',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-project/pipeline-overview',
      ],
    },
  ],
};

module.exports = sidebars;