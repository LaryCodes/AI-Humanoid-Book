import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug', 'cc8'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/config',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/config', '2b3'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/content',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/content', '6da'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/globalData',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/globalData', '0a7'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/metadata',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/metadata', '797'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/registry',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/registry', '4c8'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/routes',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/__docusaurus/debug/routes', '29b'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/404',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/404', 'ce6'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog', 'f1d'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/archive',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/archive', '412'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/authors',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/authors', 'b21'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/authors/all-sebastien-lorber-articles', '4a2'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/authors/yangshun',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/authors/yangshun', '288'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/first-blog-post',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/first-blog-post', 'c5a'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/long-blog-post',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/long-blog-post', '701'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/mdx-blog-post',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/mdx-blog-post', 'c6f'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/tags',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/tags', 'f61'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/tags/docusaurus',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/tags/docusaurus', '35d'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/tags/facebook',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/tags/facebook', 'ab4'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/tags/hello',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/tags/hello', 'dc6'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/tags/hola',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/tags/hola', 'd22'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/blog/welcome',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/blog/welcome', '5ca'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/design-showcase',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/design-showcase', '3d1'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/markdown-page',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/markdown-page', 'f9b'),
    exact: true
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/docs',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs', 'b9a'),
    routes: [
      {
        path: '/Physical-AI-Humanoid-Robotics-Book/docs',
        component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs', '5aa'),
        routes: [
          {
            path: '/Physical-AI-Humanoid-Robotics-Book/docs',
            component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs', 'd06'),
            routes: [
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/ai-robot-brain/',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/ai-robot-brain/', 'bea'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/ai-robot-brain/isaac-ros-perception',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/ai-robot-brain/isaac-ros-perception', 'c37'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/ai-robot-brain/nav2-path-planning',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/ai-robot-brain/nav2-path-planning', '530'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/ai-robot-brain/nvidia-isaac-sim',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/ai-robot-brain/nvidia-isaac-sim', 'b13'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/appendices/cloud-setup',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/appendices/cloud-setup', '799'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/appendices/hardware-guide',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/appendices/hardware-guide', '7e8'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/appendices/resources',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/appendices/resources', '59d'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/appendices/troubleshooting',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/appendices/troubleshooting', '6dd'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/category/tutorial---basics',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/category/tutorial---basics', '89c'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/category/tutorial---extras',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/category/tutorial---extras', 'ab9'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/digital-twin/',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/digital-twin/', 'e9b'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/digital-twin/gazebo-fundamentals',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/digital-twin/gazebo-fundamentals', '9b4'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/digital-twin/physics-simulation',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/digital-twin/physics-simulation', '068'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/digital-twin/unity-integration',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/digital-twin/unity-integration', '82f'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/glossary',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/glossary', '9f4'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/instructor-guide',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/instructor-guide', '51a'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/intro',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/intro', '012'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/', 'deb'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/introduction-physical-ai',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/introduction-physical-ai', '1fa'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/ros2-architecture',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/ros2-architecture', '65b'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/ros2-python-development',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/ros2-python-development', '5f6'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/urdf-robot-modeling',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/robotic-nervous-system/urdf-robot-modeling', 'b4f'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/student-resources/ros2_python_cheat_sheet',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/student-resources/ros2_python_cheat_sheet', '172'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/student-resources/vla_reference_card',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/student-resources/vla_reference_card', 'e0f'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/congratulations', '5f1'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/create-a-blog-post', 'e7e'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/create-a-document', '8d2'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/create-a-page', '3ce'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/deploy-your-site', 'cb8'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-basics/markdown-features', '415'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-extras/manage-docs-versions', '890'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/tutorial-extras/translate-your-site', 'e4c'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/video_embedding_instructions',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/video_embedding_instructions', '7d6'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/vision-language-action/',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/vision-language-action/', 'c63'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/vision-language-action/capstone-autonomous-humanoid',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/vision-language-action/capstone-autonomous-humanoid', '626'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/vision-language-action/llm-cognitive-planning',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/vision-language-action/llm-cognitive-planning', 'ad5'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/Physical-AI-Humanoid-Robotics-Book/docs/vision-language-action/voice-to-action',
                component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/docs/vision-language-action/voice-to-action', '897'),
                exact: true,
                sidebar: "mainSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/Physical-AI-Humanoid-Robotics-Book/',
    component: ComponentCreator('/Physical-AI-Humanoid-Robotics-Book/', 'd53'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
