import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/AI-Humanoid-Book/__docusaurus/debug',
    component: ComponentCreator('/AI-Humanoid-Book/__docusaurus/debug', '15e'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/__docusaurus/debug/config',
    component: ComponentCreator('/AI-Humanoid-Book/__docusaurus/debug/config', '068'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/__docusaurus/debug/content',
    component: ComponentCreator('/AI-Humanoid-Book/__docusaurus/debug/content', '41f'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/__docusaurus/debug/globalData',
    component: ComponentCreator('/AI-Humanoid-Book/__docusaurus/debug/globalData', '373'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/__docusaurus/debug/metadata',
    component: ComponentCreator('/AI-Humanoid-Book/__docusaurus/debug/metadata', 'a3a'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/__docusaurus/debug/registry',
    component: ComponentCreator('/AI-Humanoid-Book/__docusaurus/debug/registry', '6d3'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/__docusaurus/debug/routes',
    component: ComponentCreator('/AI-Humanoid-Book/__docusaurus/debug/routes', 'd47'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/404',
    component: ComponentCreator('/AI-Humanoid-Book/404', 'e4c'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog',
    component: ComponentCreator('/AI-Humanoid-Book/blog', 'd6d'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/archive',
    component: ComponentCreator('/AI-Humanoid-Book/blog/archive', '460'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/authors',
    component: ComponentCreator('/AI-Humanoid-Book/blog/authors', '44b'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/AI-Humanoid-Book/blog/authors/all-sebastien-lorber-articles', '822'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/authors/yangshun',
    component: ComponentCreator('/AI-Humanoid-Book/blog/authors/yangshun', '722'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/first-blog-post',
    component: ComponentCreator('/AI-Humanoid-Book/blog/first-blog-post', '5ac'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/long-blog-post',
    component: ComponentCreator('/AI-Humanoid-Book/blog/long-blog-post', '982'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/mdx-blog-post',
    component: ComponentCreator('/AI-Humanoid-Book/blog/mdx-blog-post', 'e56'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/tags',
    component: ComponentCreator('/AI-Humanoid-Book/blog/tags', '46b'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/tags/docusaurus',
    component: ComponentCreator('/AI-Humanoid-Book/blog/tags/docusaurus', 'fda'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/tags/facebook',
    component: ComponentCreator('/AI-Humanoid-Book/blog/tags/facebook', '1a3'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/tags/hello',
    component: ComponentCreator('/AI-Humanoid-Book/blog/tags/hello', '421'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/tags/hola',
    component: ComponentCreator('/AI-Humanoid-Book/blog/tags/hola', 'a88'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/blog/welcome',
    component: ComponentCreator('/AI-Humanoid-Book/blog/welcome', '2ee'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/design-showcase',
    component: ComponentCreator('/AI-Humanoid-Book/design-showcase', 'c21'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/markdown-page',
    component: ComponentCreator('/AI-Humanoid-Book/markdown-page', '5bc'),
    exact: true
  },
  {
    path: '/AI-Humanoid-Book/docs',
    component: ComponentCreator('/AI-Humanoid-Book/docs', 'bb3'),
    routes: [
      {
        path: '/AI-Humanoid-Book/docs',
        component: ComponentCreator('/AI-Humanoid-Book/docs', '190'),
        routes: [
          {
            path: '/AI-Humanoid-Book/docs',
            component: ComponentCreator('/AI-Humanoid-Book/docs', '782'),
            routes: [
              {
                path: '/AI-Humanoid-Book/docs/ai-robot-brain/',
                component: ComponentCreator('/AI-Humanoid-Book/docs/ai-robot-brain/', 'e46'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/ai-robot-brain/isaac-ros-perception',
                component: ComponentCreator('/AI-Humanoid-Book/docs/ai-robot-brain/isaac-ros-perception', '843'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/ai-robot-brain/nav2-path-planning',
                component: ComponentCreator('/AI-Humanoid-Book/docs/ai-robot-brain/nav2-path-planning', 'dd3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/ai-robot-brain/nvidia-isaac-sim',
                component: ComponentCreator('/AI-Humanoid-Book/docs/ai-robot-brain/nvidia-isaac-sim', '8f4'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/appendices/cloud-setup',
                component: ComponentCreator('/AI-Humanoid-Book/docs/appendices/cloud-setup', '254'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/appendices/hardware-guide',
                component: ComponentCreator('/AI-Humanoid-Book/docs/appendices/hardware-guide', '8aa'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/appendices/resources',
                component: ComponentCreator('/AI-Humanoid-Book/docs/appendices/resources', 'be0'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/appendices/troubleshooting',
                component: ComponentCreator('/AI-Humanoid-Book/docs/appendices/troubleshooting', '7f3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/category/tutorial---basics',
                component: ComponentCreator('/AI-Humanoid-Book/docs/category/tutorial---basics', '590'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/category/tutorial---extras',
                component: ComponentCreator('/AI-Humanoid-Book/docs/category/tutorial---extras', '892'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/digital-twin/',
                component: ComponentCreator('/AI-Humanoid-Book/docs/digital-twin/', '9c3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/digital-twin/gazebo-fundamentals',
                component: ComponentCreator('/AI-Humanoid-Book/docs/digital-twin/gazebo-fundamentals', '36d'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/digital-twin/physics-simulation',
                component: ComponentCreator('/AI-Humanoid-Book/docs/digital-twin/physics-simulation', 'a2a'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/digital-twin/unity-integration',
                component: ComponentCreator('/AI-Humanoid-Book/docs/digital-twin/unity-integration', '967'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/glossary',
                component: ComponentCreator('/AI-Humanoid-Book/docs/glossary', '932'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/instructor-guide',
                component: ComponentCreator('/AI-Humanoid-Book/docs/instructor-guide', 'ac4'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/intro',
                component: ComponentCreator('/AI-Humanoid-Book/docs/intro', 'f92'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/robotic-nervous-system/',
                component: ComponentCreator('/AI-Humanoid-Book/docs/robotic-nervous-system/', '9d4'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/robotic-nervous-system/introduction-physical-ai',
                component: ComponentCreator('/AI-Humanoid-Book/docs/robotic-nervous-system/introduction-physical-ai', '777'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/robotic-nervous-system/ros2-architecture',
                component: ComponentCreator('/AI-Humanoid-Book/docs/robotic-nervous-system/ros2-architecture', 'f9b'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/robotic-nervous-system/ros2-python-development',
                component: ComponentCreator('/AI-Humanoid-Book/docs/robotic-nervous-system/ros2-python-development', 'a8d'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/robotic-nervous-system/urdf-robot-modeling',
                component: ComponentCreator('/AI-Humanoid-Book/docs/robotic-nervous-system/urdf-robot-modeling', 'e5e'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/student-resources/ros2_python_cheat_sheet',
                component: ComponentCreator('/AI-Humanoid-Book/docs/student-resources/ros2_python_cheat_sheet', 'da9'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/student-resources/vla_reference_card',
                component: ComponentCreator('/AI-Humanoid-Book/docs/student-resources/vla_reference_card', '874'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/AI-Humanoid-Book/docs/tutorial-basics/congratulations', '094'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/AI-Humanoid-Book/docs/tutorial-basics/create-a-blog-post', 'c53'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/AI-Humanoid-Book/docs/tutorial-basics/create-a-document', 'd4c'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/AI-Humanoid-Book/docs/tutorial-basics/create-a-page', 'db3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/AI-Humanoid-Book/docs/tutorial-basics/deploy-your-site', '477'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/AI-Humanoid-Book/docs/tutorial-basics/markdown-features', '0c9'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/AI-Humanoid-Book/docs/tutorial-extras/manage-docs-versions', '252'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/AI-Humanoid-Book/docs/tutorial-extras/translate-your-site', '21c'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/video_embedding_instructions',
                component: ComponentCreator('/AI-Humanoid-Book/docs/video_embedding_instructions', '5c3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/vision-language-action/',
                component: ComponentCreator('/AI-Humanoid-Book/docs/vision-language-action/', 'bb2'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/vision-language-action/capstone-autonomous-humanoid',
                component: ComponentCreator('/AI-Humanoid-Book/docs/vision-language-action/capstone-autonomous-humanoid', '2a7'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/vision-language-action/llm-cognitive-planning',
                component: ComponentCreator('/AI-Humanoid-Book/docs/vision-language-action/llm-cognitive-planning', '286'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/AI-Humanoid-Book/docs/vision-language-action/voice-to-action',
                component: ComponentCreator('/AI-Humanoid-Book/docs/vision-language-action/voice-to-action', '590'),
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
    path: '/AI-Humanoid-Book/',
    component: ComponentCreator('/AI-Humanoid-Book/', '633'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
