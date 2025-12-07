import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics: From Simulation to Reality',
  tagline: 'A comprehensive guide to building intelligent embodied systems.',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://github.com/LaryCodes', // Placeholder for GitHub Pages deployment
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/AI-Humanoid-Book/', // For GitHub Pages deployment, adjust as needed

  // GitHub pages deployment config.
  organizationName: 'LaryCodes', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics-Book', // Usually your repo name.

  onBrokenLinks: 'throw',

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
            'https://github.com/specifykit/ai-native-book/tree/main/', // Changed to project repo
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
            'https://github.com/specifykit/ai-native-book/tree/main/', // Changed to project repo
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
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Physical AI Book Logo',
        src: 'img/logo.svg',
      },
      hideOnScroll: false,
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'mainSidebar',
          position: 'left',
          label: '📚 Learn',
        },
        {
          to: '/blog', 
          label: '✍️ Blog', 
          position: 'left'
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/specifykit/ai-native-book',
          position: 'right',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'ROS2 Fundamentals',
              to: '/docs/category/ros2-fundamentals',
            },
            {
              label: 'Simulation',
              to: '/docs/category/simulation',
            },
            {
              label: 'AI Integration',
              to: '/docs/category/ai-integration',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/specifykit/ai-native-book/discussions',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/specifykit',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/specifykit',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/specifykit/ai-native-book',
            },
            {
              label: 'Code Examples',
              href: 'https://github.com/specifykit/ai-native-book/tree/main/code-examples',
            },
          ],
        },
        {
          title: 'About',
          items: [
            {
              label: 'SpecifyKit',
              href: 'https://specifykit.com',
            },
            {
              label: 'License',
              href: 'https://github.com/specifykit/ai-native-book/blob/main/LICENSE',
            },
            {
              label: 'Contributing',
              href: 'https://github.com/specifykit/ai-native-book/blob/main/CONTRIBUTING.md',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} SpecifyKit. Built with ❤️ and Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.vsDark,
      additionalLanguages: ['python', 'bash'],
    },
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true,
      },
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
