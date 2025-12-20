import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Do not use browser APIs here.

const config: Config = {
  title: 'Physical AI & Humanoid Robotics: From Simulation to Reality',
  tagline: 'A comprehensive guide to building intelligent embodied systems.',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // --- GitHub Pages deployment URL config ---
  url: 'https://larycodes.github.io',
  baseUrl: '/AI-Humanoid-Book/',
  organizationName: 'LaryCodes',
  projectName: 'AI-Humanoid-Book',

  onBrokenLinks: 'warn',

  markdown: {
    format: 'mdx',
    preprocessor: ({filePath, fileContent}) => {
      return fileContent;
    },
  },

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
          editUrl: 'https://github.com/LaryCodes/AI-Humanoid-Book/tree/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl: 'https://github.com/LaryCodes/AI-Humanoid-Book/tree/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
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
          position: 'left',
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/LaryCodes/AI-Humanoid-Book',
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
            { label: 'Introduction', to: '/docs/intro' },
            { label: 'ROS2 Fundamentals', to: '/docs/01-robotic-nervous-system' },
            { label: 'Simulation', to: '/docs/02-digital-twin' },
            { label: 'AI Integration', to: '/docs/03-ai-robot-brain' },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/LaryCodes/AI-Humanoid-Book/discussions',
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
            { label: 'Blog', to: '/blog' },
            { label: 'GitHub', href: 'https://github.com/LaryCodes/AI-Humanoid-Book' },
            { label: 'Code Examples', href: 'https://github.com/LaryCodes/AI-Humanoid-Book' },
          ],
        },
        {
          title: 'About',
          items: [
            { label: 'SpecifyKit', href: 'https://specifykit.com' },
            { label: 'License', href: 'https://github.com/specifykit/ai-native-book/blob/main/LICENSE' },
            {
              label: 'Contributing',
              href: 'https://github.com/specifykit/ai-native-book/blob/main/CONTRIBUTING.md',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} SpecifyKit.`,
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
