// @ts-check
// Note: type annotations allow type checking and IDE autocompletion

const {themes} = require('prism-react-renderer');
const lightCodeTheme = themes.github;
const darkCodeTheme = themes.vsDark;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive 13-week course covering Physical AI, ROS 2, robot simulation, and VLA systems',
  favicon: 'img/favicon.ico',

  // GitHub Pages deployment config
  url: 'https://sabehshaikh.github.io',
  baseUrl: '/ai-native-text-book/',
  organizationName: 'SabehShaikh',
  projectName: 'ai-native-text-book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: '/', // Docs-only mode
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/SabehShaikh/ai-native-text-book/edit/main/',
        },
        blog: false, // Disable blog
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        language: ['en'],
        explicitSearchResultPath: true,
        docsRouteBasePath: '/',
      },
    ],
  ],

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.jpg',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Course',
          },
          {
            href: 'https://github.com/SabehShaikh/ai-native-text-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Course',
            items: [
              {
                label: 'Start Learning',
                to: '/',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'NVIDIA Isaac Sim',
                href: 'https://developer.nvidia.com/isaac-sim',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/SabehShaikh/ai-native-text-book',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/sabeh-shaikh/',
              },
            ],
          },
        ],
        copyright: `Built by Sabeh Shaikh © ${new Date().getFullYear()}`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'cpp', 'yaml', 'bash'],
      },
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: false,
      },
    }),
};

module.exports = config;
