# Quickstart Guide: Docusaurus Textbook Site

**Feature**: 001-docusaurus-textbook-site
**Date**: 2025-12-07
**Purpose**: Step-by-step guide for developers implementing the Physical AI & Humanoid Robotics textbook site

## Prerequisites

Before starting, ensure you have:

- **Node.js 18+**: Check with `node --version`
- **npm or yarn**: Check with `npm --version` or `yarn --version`
- **Git**: Check with `git --version`
- **GitHub account**: With repository access
- **Code editor**: VS Code recommended with Docusaurus extension

## Project Initialization

### Step 1: Create Docusaurus Project

```bash
# Create new Docusaurus site
npx create-docusaurus@latest ai-native-text-book classic --typescript

# Navigate to project directory
cd ai-native-text-book

# Install dependencies
npm install
```

**Expected Output**: Docusaurus project scaffolded with default configuration

### Step 2: Verify Installation

```bash
# Start development server
npm start
```

**Expected Output**: Browser opens at `http://localhost:3000` showing default Docusaurus site

**Verify**: Site loads without errors; you see default "Welcome to Docusaurus" page

Press `Ctrl+C` to stop the development server.

## Configuration

### Step 3: Configure docusaurus.config.js

Edit `docusaurus.config.js`:

```javascript
// @ts-check
// Note: type annotations allow type checking and IDE autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/vsDark');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive 13-week course covering Physical AI, ROS 2, robot simulation, and VLA systems',
  favicon: 'img/favicon.ico',

  // GitHub Pages deployment config
  url: 'https://YOUR_USERNAME.github.io',
  baseUrl: '/ai-native-text-book/',
  organizationName: 'YOUR_GITHUB_ORG',
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
          editUrl: 'https://github.com/YOUR_ORG/ai-native-text-book/edit/main/',
        },
        blog: false, // Disable blog
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

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
            href: 'https://github.com/YOUR_ORG/ai-native-text-book',
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
                href: 'https://github.com/YOUR_ORG/ai-native-text-book',
              },
              {
                label: 'Panaversity',
                href: 'https://panaversity.org',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Licensed under CC-BY-4.0.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'cpp', 'yaml', 'xml', 'bash'],
      },
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: false,
      },
    }),
};

module.exports = config;
```

**Replace**: `YOUR_USERNAME`, `YOUR_GITHUB_ORG` with actual values

### Step 4: Configure Sidebar

Edit `sidebars.js`:

```javascript
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
          items: [
            'module-1-physical-ai/week-1/index',
            'module-1-physical-ai/week-1/foundations',
            'module-1-physical-ai/week-1/applications',
          ],
        },
        // Add weeks 2-5 here
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Robot Simulation',
      collapsible: true,
      collapsed: true,
      items: [
        // Add weeks 6-7 here
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      collapsible: true,
      collapsed: true,
      items: [
        // Add weeks 8-10 here
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Humanoid Robotics & VLA',
      collapsible: true,
      collapsed: true,
      items: [
        // Add weeks 11-13 here
      ],
    },
  ],
};

module.exports = sidebars;
```

### Step 5: Apply Dark Theme Styling

Edit `src/css/custom.css`:

```css
/**
 * Physical AI Textbook - Custom Styles
 * Dark theme with Panaversity branding
 */

:root {
  /* Primary Colors (Panaversity Cyan) */
  --ifm-color-primary: #00d4ff;
  --ifm-color-primary-dark: #00b8e6;
  --ifm-color-primary-darker: #00a8d6;
  --ifm-color-primary-darkest: #0088b0;
  --ifm-color-primary-light: #1ae0ff;
  --ifm-color-primary-lighter: #2ee6ff;
  --ifm-color-primary-lightest: #5cefff;

  /* Dark Background Colors */
  --ifm-background-color: #0a0e1a;
  --ifm-background-surface-color: #141824;

  /* Text Colors */
  --ifm-font-color-base: #e8eaed;
  --ifm-heading-color: #ffffff;

  /* Code Colors */
  --ifm-code-background: #1e2430;
  --ifm-code-color: #00d4ff;

  /* Typography */
  --ifm-font-size-base: 16px;
  --ifm-line-height-base: 1.65;
  --ifm-font-family-base: 'Inter', system-ui, -apple-system, sans-serif;
  --ifm-font-family-monospace: 'Fira Code', 'JetBrains Mono', monospace;

  /* Spacing */
  --ifm-spacing-horizontal: 1.5rem;
  --ifm-spacing-vertical: 1.5rem;
}

/* Heading Sizes */
h1 {
  font-size: 2.25rem;
  font-weight: 700;
  margin-bottom: 1.5rem;
}

h2 {
  font-size: 1.75rem;
  font-weight: 600;
  margin-top: 2rem;
  margin-bottom: 1rem;
}

h3 {
  font-size: 1.375rem;
  font-weight: 600;
  margin-top: 1.5rem;
  margin-bottom: 0.75rem;
}

/* Code Blocks */
code {
  background-color: var(--ifm-code-background);
  color: var(--ifm-code-color);
  padding: 0.2em 0.4em;
  border-radius: 3px;
  font-size: 0.9em;
}

pre code {
  background-color: transparent;
  padding: 0;
}

/* Sidebar Customization */
.menu__link {
  border-radius: 4px;
  transition: background-color 0.2s ease;
}

.menu__link:hover {
  background-color: rgba(0, 212, 255, 0.1);
}

.menu__link--active {
  background-color: rgba(0, 212, 255, 0.2);
  border-left: 3px solid var(--ifm-color-primary);
}

/* Card Styling for Landing Page */
.card {
  background: var(--ifm-background-surface-color);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  padding: 1.5rem;
  transition: transform 0.2s ease, box-shadow 0.2s ease;
}

.card:hover {
  transform: translateY(-4px);
  box-shadow: 0 8px 16px rgba(0, 212, 255, 0.2);
}

/* Focus Indicators for Accessibility */
:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

/* Image Styling */
img {
  border-radius: 4px;
  max-width: 100%;
  height: auto;
}
```

## Directory Structure

### Step 6: Create Content Directories

```bash
# Remove default docs and blog
rm -rf docs blog

# Create module directories
mkdir -p docs/module-1-physical-ai/week-1
mkdir -p docs/module-2-simulation/week-6
mkdir -p docs/module-3-isaac/week-8
mkdir -p docs/module-4-humanoid-vla/week-11

# Create static assets directories
mkdir -p static/img/modules/module-1
mkdir -p static/img/modules/module-2
mkdir -p static/img/modules/module-3
mkdir -p static/img/modules/module-4
mkdir -p static/img/diagrams
```

### Step 7: Create Intro Page

Create `docs/intro.md`:

```markdown
---
sidebar_position: 1
title: Course Overview
description: Welcome to the Physical AI & Humanoid Robotics Textbook
---

# Physical AI & Humanoid Robotics Textbook

Welcome to a comprehensive 13-week course covering the fundamentals of Physical AI, ROS 2, robot simulation, NVIDIA Isaac platform, and Vision-Language-Action (VLA) systems.

## What You Will Learn

This course is structured into 4 modules spanning 13 weeks:

### Module 1: Introduction to Physical AI & ROS 2 (Weeks 1-5)
Learn the foundations of embodied intelligence and master ROS 2 for robotics development.

### Module 2: Robot Simulation (Weeks 6-7)
Explore Gazebo and Unity for realistic robot simulation environments.

### Module 3: NVIDIA Isaac Platform (Weeks 8-10)
Dive into Isaac SDK and Isaac Sim for perception, reinforcement learning, and sim-to-real transfer.

### Module 4: Humanoid Robotics & VLA (Weeks 11-13)
Study humanoid kinematics, bipedal locomotion, and cutting-edge VLA systems.

## Getting Started

Navigate to [Week 1: Physical AI Foundations](module-1-physical-ai/week-1/index.md) to begin your learning journey.

## Prerequisites

- Basic understanding of AI/ML concepts
- Python programming experience
- Familiarity with Linux command line (helpful but not required)

## Course Structure

Each week follows a consistent structure:
- **Learning Objectives**: What you'll achieve
- **Core Concepts**: Key technical concepts
- **Practical Explanation**: Detailed explanations with examples
- **Visual Aids**: Diagrams and illustrations
- **Summary**: Key takeaways and next steps
```

### Step 8: Create Week 1 Template

Create `docs/module-1-physical-ai/week-1/index.md`:

```markdown
---
id: week-1
sidebar_position: 1
title: "Week 1: Physical AI Foundations"
description: "Introduction to embodied intelligence, sensor-actuator loops, and real-world applications"
moduleId: module-1-physical-ai
weekNumber: 1
estimatedHours: 8
tags: [physical-ai, embodied-intelligence, foundations]
lastReviewed: 2025-12-07
---

# Week 1: Physical AI Foundations

## Learning Objectives

By the end of this week, you will be able to:

- Explain the concept of embodied intelligence and its applications in robotics
- Identify key components of sensor-actuator control loops
- Analyze real-world examples of Physical AI systems
- Understand the difference between traditional AI and Physical AI

## Core Concepts

### 1. Embodied Intelligence

**Definition**: The ability of a physical agent to interact with and learn from its environment through sensors and actuators.

**Context**: Unlike traditional AI that operates in digital environments, embodied intelligence requires dealing with real-world physics, noise, and uncertainty. This concept is fundamental to understanding Physical AI systems.

**Related Concepts**: Sensor-actuator loops, closed-loop control, sim-to-real transfer

### 2. Sensor-Actuator Loops

[Content to be added]

### 3. Physical AI vs Traditional AI

[Content to be added]

## Practical Explanation

[Detailed explanations with examples to be added]

## Visual Aids

[Diagrams and images to be added]

## Summary

[Key takeaways to be added]

---

**Next**: [Week 2: Embodied Intelligence](../week-2/index.md)
```

## Development Workflow

### Step 9: Run Development Server

```bash
# Start dev server with hot reloading
npm start
```

**Verify**:
- Site loads at `http://localhost:3000`
- Navigation sidebar shows modules
- Intro page displays correctly
- Week 1 page is accessible

### Step 10: Build for Production

```bash
# Create production build
npm run build
```

**Expected Output**: Build completes successfully; static files in `build/` directory

**Verify**:
```bash
# Serve production build locally
npm run serve
```

Site should load at `http://localhost:3000` with production optimizations applied.

## GitHub Pages Deployment

### Step 11: Create GitHub Repository

1. Create new repository: `ai-native-text-book`
2. Push local code to repository:

```bash
git init
git add .
git commit -m "Initial Docusaurus setup"
git branch -M main
git remote add origin https://github.com/YOUR_ORG/ai-native-text-book.git
git push -u origin main
```

### Step 12: Configure GitHub Actions

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: ./build

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

### Step 13: Enable GitHub Pages

1. Go to repository **Settings** → **Pages**
2. Under "Source", select **GitHub Actions**
3. Wait for deployment workflow to complete
4. Access site at `https://YOUR_USERNAME.github.io/ai-native-text-book/`

## Validation Checklist

Before proceeding to content development, verify:

- [ ] Development server runs without errors
- [ ] Production build completes successfully
- [ ] All 4 modules appear in sidebar
- [ ] Intro page displays correctly
- [ ] Week 1 template page loads
- [ ] Dark theme is active by default
- [ ] Code syntax highlighting works
- [ ] Navigation (prev/next) functions
- [ ] GitHub Actions workflow passes
- [ ] Site deploys to GitHub Pages
- [ ] Lighthouse score: Performance 90+, Accessibility 95+

## Next Steps

After completing this quickstart:

1. **Content Development**: Populate Week 1 with full content following the template structure
2. **Custom Components**: Develop React components for `LearningObjectives`, `ConceptCard`, etc.
3. **Landing Page**: Build custom homepage in `src/pages/index.js`
4. **Additional Weeks**: Create content for Weeks 2-13 following Week 1 pattern
5. **Assets**: Add optimized images and diagrams to `/static/img/`
6. **Search**: Install and configure Docusaurus local search plugin
7. **Testing**: Validate accessibility, performance, and mobile responsiveness

## Troubleshooting

### Build Fails with "Module not found"
- Verify all imports use correct paths
- Check that `docusaurus.config.js` has correct `baseUrl`

### GitHub Pages shows 404
- Ensure `baseUrl` in config matches repository name
- Verify GitHub Pages is enabled and set to "GitHub Actions" source

### Sidebar doesn't show all items
- Check `sidebars.js` syntax
- Verify all referenced document IDs exist in `/docs`

### Images don't load
- Ensure images are in `/static/img/` directory
- Use paths starting with `/img/` (not `./` or `../`)

## Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [MDX Documentation](https://mdxjs.com/)
- [GitHub Pages Docs](https://docs.github.com/en/pages)
- [Panaversity Branding Guide](https://ai-native.panaversity.org)

## Summary

You have now:
1. Initialized a Docusaurus project
2. Configured dark theme with Panaversity branding
3. Set up navigation with 4 modules
4. Created intro and Week 1 template pages
5. Configured GitHub Pages deployment
6. Validated the development and build process

You are ready to proceed with full content development following the implementation plan.
