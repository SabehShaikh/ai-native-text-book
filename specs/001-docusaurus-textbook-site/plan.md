# Implementation Plan: Docusaurus Textbook Site

**Branch**: `001-docusaurus-textbook-site` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-docusaurus-textbook-site/spec.md`

## Summary

Build a production-ready Docusaurus v3 static site for the Physical AI & Humanoid Robotics textbook. The site will deliver 13 weeks of educational content organized into 4 modules, with dark theme styling, responsive design, GitHub Pages deployment, and full accessibility compliance. This implementation follows a docs-only architecture with custom landing page, module-based navigation, and reusable React components for consistent content presentation.

**Technical Approach**: Use Docusaurus classic preset with TypeScript, MDX for content, custom CSS for Panaversity branding, local search plugin, and GitHub Actions for CI/CD. Content follows standardized frontmatter schema with validation at build time.

## Technical Context

**Language/Version**: Node.js 18+, TypeScript 5.x, React 18
**Primary Dependencies**: Docusaurus 3.x, React 18, MDX, Prism (syntax highlighting), @easyops-cn/docusaurus-search-local
**Storage**: Markdown/MDX files with YAML frontmatter (no database)
**Testing**: Lighthouse CI, broken link checking, frontmatter schema validation, accessibility audits
**Target Platform**: Static site hosted on GitHub Pages (HTTPS)
**Project Type**: Web (static site generation)
**Performance Goals**: FCP <1.5s, page size <500KB, search response <1s, Lighthouse score 90+
**Constraints**: Static site only (no SSR), GitHub Pages 1GB limit, 5-minute build time limit
**Scale/Scope**: 13 weeks × ~50-100 total pages, 4 modules, ~100-200 images/diagrams

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Clarity First
**Status**: ✅ PASS

- Content template enforces clear structure (Learning Objectives → Core Concepts → Practical Explanation → Visual Aids → Summary)
- All technical concepts must have definitions and context
- Frontmatter requires `description` field for every week
- **Justification**: Standardized content structure ensures consistent clarity across all 13 weeks

### Principle II: Consistency
**Status**: ✅ PASS

- Unified frontmatter schema (week-frontmatter.schema.json) ensures metadata consistency
- All weeks follow identical MDX structure
- Sidebar configuration uses consistent labeling pattern ("Week X: Topic")
- Custom CSS applies uniform typography and spacing
- **Justification**: Data model and contracts enforce consistency at build time

### Principle III: Simplicity
**Status**: ✅ PASS

- No backend, database, or server-side logic
- Minimal dependencies (Docusaurus classic preset + search plugin only)
- Static site generation reduces complexity
- No feature bloat: only essential features (docs, search, navigation)
- **Justification**: Static site architecture is simplest viable approach for textbook content

### Principle IV: Modularity
**Status**: ✅ PASS

- 4 self-contained modules with explicit boundaries
- Each week is self-contained MDX file with frontmatter metadata
- Frontmatter includes `prerequisites` field for explicit dependencies
- Prev/next navigation allows flexible learning paths
- **Justification**: Directory structure (`/module-X/week-Y/`) enforces modular organization

### Principle V: Accessibility
**Status**: ✅ PASS

- WCAG 2.1 AA compliance required (color contrast 4.5:1+)
- All images must have alt text (enforced in content template)
- Keyboard navigation supported by default in Docusaurus
- Semantic HTML with proper heading hierarchy
- Focus indicators with 2px solid cyan outline
- Screen reader compatibility via ARIA labels
- **Justification**: Accessibility built into architecture through Docusaurus defaults and custom CSS enhancements

**Overall Gate Result**: ✅ PASS - All principles satisfied without violations

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-textbook-site/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (technology decisions)
├── data-model.md        # Phase 1 output (content entities)
├── quickstart.md        # Phase 1 output (developer guide)
├── contracts/           # Phase 1 output (API contracts)
│   ├── docusaurus-config.schema.json
│   ├── sidebar-config.schema.json
│   ├── week-frontmatter.schema.json
│   └── component-props.ts
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
ai-native-text-book/
├── docs/                          # All course content (MDX files)
│   ├── intro.md                   # Course overview/landing content
│   ├── module-1-physical-ai/      # Module 1 (Weeks 1-5)
│   │   ├── week-1/
│   │   │   ├── index.md           # Main chapter page
│   │   │   ├── foundations.md     # Subchapter
│   │   │   └── applications.md    # Subchapter
│   │   ├── week-2/
│   │   │   └── index.md
│   │   ├── week-3/
│   │   ├── week-4/
│   │   └── week-5/
│   ├── module-2-simulation/       # Module 2 (Weeks 6-7)
│   │   ├── week-6/
│   │   │   ├── index.md
│   │   │   ├── gazebo-basics.md
│   │   │   └── urdf-sdf.md
│   │   └── week-7/
│   │       └── index.md
│   ├── module-3-isaac/            # Module 3 (Weeks 8-10)
│   │   ├── week-8/
│   │   ├── week-9/
│   │   └── week-10/
│   └── module-4-humanoid-vla/     # Module 4 (Weeks 11-13)
│       ├── week-11/
│       ├── week-12/
│       └── week-13/
├── src/
│   ├── components/                # Custom React components
│   │   ├── HomepageFeatures/      # Landing page module cards
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── LearningObjectives/    # Reusable content components
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── ConceptCard/
│   │   ├── CodeExample/
│   │   ├── WeekSummary/
│   │   └── VisualAid/
│   ├── css/                       # Global styles
│   │   └── custom.css             # Dark theme + Panaversity branding
│   └── pages/                     # Custom pages
│       └── index.tsx              # Landing page
├── static/
│   ├── img/                       # Static images
│   │   ├── modules/
│   │   │   ├── module-1/          # Module-specific images
│   │   │   ├── module-2/
│   │   │   ├── module-3/
│   │   │   └── module-4/
│   │   ├── diagrams/              # SVG diagrams
│   │   ├── logo.svg
│   │   └── favicon.ico
│   └── fonts/                     # Custom fonts (if needed)
├── .github/
│   └── workflows/
│       └── deploy.yml             # GitHub Actions deployment
├── docusaurus.config.js           # Main Docusaurus configuration
├── sidebars.js                    # Sidebar navigation structure
├── tsconfig.json                  # TypeScript configuration
├── package.json
└── README.md
```

**Structure Decision**: Web application structure with docs-only mode. The `/docs` directory contains all educational content organized by module and week. Custom React components in `/src/components` provide reusable UI patterns. Static assets in `/static` are served directly by GitHub Pages. This structure aligns with Docusaurus best practices and supports the modular course organization.

## Complexity Tracking

> **No constitutional violations** - this section is not applicable.

## Architecture Overview

### 1. Docusaurus Architecture

**Core System**: Docusaurus v3 is a static site generator built on React. It compiles MDX content into optimized static HTML, CSS, and JavaScript bundles. The classic preset bundles docs plugin, theme, and utilities.

**Docs-Only Mode**: We configure `routeBasePath: '/'` to serve documentation at the site root, disabling the blog feature. This simplifies navigation and focuses on educational content.

**MDX Processing**: Content files use MDX (Markdown + JSX), allowing React components within markdown. Frontmatter (YAML metadata) provides structured data for navigation, search, and SEO.

**Build Pipeline**:
1. Read MDX files from `/docs`
2. Parse frontmatter and validate against schema
3. Transform MDX to React components
4. Generate sidebar from `sidebars.js`
5. Apply custom CSS theme
6. Bundle with Webpack (code splitting, minification)
7. Output static HTML/CSS/JS to `/build`

**Routing**: Docusaurus auto-generates routes from directory structure. Example: `/docs/module-1-physical-ai/week-1/index.md` → `/module-1-physical-ai/week-1/`

### 2. Dark Theme & Panaversity Branding

**Theme Customization**: Docusaurus classic theme uses CSS variables. We override variables in `src/css/custom.css`:

- **Primary Color**: `--ifm-color-primary: #00d4ff` (Panaversity cyan)
- **Background**: `--ifm-background-color: #0a0e1a` (dark navy)
- **Surface Color**: `--ifm-background-surface-color: #141824` (elevated dark)
- **Text**: `--ifm-font-color-base: #e8eaed` (light gray)
- **Code Background**: `--ifm-code-background: #1e2430` (darker surface)

**Color Mode**: Default to dark mode via `themeConfig.colorMode.defaultMode: 'dark'`. Users can toggle light/dark with switch in navbar.

**Typography**:
- Body: 16px (1rem) for readability
- H1: 2.25rem (36px)
- H2: 1.75rem (28px)
- H3: 1.375rem (22px)
- Font Family: Inter (sans-serif), Fira Code (monospace)

**Contrast**: Cyan (#00d4ff) on dark background (#0a0e1a) = 7.2:1 ratio (exceeds WCAG AA 4.5:1)

### 3. Search Strategy

**Local Search Plugin**: Use `@easyops-cn/docusaurus-search-local` for client-side search. This plugin indexes all content at build time and provides instant search without external dependencies.

**Configuration**:
```javascript
plugins: [
  [
    require.resolve('@easyops-cn/docusaurus-search-local'),
    {
      hashed: true,
      language: ['en'],
      highlightSearchTerms: true,
      explicitSearchResultPath: true,
      docsRouteBasePath: '/'
    }
  ]
]
```

**Index Generation**: Plugin crawls MDX files, extracts headings, paragraphs, and code blocks, and builds a search index. Index is loaded asynchronously on first search query.

**Performance**: Local search typically responds in <100ms for 50-100 pages. If content grows beyond 100 pages or search latency exceeds 1s, we can migrate to Algolia DocSearch (requires manual application).

### 4. Static Asset Handling

**Images**:
- **Format**: WebP primary (25-35% smaller than JPEG), JPEG/PNG fallback
- **Optimization**: Compress to 80% quality, max width 1200px
- **Lazy Loading**: Docusaurus automatically lazy-loads images below fold
- **Path**: Store in `/static/img/`, reference as `/img/path/to/image.webp`

**Diagrams**:
- **Mermaid**: For simple diagrams (flowcharts, sequence diagrams), embed Mermaid syntax in MDX using `@docusaurus/theme-mermaid`
- **SVG**: For complex diagrams (architecture, system design), export from Excalidraw/Figma as SVG and place in `/static/img/diagrams/`
- **Alt Text**: All diagrams must have descriptive alt text for screen readers

**Fonts**: Use web-safe system fonts to avoid additional requests. If custom fonts needed, subset to Latin characters only and preload.

### 5. Routing, Navigation, and MDX

**Routing**: Docusaurus maps file paths to URLs:
- `/docs/intro.md` → `/`
- `/docs/module-1-physical-ai/week-1/index.md` → `/module-1-physical-ai/week-1/`

**Sidebar Navigation**: Defined in `sidebars.js` as nested categories. Each category can be collapsible. Module 1 defaults to `collapsed: false` for immediate access.

**Prev/Next Navigation**: Docusaurus auto-generates prev/next links based on sidebar order. Can override with `pagination_prev` and `pagination_next` in frontmatter.

**MDX Usage**:
- Import custom components: `import LearningObjectives from '@site/src/components/LearningObjectives'`
- Use components in content: `<LearningObjectives>...</LearningObjectives>`
- Mermaid diagrams: Triple-backtick mermaid blocks (requires `@docusaurus/theme-mermaid`)

## Landing Page Plan

### Structure

**Layout**: Custom React page in `src/pages/index.tsx` (overrides default docs landing)

**Sections**:
1. **Hero Section**:
   - Title: "Physical AI & Humanoid Robotics Textbook"
   - Subtitle: "A comprehensive 13-week course..."
   - Cover image (hero banner)
   - "Start Reading" CTA button → Week 1

2. **Course Badges**:
   - "13 Weeks" badge
   - "4 Modules" badge
   - "Open Source" badge
   - "CC-BY-4.0" badge

3. **Summary Section ("What You Will Learn")**:
   - Brief paragraph intro
   - 4-5 bullet points with key learning outcomes

4. **Module Overview Boxes**:
   - Grid of 4 cards (one per module)
   - Each card:
     - Module number and title
     - Icon/image
     - 2-3 sentence description
     - Week range (e.g., "Weeks 1-5")
     - "Start Module" link

5. **Footer** (reuse Docusaurus footer):
   - Links to GitHub, Panaversity
   - Copyright notice

**Responsive Layout**:
- Desktop (1200px+): 2×2 grid for module cards
- Tablet (768px-1199px): 2×2 grid or single column
- Mobile (375px-767px): Single column stack

**CSS**: Custom CSS module in `src/pages/index.module.css` with Flexbox/Grid layout

### Component Breakdown

**HomepageFeatures Component**:
```tsx
// src/components/HomepageFeatures/index.tsx
type ModuleFeature = {
  title: string;
  icon: string;
  description: string;
  weekRange: string;
  href: string;
};

const moduleFeatures: ModuleFeature[] = [
  {
    title: 'Module 1: Physical AI & ROS 2',
    icon: '🤖',
    description: 'Foundations of embodied intelligence and ROS 2...',
    weekRange: 'Weeks 1-5',
    href: '/module-1-physical-ai/week-1/'
  },
  // ... modules 2-4
];

export default function HomepageFeatures() {
  return (
    <section>
      <div className="container">
        <div className="row">
          {moduleFeatures.map((feature, idx) => (
            <ModuleCard key={idx} {...feature} />
          ))}
        </div>
      </div>
    </section>
  );
}
```

## Navigation Plan

### Sidebar Configuration

**Structure**: Hierarchical sidebar with 3 levels:
1. **Level 1**: Course Overview (doc link)
2. **Level 2**: Modules (categories)
3. **Level 3**: Weeks (categories) and subpages (doc links)

**Sidebar Definition** (`sidebars.js`):
```javascript
tutorialSidebar: [
  {
    type: 'doc',
    id: 'intro',
    label: 'Course Overview'
  },
  {
    type: 'category',
    label: 'Module 1: Introduction to Physical AI & ROS 2',
    collapsible: true,
    collapsed: false,  // Module 1 expanded by default
    items: [
      {
        type: 'category',
        label: 'Week 1: Physical AI Foundations',
        items: [
          'module-1-physical-ai/week-1/index',
          'module-1-physical-ai/week-1/foundations',
          'module-1-physical-ai/week-1/applications'
        ]
      },
      // Weeks 2-5...
    ]
  },
  // Modules 2-4 with collapsed: true
]
```

**Navbar Elements**:
- Left: "Course" link (opens sidebar)
- Right: GitHub icon link

**Breadcrumbs**: Docusaurus provides breadcrumbs automatically (e.g., "Home > Module 1 > Week 1")

**Chapter Ordering**: Controlled by `sidebar_position` in frontmatter and array order in `sidebars.js`

## Content Plan

### 13-Week to 4-Module Mapping

| Module | Weeks | Topics | Chapters |
|--------|-------|--------|----------|
| Module 1: Physical AI & ROS 2 | 1-5 | Physical AI, ROS 2 | 5 weeks × 2-3 subpages = 10-15 pages |
| Module 2: Robot Simulation | 6-7 | Gazebo, Unity | 2 weeks × 2-3 subpages = 4-6 pages |
| Module 3: NVIDIA Isaac | 8-10 | Isaac SDK, Isaac Sim, RL | 3 weeks × 2-3 subpages = 6-9 pages |
| Module 4: Humanoid & VLA | 11-13 | Humanoid robotics, VLA | 3 weeks × 2-3 subpages = 6-9 pages |

**Total**: ~26-39 pages across 13 weeks (plus intro page = ~30-40 pages)

### Week-by-Week Outline

#### Module 1: Introduction to Physical AI & ROS 2

**Week 1: Physical AI Foundations**
- Learning Objectives: Explain embodied intelligence, identify sensor-actuator loops, analyze Physical AI applications
- Core Concepts:
  - Embodied Intelligence (definition, context, vs traditional AI)
  - Sensor-Actuator Control Loops (feedback, closed-loop control)
  - Real-World Applications (autonomous vehicles, humanoid robots, drones)
- Practical Explanation: Case studies of Physical AI systems
- Visual Aids: Control loop diagram, application examples
- Summary: Key takeaways, transition to Week 2

**Week 2: Embodied Intelligence Deep Dive**
- Core Concepts: Perception-action coupling, environmental interaction, learning from experience
- Practical: Examples from robotics (grasping, navigation)

**Week 3: ROS 2 Basics**
- Core Concepts: ROS 2 architecture, nodes, workspaces, packages
- Code Examples: Creating first ROS 2 node in Python

**Week 4: ROS 2 Communication**
- Core Concepts: Topics, services, actions, parameters
- Code Examples: Publisher/subscriber, service client/server

**Week 5: ROS 2 Package Development**
- Core Concepts: Package structure, launch files, build system (colcon)
- Practical: Building a complete ROS 2 package

#### Module 2: Robot Simulation

**Week 6: Gazebo Simulation**
- Core Concepts: Gazebo setup, URDF/SDF formats, physics simulation
- Code Examples: Creating robot models, launching simulations

**Week 7: Unity Robotics**
- Core Concepts: Unity integration, Unity Robotics Hub, ROS-Unity communication
- Practical: Setting up Unity simulation environment

#### Module 3: NVIDIA Isaac Platform

**Week 8: Isaac SDK Introduction**
- Core Concepts: Isaac SDK architecture, modules, codelets
- Code Examples: Building Isaac applications

**Week 9: Isaac Sim & Perception**
- Core Concepts: Isaac Sim setup, sensor simulation, perception systems
- Practical: Running perception algorithms in Isaac Sim

**Week 10: Reinforcement Learning & Sim-to-Real**
- Core Concepts: RL for robot control, sim-to-real transfer techniques
- Code Examples: Training RL agents in Isaac Sim

#### Module 4: Humanoid Robotics & VLA

**Week 11: Humanoid Kinematics**
- Core Concepts: Forward/inverse kinematics, degrees of freedom, joint control
- Code Examples: Kinematic calculations for humanoid robots

**Week 12: Bipedal Locomotion & Balance**
- Core Concepts: Gait generation, balance control, zero-moment point
- Practical: Simulating bipedal walking

**Week 13: Vision-Language-Action (VLA) Systems**
- Core Concepts: Multimodal AI, vision-language models, action planning, GPT integration
- Code Examples: Building conversational robotics systems

### Content Template Application

Every week follows this structure:

```markdown
---
id: week-X
sidebar_position: X
title: "Week X: Topic"
description: "..."
moduleId: module-Y
weekNumber: X
tags: [...]
lastReviewed: YYYY-MM-DD
---

# Week X: Topic

## Learning Objectives
<LearningObjectives>
- Objective 1
- Objective 2
- Objective 3
</LearningObjectives>

## Core Concepts

### Concept 1: Name
**Definition**: ...
**Context**: ...
**Related Concepts**: ...

### Concept 2: Name
...

## Practical Explanation
[Hands-on examples, walkthroughs, use cases]

## Visual Aids
[Diagrams with captions and alt text]

## Summary
<WeekSummary nextWeek={{title: "Week X+1: ...", href: "..."}}>
- Key takeaway 1
- Key takeaway 2
</WeekSummary>
```

## Styling & Theming Plan

### Color Palette

**Dark Theme Primary Colors**:
```css
:root {
  --ifm-color-primary: #00d4ff;        /* Cyan accent */
  --ifm-color-primary-dark: #00b8e6;
  --ifm-color-primary-light: #1ae0ff;

  --ifm-background-color: #0a0e1a;     /* Dark navy background */
  --ifm-background-surface-color: #141824;  /* Elevated surfaces */

  --ifm-font-color-base: #e8eaed;      /* Light gray text */
  --ifm-heading-color: #ffffff;        /* White headings */

  --ifm-code-background: #1e2430;      /* Dark code background */
  --ifm-code-color: #00d4ff;           /* Cyan inline code */
}
```

**Semantic Colors**:
- Success: Green (#00cc88)
- Warning: Amber (#ffaa00)
- Danger: Red (#ff4444)
- Info: Cyan (#00d4ff)

### Typography Scale

- **Body Text**: 16px (1rem), line-height 1.65
- **H1 (Page Title)**: 36px (2.25rem), weight 700
- **H2 (Major Section)**: 28px (1.75rem), weight 600
- **H3 (Subsection)**: 22px (1.375rem), weight 600
- **H4 (Minor Section)**: 18px (1.125rem), weight 600
- **Code**: 14.4px (0.9em), Fira Code or JetBrains Mono

**Font Families**:
- Sans-serif: Inter, system-ui, -apple-system, sans-serif
- Monospace: Fira Code, JetBrains Mono, Consolas, monospace

### Spacing and Layout

- **Container Max Width**: 1200px
- **Horizontal Padding**: 1.5rem (24px)
- **Vertical Spacing**: 1.5rem between sections
- **Card Padding**: 1.5rem
- **Border Radius**: 4px for cards, 8px for large surfaces

### Custom CSS Overrides

**File**: `src/css/custom.css`

**Key Overrides**:
1. **Sidebar**:
   - Active link: cyan left border + background tint
   - Hover: subtle cyan glow

2. **Code Blocks**:
   - Dark background (#1e2430)
   - Line numbers with dimmed color
   - Copy button with cyan accent on hover

3. **Cards (Landing Page)**:
   - Background: `--ifm-background-surface-color`
   - Border: 1px solid rgba(255,255,255,0.1)
   - Hover: Lift effect (translateY(-4px)) + cyan box-shadow

4. **Focus Indicators**:
   - Outline: 2px solid cyan
   - Offset: 2px

### MDX Enhancements

**Admonitions** (Docusaurus built-in):
```markdown
:::note
This is a note admonition
:::

:::tip
This is a tip admonition
:::

:::warning
This is a warning admonition
:::
```

**Tables**: Responsive tables with horizontal scroll on mobile

**Code Highlighting**: Prism syntax highlighting with custom dark theme (VS Dark)

## Deployment Plan

### GitHub Pages Strategy

**Hosting**: GitHub Pages (free for public repos, HTTPS by default)

**Deployment Branch**: `gh-pages` (auto-created by GitHub Actions)

**Base URL**: `/ai-native-text-book/` (repository name)

**Build Output**: `/build` directory contains all static files

### GitHub Actions Workflow

**File**: `.github/workflows/deploy.yml`

**Trigger**: Push to `main` branch or manual workflow dispatch

**Workflow Steps**:
1. Checkout repository
2. Setup Node.js 18 with npm cache
3. Install dependencies (`npm ci`)
4. Build site (`npm run build`)
5. Upload build artifact
6. Deploy to GitHub Pages

**Deployment Time**: Typically 2-3 minutes (build ~1-2 min, deploy ~30s)

**Environment**: GitHub Pages environment with HTTPS

### Build Commands

**Development**: `npm start` (hot reload on port 3000)
**Production Build**: `npm run build` (output to `/build`)
**Serve Build**: `npm run serve` (test production build locally)
**Clear Cache**: `npm run clear` (Docusaurus cache reset)

### Optimization Plan

**Image Compression**:
- Pre-build script: Use `sharp` or `imagemin` to optimize images
- Target: WebP 80% quality, max 1200px width
- Estimated savings: 30-50% file size reduction

**Bundle Size Control**:
- Code splitting: Automatic per route
- Tree shaking: Remove unused CSS/JS
- Minification: Production build applies minification
- Webpack Bundle Analyzer: Monitor bundle size

**Performance Budget**:
- Total page size: <500KB
- JavaScript bundle: <200KB
- CSS bundle: <50KB
- Images per page: <10, <100KB each

**Lazy Loading**:
- Images below fold: Lazy load automatically
- Components: Dynamic imports for large components

## Milestones & Phases

### Phase 1: Project Scaffolding (M1)
**Goal**: Initialize Docusaurus project with basic configuration

**Tasks**:
- Create Docusaurus project with TypeScript
- Configure `docusaurus.config.js` (title, URL, baseUrl)
- Set up GitHub repository
- Configure GitHub Pages in repo settings
- Create directory structure (`/docs/module-X/week-Y/`)
- Apply dark theme CSS variables

**Acceptance Criteria**:
- `npm start` runs without errors
- Default site loads at localhost:3000
- Dark theme active by default
- Repository connected to GitHub

**Estimated Effort**: Foundation setup

---

### Phase 2: Landing Page (M2)
**Goal**: Build custom landing page with course overview

**Tasks**:
- Create `src/pages/index.tsx`
- Implement Hero section with title, tagline, CTA
- Add course badges component
- Build HomepageFeatures component (4 module cards)
- Style with CSS modules
- Make responsive (375px to 1920px)

**Acceptance Criteria**:
- Landing page displays all sections
- Module cards link to Week 1 of each module
- Mobile layout stacks vertically
- Passes Lighthouse accessibility audit (95+)

**Estimated Effort**: Custom page development

---

### Phase 3: Navigation + Sidebar (M3)
**Goal**: Configure complete navigation structure

**Tasks**:
- Configure `sidebars.js` with 4 modules + 13 weeks
- Set Module 1 to `collapsed: false`
- Configure navbar items (Course, GitHub link)
- Set up footer links
- Test prev/next navigation

**Acceptance Criteria**:
- All 13 weeks appear in sidebar
- Clicking week navigates correctly
- Breadcrumbs show correct hierarchy
- Prev/next buttons work

**Estimated Effort**: Configuration and testing

---

### Phase 4: Module 1 Content (M4)
**Goal**: Complete all content for Weeks 1-5

**Tasks**:
- Create Week 1-5 MDX files with frontmatter
- Write Learning Objectives for each week
- Write Core Concepts sections
- Add Practical Explanations with examples
- Include code examples for ROS 2 weeks (3-5)
- Add diagrams and images
- Write summaries and next-week links
- Validate frontmatter against schema

**Acceptance Criteria**:
- All 5 weeks have complete content
- All sections present (Objectives, Concepts, Practical, Visual, Summary)
- Code examples have syntax highlighting
- All images have alt text
- No broken links

**Estimated Effort**: Content creation (largest phase)

---

### Phase 5: Module 2 Content (M5)
**Goal**: Complete Weeks 6-7 (Gazebo, Unity)

**Tasks**:
- Create Week 6-7 MDX files
- Write content for simulation topics
- Add URDF/SDF examples
- Include Unity integration code examples
- Add simulation diagrams

**Acceptance Criteria**:
- Weeks 6-7 complete with all sections
- Simulation examples functional
- Links to external resources (Gazebo docs, Unity Robotics Hub)

**Estimated Effort**: Content creation

---

### Phase 6: Module 3 Content (M6)
**Goal**: Complete Weeks 8-10 (Isaac SDK, Isaac Sim, RL)

**Tasks**:
- Create Week 8-10 MDX files
- Write Isaac SDK content
- Include Isaac Sim examples
- Add RL training examples
- Include sim-to-real transfer concepts

**Acceptance Criteria**:
- Weeks 8-10 complete
- Isaac-specific code examples
- RL concepts clearly explained

**Estimated Effort**: Content creation

---

### Phase 7: Module 4 Content (M7)
**Goal**: Complete Weeks 11-13 (Humanoid robotics, VLA)

**Tasks**:
- Create Week 11-13 MDX files
- Write humanoid kinematics content
- Add bipedal locomotion concepts
- Include VLA system explanations
- Add GPT integration examples

**Acceptance Criteria**:
- Weeks 11-13 complete
- Final week (13) includes course summary
- All 13 weeks published

**Estimated Effort**: Content creation

---

### Phase 8: Theming + Polish (M8)
**Goal**: Finalize styling, accessibility, performance

**Tasks**:
- Develop custom React components (LearningObjectives, ConceptCard, etc.)
- Refine CSS for consistency
- Optimize all images to WebP
- Run Lighthouse audits
- Fix accessibility issues
- Test on mobile devices (iPhone, Android)
- Add focus indicators
- Validate WCAG 2.1 AA compliance

**Acceptance Criteria**:
- Lighthouse: Performance 90+, Accessibility 95+, SEO 100
- All images <100KB
- WCAG AA compliance verified
- Mobile responsive on 375px+

**Estimated Effort**: Polish and optimization

---

### Phase 9: Deployment (M9)
**Goal**: Deploy to GitHub Pages with CI/CD

**Tasks**:
- Create `.github/workflows/deploy.yml`
- Configure GitHub Actions workflow
- Test deployment to gh-pages branch
- Verify site loads at GitHub Pages URL
- Set up custom domain (optional)
- Configure HTTPS
- Test all features on production site

**Acceptance Criteria**:
- GitHub Actions workflow passes
- Site accessible at `https://USERNAME.github.io/ai-native-text-book/`
- All pages load correctly
- Search works
- No 404 errors

**Estimated Effort**: CI/CD setup and validation

---

## Risks & Mitigation

### Risk 1: Large Content Volume Slows Build Times
**Impact**: Medium
**Likelihood**: Low
**Current Estimate**: 13 weeks × 2-3 pages = ~30-40 pages (well within Docusaurus capacity)

**Mitigation**:
- Monitor build times in GitHub Actions logs
- If build time approaches 4 minutes, enable incremental builds
- Optimize image processing (run compression pre-commit)
- Use build caching in GitHub Actions

**Trigger**: Build time exceeds 3 minutes
**Action**: Enable incremental builds, investigate slow plugins

---

### Risk 2: GitHub Pages Asset Size Limits
**Impact**: Medium
**Likelihood**: Low (if images optimized)
**GitHub Limit**: 1GB repository size recommended

**Mitigation**:
- Compress all images to WebP at 80% quality
- Keep diagrams under 100KB each
- Use Mermaid for simple diagrams instead of images
- Monitor repository size with `git count-objects -vH`

**Trigger**: Repository exceeds 500MB
**Action**: Audit large files, consider external CDN for assets

---

### Risk 3: Search Performance Degrades with Content Growth
**Impact**: Low
**Likelihood**: Low (local search handles 50-100 pages easily)

**Mitigation**:
- Test search performance after each module addition
- Benchmark with 100+ test queries
- Have Algolia DocSearch application ready
- Optimize search index by excluding non-content pages

**Trigger**: Search latency exceeds 1 second
**Action**: Apply for Algolia DocSearch (free for open source)

---

### Risk 4: Content Accuracy and Maintenance
**Impact**: High
**Likelihood**: Medium (ROS 2, Isaac Sim evolve rapidly)

**Mitigation**:
- Include version numbers in content (e.g., "ROS 2 Humble", "Isaac Sim 2023.1")
- Add `lastReviewed` date to frontmatter
- Link to official documentation for reference
- Plan quarterly content review cycle

**Trigger**: Technology version updates
**Action**: Update relevant weeks, increment `lastReviewed` date

---

### Risk 5: Accessibility Compliance Gaps
**Impact**: Medium
**Likelihood**: Low (Docusaurus has good defaults)

**Mitigation**:
- Run axe DevTools audit on every page
- Test with NVDA/VoiceOver screen readers
- Validate keyboard navigation
- Ensure all images have alt text (enforced in template)
- Maintain 4.5:1 contrast ratio

**Trigger**: Lighthouse accessibility score <95
**Action**: Fix identified issues, re-audit

---

## Acceptance Criteria Alignment

All planning decisions align with specification requirements:

### Functional Requirements Coverage

| Requirement | Plan Alignment |
|-------------|----------------|
| FR-001: Landing page | Phase 2 (M2) delivers custom landing page with title, description, module overview, CTA |
| FR-002: Sidebar navigation | Phase 3 (M3) configures sidebar with 4 modules + 13 weeks |
| FR-003: 13 week pages | Phases 4-7 (M4-M7) create all 13 weeks with required sections |
| FR-004: Syntax highlighting | Prism integration in `docusaurus.config.js` supports Python, C++, YAML, XML, bash |
| FR-005: Search <1s | Local search plugin (research.md) meets performance requirement |
| FR-006: Dark mode | Dark theme CSS (custom.css) sets `defaultMode: 'dark'` |
| FR-007: Responsive 375px+ | CSS media queries and Docusaurus responsive defaults |
| FR-008: GitHub Pages HTTPS | GitHub Actions deployment (Phase 9/M9) to GitHub Pages |
| FR-009: Deep linking | Docusaurus URL anchors support deep linking to sections |
| FR-010: Prev/next navigation | Sidebar order auto-generates prev/next links |
| FR-011: Images with alt text | Content template enforces alt text; validation in Phase 8/M8 |
| FR-012: Progressive enhancement | Static site generation ensures core content without JS |

### Content Requirements Coverage

| Module | Weeks | Plan Coverage |
|--------|-------|---------------|
| CR-001: Module 1 Physical AI & ROS 2 | 1-5 | Phase 4 (M4) – Weeks 1-2 Physical AI, Weeks 3-5 ROS 2 |
| CR-002: Module 2 Simulation | 6-7 | Phase 5 (M5) – Gazebo, Unity |
| CR-003: Module 3 Isaac | 8-10 | Phase 6 (M6) – Isaac SDK, Isaac Sim, RL |
| CR-004: Module 4 Humanoid & VLA | 11-13 | Phase 7 (M7) – Kinematics, locomotion, VLA |

### Design, Performance, Accessibility Requirements

| Category | Requirements | Plan Coverage |
|----------|--------------|---------------|
| Design | DR-001 to DR-006 | Custom CSS (custom.css) applies Panaversity branding, typography, code highlighting |
| Performance | PR-001 to PR-006 | Optimization plan targets FCP <1.5s, <500KB pages, Lighthouse 90+, build <5min |
| Accessibility | AR-001 to AR-005 | WCAG 2.1 AA compliance, keyboard nav, ARIA labels, screen reader support, focus indicators |
| Deployment | DEP-001 to DEP-004 | GitHub Actions workflow (Phase 9/M9) automates deployment with HTTPS |

### Success Criteria Coverage

All 10 success criteria (SC-001 to SC-010) are addressed:

- **SC-001** (3 clicks to Week 1): Landing page CTA → Week 1 (2 clicks)
- **SC-002** (Zero broken links): Link validation in Phase 8
- **SC-003** (Lighthouse 90+): Performance optimization in Phase 8
- **SC-004** (Search <10s): Local search <1s response time
- **SC-005** (375px+ responsive): CSS media queries + Docusaurus defaults
- **SC-006** (Build <5min): GitHub Actions with optimization
- **SC-007** (100% alt text): Enforced in content template + validation
- **SC-008** (HTTPS on GitHub Pages): Automatic with GitHub Pages
- **SC-009** (Prev/next navigation): Sidebar-based auto-generation
- **SC-010** (Syntax highlighting): Prism with Python, C++, YAML, XML, bash

### Constitution Alignment

| Principle | Plan Alignment |
|-----------|----------------|
| Clarity First | Content template enforces clear structure (Objectives → Concepts → Practical → Visual → Summary) |
| Consistency | Frontmatter schema, sidebar pattern, typography scale ensure uniformity |
| Simplicity | Static site, minimal dependencies, no backend complexity |
| Modularity | 4 modules with self-contained weeks, explicit prerequisites in frontmatter |
| Accessibility | WCAG 2.1 AA compliance, keyboard nav, ARIA, screen reader support |

---

## Implementation Sequence

1. **Phase 1** (M1): Scaffolding → Foundation ready for development
2. **Phase 2** (M2): Landing page → Entry point complete
3. **Phase 3** (M3): Navigation → Site structure established
4. **Phase 4** (M4): Module 1 content → First 5 weeks published, validate content template
5. **Phase 5** (M5): Module 2 content → Simulation weeks added
6. **Phase 6** (M6): Module 3 content → Isaac platform weeks added
7. **Phase 7** (M7): Module 4 content → Final weeks + course completion
8. **Phase 8** (M8): Polish → Performance, accessibility, styling refinement
9. **Phase 9** (M9): Deployment → Production release

**Critical Path**: Phases 1-3 must complete before content creation (4-7). Phase 8 can partially overlap with content phases. Phase 9 is final gate.

---

## Architectural Decisions Requiring ADR

Three architecturally significant decisions were made during planning:

### Decision 1: Docusaurus v3 as Static Site Generator
**Rationale**: Mature ecosystem, MDX support, performance optimization, accessibility defaults
**Alternatives**: VitePress (less mature), GitBook (vendor lock-in), custom Next.js (higher complexity)
**Recommendation**: Document in ADR as foundational technology choice

### Decision 2: Local Search Plugin vs Algolia DocSearch
**Rationale**: Zero-cost, no external dependencies, privacy-respecting, sufficient for 50-100 pages
**Alternatives**: Algolia DocSearch (requires application), custom search (high effort)
**Recommendation**: Document in ADR with migration path to Algolia if needed

### Decision 3: GitHub Pages for Hosting
**Rationale**: Free for public repos, automatic HTTPS, GitHub Actions integration, 1GB capacity sufficient
**Alternatives**: Netlify (extra dependency), Vercel (unnecessary features), custom hosting (cost)
**Recommendation**: Document in ADR as deployment architecture

**Suggested Command**:
```
📋 Architectural decisions detected:
1. Static site generator selection (Docusaurus v3)
2. Search implementation strategy (local search with Algolia upgrade path)
3. Hosting and deployment platform (GitHub Pages)

Document reasoning and tradeoffs? Run:
/sp.adr static-site-architecture
```

---

## Summary

This plan provides a complete implementation roadmap for the Physical AI & Humanoid Robotics textbook site:

- **Architecture**: Docusaurus v3 static site with docs-only mode, MDX content, local search, GitHub Pages deployment
- **Structure**: 4 modules, 13 weeks, ~30-40 pages with consistent frontmatter and content template
- **Styling**: Dark theme with Panaversity cyan branding, WCAG 2.1 AA compliant, responsive 375px+
- **Deployment**: GitHub Actions CI/CD to GitHub Pages with automatic HTTPS
- **Phases**: 9 milestones from scaffolding through deployment
- **Risks**: Mitigated through monitoring, optimization, and contingency plans

All decisions align with the project constitution (Clarity, Consistency, Simplicity, Modularity, Accessibility) and satisfy all specification requirements (functional, content, design, performance, accessibility, deployment).

**Next Command**: `/sp.tasks` to generate actionable, testable tasks from this plan.
