# Tasks: Docusaurus Textbook Site

**Input**: Design documents from `/specs/001-docusaurus-textbook-site/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Organization**: Tasks are grouped by user story (US1-US6) to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Per plan.md, this is a web application with Docusaurus structure:
- Root: `ai-native-text-book/`
- Docs: `docs/`
- Source: `src/`
- Static: `static/`
- Config: `docusaurus.config.js`, `sidebars.js`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

**Priority**: P0 (Critical path - must complete first)

- [X] T001 Initialize Docusaurus project with TypeScript at repository root using `npx create-docusaurus@latest . classic --typescript`
- [X] T002 [P] Install additional dependencies: `@easyops-cn/docusaurus-search-local`, `@docusaurus/theme-mermaid`
- [X] T003 [P] Create directory structure: `docs/module-{1-4}/week-{1-13}/`, `static/img/modules/`, `static/img/diagrams/`, `src/components/`
- [X] T004 [P] Initialize Git repository and create `.gitignore` for `node_modules/`, `build/`, `.docusaurus/`
- [X] T005 [P] Create `README.md` with project overview, setup instructions, and contribution guidelines

**Acceptance Criteria**:
- `npm start` runs without errors
- Default Docusaurus site loads at localhost:3000
- All directory structure present
- Git repository initialized with initial commit

**Validation**: Run `npm start` and verify browser opens with default Docusaurus page

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core configuration that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

**Priority**: P0 (Critical path)

- [X] T006 Configure `docusaurus.config.js` with site title, tagline, URL, baseUrl, organizationName, projectName per plan.md Architecture Overview section
- [X] T007 [P] Configure dark theme in `docusaurus.config.js`: set `colorMode.defaultMode: 'dark'`, configure Prism themes
- [X] T008 [P] Add search plugin configuration in `docusaurus.config.js`: install and configure `@easyops-cn/docusaurus-search-local` with `hashed: true`, `language: ['en']`
- [X] T009 [P] Enable docs-only mode in `docusaurus.config.js`: set `docs.routeBasePath: '/'`, `blog: false`
- [X] T010 [P] Configure additional Prism languages in `docusaurus.config.js`: `additionalLanguages: ['python', 'cpp', 'yaml', 'xml', 'bash']`
- [X] T011 Create `src/css/custom.css` with Panaversity dark theme CSS variables per plan.md Styling & Theming Plan
- [X] T012 Apply dark theme color palette in `src/css/custom.css`: primary cyan (#00d4ff), background (#0a0e1a), code background (#1e2430)
- [X] T013 [P] Configure typography scale in `src/css/custom.css`: body 16px, H1 36px, H2 28px, H3 22px, Inter font family
- [X] T014 [P] Add focus indicators and accessibility styles in `src/css/custom.css`: 2px solid cyan outline, WCAG compliant contrast
- [X] T015 Configure `sidebars.js` with complete 4-module structure per plan.md Navigation Plan section

**Acceptance Criteria**:
- Dark theme active by default
- Site title shows "Physical AI & Humanoid Robotics Textbook"
- CSS variables applied (cyan primary color visible)
- Typography scale correct (inspect with browser DevTools)
- Sidebar structure configured (4 modules visible in config)
- Build completes without errors: `npm run build`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

**Validation**: Run `npm start`, verify dark theme, inspect CSS variables in DevTools, check sidebar.js exports correctly

---

## Phase 3: User Story 1 - Access Course Landing Page (Priority: P1) 🎯 MVP

**Goal**: Deliver a professional landing page that introduces the course and enables navigation to content

**Independent Test**: Navigate to deployed site URL and verify landing page displays correctly with navigation to content

### Implementation for User Story 1

- [X] T016 [P] [US1] Create `src/pages/index.tsx` with custom landing page replacing default homepage
- [X] T017 [US1] Implement Hero section in `src/pages/index.tsx`: title, subtitle, cover image placeholder, "Start Reading" CTA button
- [X] T018 [P] [US1] Create `src/pages/index.module.css` with landing page styles: hero section, responsive grid, card styling
- [X] T019 [P] [US1] Create `src/components/HomepageFeatures/index.tsx` component for module overview cards
- [X] T020 [US1] Implement 4 module cards in `HomepageFeatures`: Module 1-4 with title, description, icon, week range, href
- [X] T021 [P] [US1] Create `src/components/HomepageFeatures/styles.module.css` with card styling, hover effects, responsive grid
- [X] T022 [US1] Add course badges section to `src/pages/index.tsx`: "13 Weeks", "4 Modules", "Open Source", "CC-BY-4.0"
- [X] T023 [P] [US1] Add "What You Will Learn" summary section to `src/pages/index.tsx` with 4-5 bullet points
- [X] T024 [US1] Implement responsive layout in `index.module.css`: 2×2 grid desktop, single column mobile (375px+)
- [X] T025 [US1] Configure "Start Reading" CTA button to link to Week 1: `/module-1-physical-ai/week-1/`
- [X] T026 [P] [US1] Add placeholder logo to `static/img/logo.svg` (can be simple text-based SVG)
- [X] T027 [P] [US1] Add placeholder favicon to `static/img/favicon.ico`

**Acceptance Criteria (US1)**:
- Landing page displays at site root (`/`)
- Hero section shows title, subtitle, CTA button
- 4 module cards display in grid layout
- CTA button navigates to Week 1 (even if Week 1 is placeholder)
- Mobile layout (375px width): elements stack vertically, no horizontal scroll
- Desktop layout (1200px+ width): 2×2 grid for module cards
- No console errors, build succeeds

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

**Validation**:
1. Run `npm start`, navigate to `http://localhost:3000`
2. Verify landing page displays (not default Docusaurus page)
3. Click "Start Reading" button, verify navigation attempt (may 404 if Week 1 not created yet)
4. Resize browser to 375px width, verify vertical stack layout
5. Run `npm run build`, verify build succeeds

---

## Phase 4: User Story 2 - Navigate Course Structure (Priority: P1) 🎯 MVP

**Goal**: Deliver sidebar navigation showing all 13 weeks organized by 4 modules

**Independent Test**: Load any page and verify sidebar displays all weeks grouped by 4 modules with working links

### Implementation for User Story 2

- [X] T028 [US2] Complete `sidebars.js` configuration: define all 13 weeks under 4 module categories per data-model.md week mapping
- [X] T029 [P] [US2] Configure Module 1 category in `sidebars.js`: "Introduction to Physical AI & ROS 2", weeks 1-5, `collapsed: false`
- [X] T030 [P] [US2] Configure Module 2 category in `sidebars.js`: "Robot Simulation", weeks 6-7, `collapsed: true`
- [X] T031 [P] [US2] Configure Module 3 category in `sidebars.js`: "NVIDIA Isaac Platform", weeks 8-10, `collapsed: true`
- [X] T032 [P] [US2] Configure Module 4 category in `sidebars.js`: "Humanoid Robotics & VLA", weeks 11-13, `collapsed: true`
- [X] T033 [US2] Add "Course Overview" doc link at top of sidebar pointing to `intro` doc
- [X] T034 [P] [US2] Create placeholder `docs/intro.md` with course overview content per quickstart.md Step 7
- [X] T035 [P] [US2] Create placeholder week files for all 13 weeks: `docs/module-X/week-Y/index.md` with minimal frontmatter
- [X] T036 [US2] Customize sidebar styling in `src/css/custom.css`: active link highlighting, hover effects, module category styling
- [X] T037 [P] [US2] Configure navbar in `docusaurus.config.js`: "Course" link (left), GitHub link (right)
- [X] T038 [P] [US2] Configure footer in `docusaurus.config.js`: links to GitHub, Panaversity, copyright notice "© 2025 Panaversity. CC-BY-4.0"
- [X] T039 [US2] Test mobile sidebar: verify hamburger menu appears <768px, sidebar opens/closes correctly

**Acceptance Criteria (US2)**:
- Sidebar displays 4 module sections
- Each module shows correct week range (Module 1: Weeks 1-5, Module 2: Weeks 6-7, etc.)
- Module 1 expanded by default, Modules 2-4 collapsed
- Clicking any week link navigates to that week's page
- Mobile (375px): hamburger menu icon appears, sidebar opens on tap
- Navbar shows "Course" and "GitHub" links
- Footer displays copyright and links

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

**Validation**:
1. Run `npm start`, verify sidebar visible on left
2. Expand/collapse each module, verify behavior
3. Click Week 1 link, verify navigation to `/module-1-physical-ai/week-1/`
4. Resize to 375px, verify hamburger menu appears
5. Tap hamburger, verify sidebar opens
6. Verify footer links render correctly

---

## Phase 5: User Story 3 - Read Week 1 Content (Priority: P1) 🎯 MVP

**Goal**: Deliver complete Week 1 content validating the content template for all future weeks

**Independent Test**: Navigate to Week 1 and verify it contains all required sections in proper format

### Implementation for User Story 3

- [X] T040 [P] [US3] Create `src/components/LearningObjectives/index.tsx` component with styled list wrapper
- [X] T041 [P] [US3] Create `src/components/LearningObjectives/styles.module.css` with objective list styling (icon, indentation)
- [X] T042 [US3] Write Week 1 frontmatter in `docs/module-1-physical-ai/week-1/index.md` per week-frontmatter.schema.json: id, title, description, moduleId, weekNumber, tags
- [X] T043 [US3] Write Week 1 Learning Objectives section using `<LearningObjectives>` component: 3-5 objectives per plan.md content outline
- [X] T044 [US3] Write Week 1 Core Concepts section: Embodied Intelligence, Sensor-Actuator Loops, Physical AI vs Traditional AI (definitions + context)
- [X] T045 [US3] Write Week 1 Practical Explanation section: case studies, hands-on examples per plan.md Week 1 outline
- [X] T046 [P] [US3] Add Week 1 diagrams to `static/img/modules/module-1/`: control loop diagram (Mermaid or SVG), application examples
- [X] T047 [US3] Embed diagrams in Week 1 Visual Aids section with proper alt text and captions
- [X] T048 [P] [US3] Create `src/components/WeekSummary/index.tsx` component for summary section with next-week link
- [X] T049 [P] [US3] Create `src/components/WeekSummary/styles.module.css` with summary box styling
- [X] T050 [US3] Write Week 1 Summary section using `<WeekSummary>` component: key takeaways, link to Week 2
- [X] T051 [US3] Add code examples to Week 1 (if applicable): Python snippets with syntax highlighting
- [X] T052 [P] [US3] Validate Week 1 frontmatter against `week-frontmatter.schema.json`: run JSON schema validation
- [X] T053 [US3] Test prev/next navigation from Week 1: verify "Next: Week 2" link works
- [X] T054 [P] [US3] Verify all Week 1 images have alt text: inspect with accessibility checker

**Acceptance Criteria (US3)**:
- Week 1 page displays at `/module-1-physical-ai/week-1/`
- All 5 sections present: Learning Objectives, Core Concepts, Practical Explanation, Visual Aids, Summary
- Learning Objectives: 3-5 items displayed with custom styling
- Core Concepts: 2-4 subsections with definitions and context
- Code examples: syntax highlighting active (if code present)
- Diagrams: render correctly with captions and alt text
- Summary: displays with "Next: Week 2" link
- Mobile responsive: readable on 375px width
- No broken links, no console errors

**Checkpoint**: At this point, User Story 3 should be fully functional and Week 1 serves as template for all other weeks

**Validation**:
1. Navigate to Week 1, verify all sections render
2. Inspect `<LearningObjectives>` component rendering
3. Verify diagrams load and have alt text
4. Click "Next: Week 2" link (may 404 if Week 2 incomplete)
5. Resize to 375px, verify content readable
6. Run accessibility audit (Lighthouse or axe DevTools), check for alt text

---

## Phase 6: User Story 4 - Search Course Content (Priority: P2)

**Goal**: Deliver functional search across all course content with sub-1-second response time

**Independent Test**: Enter search queries and verify results are relevant and comprehensive

### Implementation for User Story 4

- [ ] T055 [US4] Verify search plugin installed and configured: check `@easyops-cn/docusaurus-search-local` in `docusaurus.config.js` plugins array
- [ ] T056 [US4] Build site to generate search index: run `npm run build`, verify `search-index.json` created in `build/`
- [ ] T057 [US4] Test search functionality: run `npm run serve`, type query in search box, verify results appear
- [ ] T058 [P] [US4] Customize search UI styling in `src/css/custom.css`: search box styling, results dropdown, highlighting
- [ ] T059 [US4] Test search with multiple queries: "ROS 2 nodes", "embodied intelligence", "Gazebo", "Isaac Sim" - verify relevant results
- [ ] T060 [P] [US4] Test search performance: measure time from query input to results display, verify <1 second
- [ ] T061 [US4] Test search result relevance: verify results show context snippets, highlighted search terms
- [ ] T062 [P] [US4] Test search navigation: click search result, verify navigation to correct page and section (if anchor supported)

**Acceptance Criteria (US4)**:
- Search box visible in navbar
- Typing query shows suggestions within 1 second
- Search for "ROS 2 nodes" returns all pages mentioning ROS 2 nodes
- Results show context snippets with search terms highlighted
- Clicking result navigates to correct page
- Search works on production build (`npm run serve`)

**Checkpoint**: At this point, User Stories 1-4 should all work independently

**Validation**:
1. Run `npm run build && npm run serve`
2. Enter search query "ROS 2"
3. Verify results appear <1 second
4. Check results for context snippets
5. Click result, verify navigation
6. Test edge cases: empty query, no results, special characters

---

## Phase 7: User Story 5 - Access Complete Course Content (Weeks 2-13) (Priority: P2)

**Goal**: Deliver all 13 weeks of content following Week 1 template structure

**Independent Test**: Verify each week (2-13) exists, follows template structure, and covers designated topics

### Implementation for User Story 5 - Module 1 (Weeks 2-5)

- [ ] T063 [P] [US5] Write Week 2 content in `docs/module-1-physical-ai/week-2/index.md`: Embodied Intelligence Deep Dive per plan.md outline
- [ ] T064 [P] [US5] Write Week 3 content in `docs/module-1-physical-ai/week-3/index.md`: ROS 2 Basics (architecture, nodes, packages)
- [ ] T065 [P] [US5] Write Week 4 content in `docs/module-1-physical-ai/week-4/index.md`: ROS 2 Communication (topics, services, actions)
- [ ] T066 [P] [US5] Write Week 5 content in `docs/module-1-physical-ai/week-5/index.md`: ROS 2 Package Development (structure, launch files)
- [ ] T067 [P] [US5] Add ROS 2 code examples to Weeks 3-5: Python publisher/subscriber, service client/server, package structure
- [ ] T068 [P] [US5] Add diagrams for Weeks 2-5 to `static/img/modules/module-1/`: ROS 2 architecture diagram, communication patterns

### Implementation for User Story 5 - Module 2 (Weeks 6-7)

- [ ] T069 [P] [US5] Write Week 6 content in `docs/module-2-simulation/week-6/index.md`: Gazebo Simulation (setup, URDF/SDF, physics)
- [ ] T070 [P] [US5] Write Week 7 content in `docs/module-2-simulation/week-7/index.md`: Unity Robotics (integration, ROS-Unity communication)
- [ ] T071 [P] [US5] Add simulation code examples to Weeks 6-7: URDF robot models, Gazebo launch files, Unity scripts
- [ ] T072 [P] [US5] Add diagrams for Weeks 6-7 to `static/img/modules/module-2/`: Gazebo architecture, Unity ROS integration

### Implementation for User Story 5 - Module 3 (Weeks 8-10)

- [ ] T073 [P] [US5] Write Week 8 content in `docs/module-3-isaac/week-8/index.md`: Isaac SDK Introduction (architecture, modules, codelets)
- [ ] T074 [P] [US5] Write Week 9 content in `docs/module-3-isaac/week-9/index.md`: Isaac Sim & Perception (sensor simulation, perception algorithms)
- [ ] T075 [P] [US5] Write Week 10 content in `docs/module-3-isaac/week-10/index.md`: Reinforcement Learning & Sim-to-Real Transfer
- [ ] T076 [P] [US5] Add Isaac code examples to Weeks 8-10: Isaac SDK applications, perception pipelines, RL training scripts
- [ ] T077 [P] [US5] Add diagrams for Weeks 8-10 to `static/img/modules/module-3/`: Isaac architecture, sim-to-real pipeline

### Implementation for User Story 5 - Module 4 (Weeks 11-13)

- [ ] T078 [P] [US5] Write Week 11 content in `docs/module-4-humanoid-vla/week-11/index.md`: Humanoid Kinematics (forward/inverse, DOF)
- [ ] T079 [P] [US5] Write Week 12 content in `docs/module-4-humanoid-vla/week-12/index.md`: Bipedal Locomotion & Balance (gait, ZMP, control)
- [ ] T080 [P] [US5] Write Week 13 content in `docs/module-4-humanoid-vla/week-13/index.md`: Vision-Language-Action Systems (VLA, multimodal AI, GPT)
- [ ] T081 [P] [US5] Add humanoid/VLA code examples to Weeks 11-13: kinematics calculations, locomotion control, VLA integration
- [ ] T082 [P] [US5] Add diagrams for Weeks 11-13 to `static/img/modules/module-4/`: humanoid kinematic chain, VLA system architecture

### Content Validation for User Story 5

- [ ] T083 [US5] Validate all week frontmatter (Weeks 2-13) against schema: run JSON schema validation for each week
- [ ] T084 [P] [US5] Verify all weeks have 5 required sections: Learning Objectives, Core Concepts, Practical Explanation, Visual Aids, Summary
- [ ] T085 [P] [US5] Verify all images have alt text: automated scan with accessibility checker
- [ ] T086 [US5] Test navigation through all 13 weeks: verify prev/next links work sequentially
- [ ] T087 [P] [US5] Verify all internal links resolve correctly: run broken link checker
- [ ] T088 [US5] Verify all code examples have syntax highlighting: inspect with browser DevTools

**Acceptance Criteria (US5)**:
- All 13 weeks exist and are navigable
- Each week (2-13) has all 5 sections (Objectives, Concepts, Practical, Visual, Summary)
- Module 2 content (Weeks 6-7): Gazebo and Unity topics covered with examples
- Module 3 content (Weeks 8-10): Isaac SDK, Isaac Sim, RL topics covered with examples
- Module 4 content (Weeks 11-13): Humanoid kinematics, locomotion, VLA topics covered
- Week 13 includes course summary and final takeaways
- All code examples display with syntax highlighting
- All images load and have alt text
- No broken links across all content
- Prev/next navigation works for all weeks

**Checkpoint**: All user stories (1-5) should now be independently functional

**Validation**:
1. Navigate through all 13 weeks using next links
2. Spot-check 3-4 weeks for all 5 sections
3. Verify Module 2 weeks cover Gazebo/Unity
4. Verify Module 3 weeks cover Isaac platform
5. Verify Module 4 weeks cover humanoid robotics and VLA
6. Run broken link checker: `npm run build` (Docusaurus checks links)
7. Run accessibility audit on 3-4 random weeks

---

## Phase 8: User Story 6 - View Site on Mobile Devices (Priority: P2)

**Goal**: Deliver fully responsive site readable and navigable on mobile devices (375px+)

**Independent Test**: Load site on devices with screens 375px and above, verify all features work

### Implementation for User Story 6

- [ ] T089 [US6] Test landing page on mobile (375px): verify hero, module cards, CTA button all readable, no horizontal scroll
- [ ] T090 [P] [US6] Test sidebar navigation on mobile: verify hamburger menu, sidebar opens/closes, all links tappable
- [ ] T091 [P] [US6] Test Week 1 content on mobile: verify text readable without zooming, images fit within screen
- [ ] T092 [US6] Test code blocks on mobile: verify horizontal scroll within code blocks, no page layout breaking
- [ ] T093 [P] [US6] Add responsive image sizing in `src/css/custom.css`: `max-width: 100%`, `height: auto`
- [ ] T094 [P] [US6] Test tables on mobile: verify tables are horizontally scrollable if wide
- [ ] T095 [US6] Test navigation on mobile: verify prev/next buttons are tappable, sufficient touch target size (44px+)
- [ ] T096 [P] [US6] Test search on mobile: verify search box accessible, results readable
- [ ] T097 [US6] Use browser DevTools responsive mode: test at 375px, 768px, 1024px, 1920px widths
- [ ] T098 [P] [US6] Test on real devices (if available): iPhone, Android phone, iPad - verify functionality
- [ ] T099 [US6] Verify touch interactions: tap hamburger menu, tap links, tap buttons - all work smoothly

**Acceptance Criteria (US6)**:
- Site loads and renders correctly on 375px width (iPhone SE size)
- Landing page: elements stack vertically, no horizontal scroll
- Sidebar: hamburger menu appears, taps open/close sidebar
- Content pages: text readable, images fit screen, code scrolls horizontally within blocks
- Navigation: prev/next buttons have sufficient touch target size
- Search: search box accessible and usable on mobile
- Tested on multiple viewport sizes: 375px, 768px, 1024px, 1920px

**Validation**:
1. Open Chrome DevTools, set responsive mode to 375px
2. Navigate to landing page, verify vertical stack layout
3. Tap hamburger menu, verify sidebar opens
4. Navigate to Week 1, verify text readable
5. Check code blocks for horizontal scroll
6. Test on iPhone or Android device if available
7. Verify Lighthouse mobile score 90+

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements affecting multiple user stories and final deployment

**Priority**: P2-P3 (Polish and optimization)

### Theming & Styling Polish

- [ ] T100 [P] [P2] Refine card hover effects in `src/css/custom.css`: transform, box-shadow, transition timing
- [ ] T101 [P] [P2] Add sidebar active link styling in `src/css/custom.css`: cyan left border, background highlight
- [ ] T102 [P] [P2] Customize code block styling in `src/css/custom.css`: line numbers, copy button styling
- [ ] T103 [P] [P3] Add custom scrollbar styling in `src/css/custom.css` for dark theme consistency
- [ ] T104 [P] [P3] Create `src/components/ConceptCard/index.tsx` component for styled concept boxes (optional enhancement)
- [ ] T105 [P] [P3] Create `src/components/CodeExample/index.tsx` component with enhanced code display (optional)

### Image & Asset Optimization

- [ ] T106 [P] [P1] Optimize all images to WebP format: convert PNG/JPEG in `static/img/` to WebP at 80% quality
- [ ] T107 [P] [P2] Compress all images: ensure each image <100KB, diagrams <50KB
- [ ] T108 [P] [P2] Add Mermaid diagrams where applicable: convert simple diagrams from static images to Mermaid syntax in MDX
- [ ] T109 [P] [P3] Create placeholder images for missing diagrams: generate or source diagram images for all weeks

### Performance Optimization

- [ ] T110 [P1] Run production build and measure size: `npm run build`, check `build/` directory size <500MB
- [ ] T111 [P1] Test production build performance: `npm run serve`, measure FCP <1.5s, LCP <2.5s using Lighthouse
- [ ] T112 [P2] Optimize bundle size: analyze with `webpack-bundle-analyzer` (if available), identify large dependencies
- [ ] T113 [P3] Enable service worker for offline support in `docusaurus.config.js` (optional)

### Accessibility Validation

- [ ] T114 [P1] Run Lighthouse accessibility audit: verify score 95+ on landing page, Week 1, Week 13
- [ ] T115 [P] [P1] Verify keyboard navigation: tab through all interactive elements, verify focus indicators visible
- [ ] T116 [P] [P1] Verify color contrast: use WebAIM contrast checker, ensure all text meets WCAG AA (4.5:1 minimum)
- [ ] T117 [P1] Run axe DevTools audit: scan landing page and 3-4 content pages, fix critical issues
- [ ] T118 [P] [P2] Test with screen reader (NVDA or VoiceOver): verify heading hierarchy, ARIA labels, alt text announcements
- [ ] T119 [P] [P2] Verify semantic HTML: inspect heading hierarchy (h1 → h2 → h3, no skipping), landmark elements

### Content Quality & Validation

- [ ] T120 [P1] Run broken link checker: use Docusaurus build process or external tool, verify zero broken links
- [ ] T121 [P] [P1] Validate all frontmatter: run JSON schema validation against `week-frontmatter.schema.json` for all weeks
- [ ] T122 [P] [P2] Spell check all content: run spell checker on all MDX files, fix typos
- [ ] T123 [P2] Review content for constitution alignment: verify Clarity, Consistency, Simplicity in sample weeks
- [ ] T124 [P] [P3] Add last-reviewed dates to all weeks: populate `lastReviewed` field in frontmatter

### Documentation

- [ ] T125 [P] [P1] Update `README.md` with setup instructions, deployment guide, contribution guidelines
- [ ] T126 [P] [P2] Create `CONTRIBUTING.md` with content contribution guidelines, frontmatter schema reference
- [ ] T127 [P] [P3] Create `docs/changelog.md` with version history and content update log

---

## Phase 10: GitHub Pages Deployment (Priority: P0 - Critical)

**Purpose**: Deploy site to production on GitHub Pages with CI/CD

**Priority**: P0 (Critical path to launch)

### GitHub Setup

- [ ] T128 [P1] Create GitHub repository: initialize remote repository `ai-native-text-book` on GitHub
- [ ] T129 [P1] Push local code to GitHub: commit all changes, push to `main` branch
- [ ] T130 [P] [P1] Enable GitHub Pages in repository settings: set source to "GitHub Actions"
- [ ] T131 [P] [P1] Configure repository settings: verify GitHub Pages enabled, HTTPS enforced

### GitHub Actions Workflow

- [ ] T132 [P1] Create `.github/workflows/deploy.yml` per plan.md Deployment Plan section
- [ ] T133 [P1] Configure workflow triggers: `push` to `main` branch, `workflow_dispatch` for manual runs
- [ ] T134 [P1] Configure build job in workflow: checkout code, setup Node.js 18, install dependencies, run build
- [ ] T135 [P1] Configure deploy job in workflow: upload build artifact, deploy to GitHub Pages using `actions/deploy-pages@v4`
- [ ] T136 [P] [P1] Set workflow permissions: `contents: read`, `pages: write`, `id-token: write`
- [ ] T137 [P1] Commit and push workflow file: add `.github/workflows/deploy.yml` to repository
- [ ] T138 [P1] Monitor first deployment: verify workflow runs successfully, check Actions tab for errors

### Deployment Validation

- [ ] T139 [P1] Verify site is live: navigate to `https://USERNAME.github.io/ai-native-text-book/`, verify landing page loads
- [ ] T140 [P] [P1] Test all navigation on production: verify sidebar, prev/next links, search all work
- [ ] T141 [P] [P1] Test landing page on production: verify module cards, CTA button function correctly
- [ ] T142 [P1] Verify HTTPS works: check for green lock icon, certificate valid
- [ ] T143 [P] [P1] Test search on production: verify search index generated and search works
- [ ] T144 [P] [P1] Run Lighthouse audit on production URL: verify Performance 90+, Accessibility 95+, SEO 100

### Production Testing

- [ ] T145 [P] [P1] Test on multiple browsers: Chrome, Firefox, Safari, Edge - verify site works on all
- [ ] T146 [P] [P1] Test on mobile devices: verify site works on iPhone, Android phone
- [ ] T147 [P1] Verify all 13 weeks accessible on production: spot-check Week 1, Week 7, Week 13
- [ ] T148 [P] [P2] Check page load times: verify FCP <1.5s, LCP <2.5s on production with Lighthouse
- [ ] T149 [P2] Monitor deployment workflow: verify future pushes to `main` trigger automatic deployments

**Acceptance Criteria (Deployment)**:
- Site deployed to `https://USERNAME.github.io/ai-native-text-book/`
- All pages load correctly on production
- HTTPS active with valid certificate
- GitHub Actions workflow passes successfully
- Lighthouse scores: Performance 90+, Accessibility 95+, SEO 100
- Search works on production
- Mobile responsive on production
- No 404 errors or broken links

**Validation**:
1. Navigate to production URL
2. Test all user stories on production (US1-US6)
3. Run Lighthouse audit from production URL
4. Test on mobile device
5. Verify GitHub Actions workflow success in Actions tab

---

## Dependencies & Execution Order

### Phase Dependencies

1. **Setup (Phase 1)**: No dependencies - start immediately
2. **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
3. **User Story 1 (Phase 3)**: Depends on Foundational
4. **User Story 2 (Phase 4)**: Depends on Foundational
5. **User Story 3 (Phase 5)**: Depends on US1 (landing page), US2 (navigation structure)
6. **User Story 4 (Phase 6)**: Depends on US3 (Week 1 content for search testing)
7. **User Story 5 (Phase 7)**: Depends on US3 (Week 1 as template)
8. **User Story 6 (Phase 8)**: Depends on US1, US3, US5 (content to test on mobile)
9. **Polish (Phase 9)**: Depends on all user stories
10. **Deployment (Phase 10)**: Can run in parallel with user stories, but validate after Polish

### Critical Path

Setup → Foundational → US1 + US2 → US3 → US5 → Polish → Deployment

**Minimum Viable Product (MVP)**:
- Setup + Foundational + US1 + US2 + US3 (Landing page, Navigation, Week 1 content)

**Full Feature Set**:
- All phases (US1-US6 + Polish + Deployment)

### Parallel Opportunities

**Setup Phase**: T002, T003, T004, T005 can run in parallel

**Foundational Phase**: T007, T008, T009, T010, T013, T014 can run in parallel

**User Story 1**: T018, T019, T021, T023, T026, T027 can run in parallel

**User Story 2**: T029, T030, T031, T032, T034, T035, T037, T038 can run in parallel

**User Story 3**: T040, T041, T046, T048, T049, T052, T054 can run in parallel

**User Story 5**: All week content tasks (T063-T082) can run in parallel if multiple developers

**Polish Phase**: Most tasks can run in parallel (T100-T127)

**Deployment**: T130, T131, T136 can run in parallel

---

## Implementation Strategy

### MVP First (User Stories 1-3)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T015)
3. Complete Phase 3: User Story 1 - Landing Page (T016-T027)
4. Complete Phase 4: User Story 2 - Navigation (T028-T039)
5. Complete Phase 5: User Story 3 - Week 1 Content (T040-T054)
6. **STOP and VALIDATE**: Test landing page, navigation, Week 1 independently
7. Deploy MVP to GitHub Pages (Phase 10)

### Incremental Delivery

1. MVP (US1-US3) → Deploy and validate
2. Add US4 (Search) → Deploy and validate
3. Add US5 (Complete content) → Deploy and validate
4. Add US6 (Mobile) → Deploy and validate
5. Polish (Phase 9) → Final deployment

### Parallel Team Strategy

With 2-3 developers:

1. **Developer A**: Setup + Foundational (blocking tasks)
2. Once Foundational complete:
   - **Developer A**: US1 (Landing Page) + US2 (Navigation)
   - **Developer B**: US3 (Week 1 Content)
   - **Developer C**: US4 (Search configuration)
3. After US3 template validated:
   - **Developer A**: US5 Module 1-2 content (Weeks 2-7)
   - **Developer B**: US5 Module 3-4 content (Weeks 8-13)
   - **Developer C**: US6 (Mobile testing) + Deployment setup
4. All developers: Polish phase (parallel tasks)

---

## Task Estimates

**Phase 1: Setup**: 2-3 hours
**Phase 2: Foundational**: 4-6 hours
**Phase 3: US1 (Landing Page)**: 4-6 hours
**Phase 4: US2 (Navigation)**: 3-4 hours
**Phase 5: US3 (Week 1 Content)**: 8-12 hours (includes component development)
**Phase 6: US4 (Search)**: 2-3 hours
**Phase 7: US5 (Weeks 2-13 Content)**: 40-60 hours (3-5 hours per week × 12 weeks)
**Phase 8: US6 (Mobile)**: 3-4 hours
**Phase 9: Polish**: 8-12 hours
**Phase 10: Deployment**: 2-3 hours

**Total Estimate**: 76-113 hours

**MVP Estimate** (US1-US3 + Deployment): 23-34 hours

---

## Constitution Alignment Validation

### Clarity First (Principle I)

**Tasks enforcing Clarity**:
- T042-T050: Week 1 content follows clear structure (Objectives → Concepts → Practical → Visual → Summary)
- T052: Frontmatter validation ensures descriptions present
- T084: Verify all weeks have 5 required sections

**Validation**: Review Week 1 (T040-T054) to ensure clear structure implemented

### Consistency (Principle II)

**Tasks enforcing Consistency**:
- T028-T032: Sidebar uses consistent labeling ("Week X: Topic")
- T083: Validate all frontmatter against schema
- T013: Typography scale applied uniformly
- T121: JSON schema validation for all weeks

**Validation**: Inspect multiple weeks for consistent formatting and structure

### Simplicity (Principle III)

**Tasks enforcing Simplicity**:
- T002: Minimal dependencies (Docusaurus + search plugin only)
- T009: Docs-only mode (no blog, showcase)
- T001: Static site generation (no backend/database)

**Validation**: Review `package.json` dependencies, verify no unnecessary features added

### Modularity (Principle IV)

**Tasks enforcing Modularity**:
- T035: Each week is self-contained MDX file
- T028-T032: 4 module categories with clear boundaries
- T042: Frontmatter includes prerequisites field

**Validation**: Verify each week can be read independently, prerequisites documented

### Accessibility (Principle V)

**Tasks enforcing Accessibility**:
- T014: Focus indicators, WCAG contrast in CSS
- T054, T085: All images have alt text
- T114-T119: Accessibility audits (Lighthouse, axe, screen reader, keyboard nav)
- T116: Color contrast verification

**Validation**: Run Lighthouse accessibility audit, verify 95+ score

---

## Notes

- **[P]** tasks = different files, no dependencies, can run in parallel
- **[Story]** label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group of tasks
- Stop at any checkpoint to validate story independently
- **Priority levels**: P0 = Critical path, P1 = Core functionality, P2 = Enhanced features, P3 = Nice-to-have
- Follow quickstart.md for detailed setup steps (T001-T015)
- Reference plan.md for architecture decisions and detailed specifications
- Reference data-model.md for content structure and entity definitions
- Reference contracts/ for configuration schemas

---

## Success Metrics

**After MVP (US1-US3 + Deployment)**:
- Site deployed and accessible via HTTPS
- Landing page functional with navigation to Week 1
- Sidebar shows all 13 weeks (even if placeholder)
- Week 1 complete with all 5 sections
- Lighthouse: Performance 90+, Accessibility 95+

**After All User Stories (US1-US6)**:
- All 13 weeks published with complete content
- Search functional with <1s response time
- Mobile responsive on 375px+ devices
- Zero broken links
- All acceptance criteria (SC-001 to SC-010) met per spec.md

**After Polish & Deployment (Phase 9-10)**:
- Production site live on GitHub Pages
- All images optimized (WebP, <100KB)
- Accessibility audit: WCAG 2.1 AA compliant
- Performance budget met: FCP <1.5s, page size <500KB
- GitHub Actions workflow automated and passing
