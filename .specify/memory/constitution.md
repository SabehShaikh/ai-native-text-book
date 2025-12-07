<!--
Sync Impact Report:
- Version change: [initial template] → 1.0.0
- Modified principles: All placeholders filled with project-specific values
- Added sections: All template sections completed
- Removed sections: None
- Templates requiring updates:
  ✅ plan-template.md (reviewed - aligned)
  ✅ spec-template.md (reviewed - aligned)
  ✅ tasks-template.md (reviewed - aligned)
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Clarity First
All content MUST be accurate and beginner-friendly. Technical explanations MUST use accessible language without sacrificing precision. Every concept MUST be explained with context before diving into implementation details.

**Rationale**: Students entering Physical AI come from diverse backgrounds. Complex topics become barriers if not properly scaffolded. Clear explanations accelerate learning and reduce cognitive load.

### II. Consistency
All chapters MUST follow a unified structure: Learning Objectives → Core Concepts → Practical Explanations → Visual Aids → Summary. Terminology MUST be used consistently across all modules. Formatting and style MUST be uniform.

**Rationale**: Consistent structure reduces cognitive overhead, allowing students to focus on content rather than navigation. Predictable patterns support efficient learning and reference.

### III. Simplicity
UI MUST be clean with fast loading times and intuitive navigation. Features MUST serve a clear learning purpose—no feature bloat. Code examples MUST be minimal and focused on the concept being taught.

**Rationale**: Complexity is the enemy of learning. Every additional feature, style, or navigation element competes for student attention. Simple systems are maintainable and scale better.

### IV. Modularity
Each chapter MUST be self-contained with clear entry and exit points. Dependencies between chapters MUST be explicitly stated. Content MUST be structured to allow independent study of individual topics.

**Rationale**: Students learn at different paces and may need to revisit specific topics. Self-contained chapters enable flexible learning paths and easier content updates.

### V. Accessibility
Content MUST be readable on desktop and mobile devices. Navigation MUST be keyboard-accessible. Visual aids MUST include alt text. Color schemes MUST meet WCAG contrast standards.

**Rationale**: Educational resources must be available to all learners regardless of device, ability, or learning environment. Accessibility is not optional—it's a core quality standard.

## Content Standards

### Accuracy
- All technical content MUST align with course learning outcomes
- Code examples MUST be tested and functional
- External resources MUST be verified before linking
- Deprecated or outdated practices MUST NOT be presented without clear warnings

### Presentation Quality
- Professional styling matching Panaversity brand standards
- Consistent use of headings, lists, code blocks, and visual hierarchy
- Images and diagrams MUST be high-quality and clearly labeled
- Tables MUST be responsive and readable on all screen sizes

### Performance Standards
- Pages MUST load in under 3 seconds on standard connections
- Images MUST be optimized for web delivery
- Search functionality MUST return results within 1 second
- Site MUST be fully functional without JavaScript (progressive enhancement)

## Development Workflow

### Spec-Driven Development
All features and content additions MUST follow the SpecKit Plus workflow:
1. **Constitution** (this document) → defines principles
2. **Specification** (`/sp.specify`) → captures requirements
3. **Planning** (`/sp.plan`) → designs architecture
4. **Tasks** (`/sp.tasks`) → breaks into testable units
5. **Implementation** (`/sp.implement`) → executes plan

### Iterative Content Cycles
Content development MUST proceed in small, testable increments:
- Complete one chapter before starting another
- Review and validate each chapter before moving forward
- Incorporate feedback before expanding scope
- Deploy working increments to get early student feedback

### Quality Gates
Before merging content:
- All markdown MUST validate without errors
- All internal links MUST resolve correctly
- All code examples MUST be syntax-checked
- Visual elements MUST render correctly on mobile and desktop
- Search index MUST include new content

## Governance

### Authority
This constitution supersedes all other practices and guides all development decisions. When practices conflict with these principles, the constitution wins.

### Amendment Process
Constitutional changes require:
1. Documented rationale for the change
2. Impact analysis on existing content and workflows
3. Migration plan for affected artifacts
4. Approval before implementation
5. Version increment per semantic versioning rules

### Compliance
- All feature planning MUST include a "Constitution Check" validating alignment with principles
- All pull requests MUST verify compliance with relevant principles
- Complexity MUST be justified when principles require trade-offs
- Runtime development guidance is provided in `CLAUDE.md` for agent-specific execution

### Versioning
Constitution follows semantic versioning (MAJOR.MINOR.PATCH):
- **MAJOR**: Backward-incompatible changes (principle removals, redefinitions)
- **MINOR**: New principles or materially expanded guidance
- **PATCH**: Clarifications, wording fixes, non-semantic refinements

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
