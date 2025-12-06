/**
 * TypeScript type definitions for custom React components
 * Used in MDX content for the Physical AI & Humanoid Robotics Textbook
 */

/**
 * Learning Objectives Component
 * Displays a styled list of learning objectives for a week
 */
export interface LearningObjectivesProps {
  /** Child elements (typically <li> items or plain text list) */
  children: React.ReactNode;
  /** Optional CSS class for custom styling */
  className?: string;
}

/**
 * Code Example Component
 * Enhanced code block with title, highlights, and copy button
 */
export interface CodeExampleProps {
  /** Programming language for syntax highlighting */
  language: 'python' | 'cpp' | 'yaml' | 'xml' | 'bash' | 'javascript' | 'typescript' | 'json';
  /** Source code to display */
  code: string;
  /** Optional title/filename displayed above code block */
  title?: string;
  /** Optional line numbers to highlight (e.g., [5, 7, 8]) */
  highlightLines?: number[];
  /** Optional description displayed below code block */
  description?: string;
  /** Show line numbers (default: true) */
  showLineNumbers?: boolean;
  /** Enable copy button (default: true) */
  showCopyButton?: boolean;
  /** Maximum height before scrolling (CSS value, e.g., "400px") */
  maxHeight?: string;
}

/**
 * Callout Component
 * Styled admonition box for notes, warnings, tips, etc.
 */
export interface CalloutProps {
  /** Callout type determines icon and color scheme */
  type: 'note' | 'tip' | 'info' | 'warning' | 'danger' | 'success';
  /** Optional custom title (defaults based on type) */
  title?: string;
  /** Callout content */
  children: React.ReactNode;
  /** Allow collapse/expand (default: false) */
  collapsible?: boolean;
  /** Default collapsed state if collapsible (default: false) */
  collapsed?: boolean;
}

/**
 * Concept Card Component
 * Styled card for displaying core concepts with definition and context
 */
export interface ConceptCardProps {
  /** Concept name/title */
  name: string;
  /** Concept definition */
  definition: string;
  /** Context/relevance explanation */
  context?: string;
  /** Related concepts (array of concept names) */
  relatedConcepts?: string[];
  /** Optional icon (emoji or icon name) */
  icon?: string;
}

/**
 * Module Overview Component
 * Card component for landing page module previews
 */
export interface ModuleOverviewProps {
  /** Module number (1-4) */
  moduleNumber: 1 | 2 | 3 | 4;
  /** Module title */
  title: string;
  /** Module description */
  description: string;
  /** Array of week titles in this module */
  weeks: string[];
  /** Link to first week in module */
  href: string;
  /** Optional icon or image URL */
  icon?: string;
  /** Optional badge text (e.g., "Weeks 1-5") */
  badge?: string;
}

/**
 * Progress Indicator Component
 * Shows user's progress through the course (optional future feature)
 */
export interface ProgressIndicatorProps {
  /** Total number of weeks */
  totalWeeks: number;
  /** Number of completed weeks */
  completedWeeks: number;
  /** Current week number */
  currentWeek: number;
  /** Show detailed breakdown by module (default: false) */
  showModules?: boolean;
}

/**
 * Visual Aid Wrapper Component
 * Wrapper for images and diagrams with caption and zoom
 */
export interface VisualAidProps {
  /** Image source (path or URL) */
  src: string;
  /** Alt text for accessibility */
  alt: string;
  /** Caption displayed below image */
  caption?: string;
  /** Image width (CSS value or number in px) */
  width?: string | number;
  /** Enable click-to-zoom (default: true) */
  zoomable?: boolean;
  /** Align image (default: "center") */
  align?: 'left' | 'center' | 'right';
}

/**
 * Week Summary Component
 * Styled summary section at end of week content
 */
export interface WeekSummaryProps {
  /** Summary content (key takeaways) */
  children: React.ReactNode;
  /** Optional link to next week */
  nextWeek?: {
    title: string;
    href: string;
  };
  /** Optional additional resources */
  resources?: Array<{
    title: string;
    href: string;
    type: 'documentation' | 'video' | 'article' | 'github';
  }>;
}

/**
 * Interactive Diagram Component
 * Wrapper for Mermaid diagrams with optional click interactions
 */
export interface InteractiveDiagramProps {
  /** Mermaid diagram code */
  mermaidCode: string;
  /** Diagram caption */
  caption?: string;
  /** Enable node click interactions (default: false) */
  interactive?: boolean;
  /** Callback when node is clicked (if interactive) */
  onNodeClick?: (nodeId: string) => void;
}

/**
 * Table of Contents Component
 * Custom ToC for complex pages (overrides default Docusaurus ToC)
 */
export interface TableOfContentsProps {
  /** Heading items to display */
  items: Array<{
    id: string;
    title: string;
    level: 2 | 3 | 4;
  }>;
  /** Minimum heading level to display (default: 2) */
  minHeadingLevel?: 2 | 3 | 4;
  /** Maximum heading level to display (default: 3) */
  maxHeadingLevel?: 2 | 3 | 4;
}

/**
 * Quiz Component (Optional Future Feature)
 * Interactive self-assessment quiz
 */
export interface QuizProps {
  /** Quiz title */
  title: string;
  /** Quiz questions */
  questions: Array<{
    id: string;
    question: string;
    options: string[];
    correctAnswer: number;
    explanation?: string;
  }>;
  /** Show results immediately (default: false) */
  immediateResults?: boolean;
  /** Passing score percentage (default: 70) */
  passingScore?: number;
}

/**
 * Resource Link Component
 * Styled external resource link with icon
 */
export interface ResourceLinkProps {
  /** Link URL */
  href: string;
  /** Link text */
  children: React.ReactNode;
  /** Resource type (determines icon) */
  type?: 'documentation' | 'video' | 'article' | 'github' | 'external';
  /** Open in new tab (default: true for external links) */
  newTab?: boolean;
}

/**
 * Step-by-Step Guide Component
 * Numbered steps for tutorials and walkthroughs
 */
export interface StepGuideProps {
  /** Guide steps */
  steps: Array<{
    title: string;
    content: React.ReactNode;
    code?: string;
    language?: string;
  }>;
  /** Show step numbers (default: true) */
  numbered?: boolean;
}

/**
 * Comparison Table Component
 * Styled table for comparing technologies, approaches, etc.
 */
export interface ComparisonTableProps {
  /** Table title */
  title?: string;
  /** Column headers */
  headers: string[];
  /** Table rows */
  rows: Array<{
    label: string;
    values: Array<string | React.ReactNode>;
  }>;
  /** Highlight column index (default: none) */
  highlightColumn?: number;
}

/**
 * Landing Page Hero Component
 * Hero section on landing page
 */
export interface HeroProps {
  /** Main title */
  title: string;
  /** Subtitle/tagline */
  tagline: string;
  /** Call-to-action button text */
  ctaText?: string;
  /** Call-to-action button link */
  ctaHref?: string;
  /** Background image or gradient */
  backgroundImage?: string;
  /** Hero height (CSS value, default: "60vh") */
  height?: string;
}
