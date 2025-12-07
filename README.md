# Physical AI & Humanoid Robotics Textbook

A comprehensive 13-week course covering Physical AI, ROS 2, robot simulation, NVIDIA Isaac platform, and Vision-Language-Action (VLA) systems.

## Overview

This open-source textbook provides a complete learning path for students and practitioners interested in Physical AI and humanoid robotics. The course is structured into 4 modules spanning 13 weeks of content.

### Course Structure

**Module 1: Introduction to Physical AI & ROS 2 (Weeks 1-5)**
- Physical AI foundations and embodied intelligence
- ROS 2 architecture, nodes, and communication
- ROS 2 package development

**Module 2: Robot Simulation (Weeks 6-7)**
- Gazebo simulation environment
- Unity robotics integration

**Module 3: NVIDIA Isaac Platform (Weeks 8-10)**
- Isaac SDK and Isaac Sim
- Perception systems and reinforcement learning
- Sim-to-real transfer techniques

**Module 4: Humanoid Robotics & VLA (Weeks 11-13)**
- Humanoid kinematics and bipedal locomotion
- Vision-Language-Action systems
- Conversational robotics with GPT integration

## Getting Started

### Prerequisites

- Node.js 18 or higher
- npm or yarn package manager
- Git

### Installation

1. Clone the repository:
```bash
git clone https://github.com/your-org/ai-native-text-book.git
cd ai-native-text-book
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

The site will open at `http://localhost:3000`.

### Build for Production

```bash
npm run build
```

This command generates static content into the `build` directory that can be served using any static hosting service.

## Project Structure

```
ai-native-text-book/
├── docs/                  # Course content (MDX files)
│   ├── intro.md          # Course overview
│   ├── module-1-physical-ai/
│   ├── module-2-simulation/
│   ├── module-3-isaac/
│   └── module-4-humanoid-vla/
├── src/
│   ├── components/       # Custom React components
│   ├── css/             # Global styles
│   └── pages/           # Custom pages (landing page)
├── static/
│   └── img/             # Static images and diagrams
├── docusaurus.config.js # Docusaurus configuration
└── sidebars.js          # Sidebar navigation
```

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details on:
- Content contribution process
- Frontmatter schema requirements
- Code of conduct
- Pull request process

## Technology Stack

- **Framework**: Docusaurus 3.x
- **Language**: TypeScript, React 18
- **Content Format**: MDX (Markdown + JSX)
- **Styling**: Custom CSS with dark theme
- **Search**: Local search plugin
- **Deployment**: GitHub Pages

## License

This content is licensed under [CC-BY-4.0](https://creativecommons.org/licenses/by/4.0/).

Copyright © 2025 Panaversity

## Contact

- Website: [ai-native.panaversity.org](https://ai-native.panaversity.org)
- GitHub: [github.com/your-org/ai-native-text-book](https://github.com/your-org/ai-native-text-book)

## Acknowledgments

This textbook is part of the Panaversity AI-Native initiative.
