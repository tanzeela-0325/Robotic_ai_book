# AI-Native Physical Humanoid Robotics Book

This repository contains the source code for the AI-Native Physical Humanoid Robotics book, organized for Docusaurus documentation generation.

## About This Book

This comprehensive guide explores the cutting-edge field of AI-native physical humanoid robotics, covering:
- The Robotic Nervous System (ROS 2)
- The Digital Twin (Gazebo & Unity)
- The AI-Robot Brain (NVIDIA Isaac™)
- Vision-Language-Action (VLA) systems
- Capstone project integration
- RAG (Retrieval-Augmented Generation) system integration

## Repository Structure

```
docusaurus-book/
├── docs/
│   ├── intro.md                 # Book introduction
│   ├── modules/                 # Four core modules
│   │   ├── module-1-ros-nervous-system/
│   │   ├── module-2-digital-twin/
│   │   ├── module-3-ai-brain/
│   │   └── module-4-vla/
│   ├── capstone-project/
│   └── rag-integration/
├── docusaurus.config.js         # Docusaurus configuration
├── sidebars.js                  # Navigation sidebar configuration
└── package.json                 # Project dependencies
```

## Getting Started

To run this documentation locally:

1. **Install dependencies**:
   ```bash
   npm install
   ```

2. **Start the development server**:
   ```bash
   npm run start
   ```

3. **Build for production**:
   ```bash
   npm run build
   ```

4. **Serve the built site**:
   ```bash
   npm run serve
   ```

## Contributing

Contributions are welcome! Please see the [CONTRIBUTING.md](CONTRIBUTING.md) file for guidelines.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

This book represents the culmination of research and development in AI-native humanoid robotics, incorporating the latest advances in robotics, AI, and simulation technologies.