---
id: 1
title: ai-humanoid-robotics-book-constitution
stage: constitution
date: 2025-12-15
surface: agent
model: Claude Sonnet 4.5
feature: ai-humanoid-robotics-book
branch: 001-ai-humanoid-robotics-book
user: user
command: /sp.constitution
labels: [constitution, robotics, ai, book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - 001-ai-humanoid-robotics-book/checklists/requirements.md
tests:
 - null
---

## Prompt

/sp.constitution

Project Title:
"AI-Native Physical Humanoid Robotics: Architecture, Simulation, and Cognition"

Project Type:
Academic + Engineering Book (Research-grade, reproducible, industry-aligned)

Primary Goal:
Generate a complete constitution file that governs the writing, verification, and generation of a full-length technical book on Physical Humanoid Robotics integrated with AI, Cloud, and RAG-based cognition systems.

Target Audience:
- Graduate-level Computer Science & Robotics students
- Robotics researchers
- AI engineers (ROS 2, Simulation, LLMs, Cloud Robotics)

Writing Principles (MANDATORY):
1. Accuracy through primary-source verification only
2. Academic clarity (Flesch-Kincaid Grade 10–12)
3. Reproducibility (code, configs, datasets must be traceable)
4. Zero plagiarism tolerance
5. Peer-reviewed preference (minimum 50%)
6. Engineering realism (no speculative claims without citation)
- Each ROS concept mapped to biological nervous systems

---

Module 2: The Digital Twin (Gazebo & Unity)
Focus:
- Physics simulation (ODE / Bullet)
- Gravity, collision, friction modeling
- Gazebo for robotics accuracy
- Unity for human-robot interaction & rendering
- Sensor simulation:
  - LiDAR
  - Depth Cameras
  - IMU
Standards:
- Simulation-to-Reality (Sim2Real) gap discussion
- Parameter tables for reproducibility

---

Module 3: The AI-Robot Brain (NVIDIA Isaac™)
Focus:
- NVIDIA Isaac Sim (synthetic data, photorealism)
- Isaac ROS pipelines
- GPU-accelerated VSLAM

Citation & Verification Rules:
- Citation Style: APA (7th Edition)
- Every factual claim MUST be traceable
- Each chapter must include a "Claim-to-Source Mapping" table
- Use DOI-based citations where possible
- No uncited claims allowed
- Inline citations + reference section required

Output Format Rules:
- Final book output: PDF
- Embedded citations
- Diagrams described textually (LaTeX/Markdown compatible)
- Code blocks must be executable or clearly marked as pseudocode

Book Length Constraints:
- Total Word Count: 5,000–7,000 words
- Minimum Sources: 15
- Minimum Peer-Reviewed Sources: 50%

---

MODULE STRUCTURE (MANDATORY):

Module 1: The Robotic Nervous System (ROS 2)
Focus:
- ROS 2 middleware architecture
- Nodes, Topics, Services, Actions
- rclpy-based Python AI agents
- Bridging LLM-based planners to ROS controllers
- URDF/XACRO for humanoid morphology
Standards:
- ROS 2 Humble for humanoid navigation
- Bipedal constraints
Standards:
- Hardware assumptions must be explicit
- Performance benchmarks cited

---

Module 4: Vision-Language-Action (VLA)
Focus:
- Multimodal cognition
- Voice-to-Action using Whisper
- Language-to-Plan using LLMs
- Plan-to-ROS Action Graphs
- Object detection + manipulation
Capstone Project:
"Autonomous Humanoid Assistant"
- Voice command ingestion
- Task decomposition
- Path planning
- Object recognition
- Manipulation execution

---

RAG SYSTEM (MANDATORY SECTION):

Design a Retrieval-Augmented Generation (RAG) system specifically for a Physical Humanoid Robot.

RAG Requirements:
- Knowledge Sources:
  - Robotics manuals
  - ROS documentation
  - Academic papers
  - Sensor datasheets
- Vector database architecture
- Cloud-based inference
- Edge execution constraints
- Safety filtering layer
- Deterministic fallback behaviors

Explain:
- Query → Retrieval → Reasoning → Action loop
- Latency constraints
- Failure modes
- Hallucination mitigation

---

Cloud & Tooling Constraints:
- Cloud-based AI reasoning
- SpecKitPlus integration assumed
- Edge-device safety enforcement
- Offline-safe modes required

---

Success Criteria (NON-NEGOTIABLE):
- Every claim verified
- Zero plagiarism
- Passes academic peer review
- Fully traceable sources
- Technically executable concepts

---

Claude Instructions:
- Produce ONLY the constitution file
- Use structured sections and bullet points
- Write in formal academic English
- Do NOT write the book itself
- Do NOT include filler text
- Do NOT speculate without citations


this is my constitution

## Response snapshot

I have successfully created a constitution file for the AI-Native Physical Humanoid Robotics book project. The constitution includes:

1. Core principles covering academic excellence, technical accuracy, content traceability, reproducibility standards, technology integration, and quality assurance
2. Content standards with writing requirements, output format requirements, and book constraints
3. Complete module architecture covering all 4 required modules (ROS 2, Digital Twin, AI-Robot Brain, VLA)
4. RAG system architecture with all specified requirements
5. Development workflow and quality gates
6. Governance section

The constitution meets all the requirements specified in the original feature description, including:
- Academic clarity and primary-source verification
- Citation requirements (APA 7th Edition)
- Module specifications with their specific focus areas
- RAG system requirements
- Book constraints (word count, sources, etc.)
- Success criteria for verification and peer review

A quality checklist was also created to validate the constitution against the requirements.

## Outcome

- ✅ Impact: Created complete constitution file governing the AI-Native Physical Humanoid Robotics book project
- 🧪 Tests: null
- 📁 Files: Created .specify/memory/constitution.md and 001-ai-humanoid-robotics-book/checklists/requirements.md
- 🔁 Next prompts: Project constitution is complete, ready for book development
- 🧠 Reflection: The constitution comprehensively covers all requirements from the original specification

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null