<!--
Sync Impact Report:
  Version change: None (initial creation) -> 1.0.0
  List of modified principles: None (initial creation)
  Added sections: Core Principles, Technology Stack and Integration, Quality Assurance and Continuous Improvement, Governance
  Removed sections: None
  Templates requiring updates:
    .specify/templates/plan-template.md: ✅ updated
    .specify/templates/spec-template.md: ✅ updated
    .specify/templates/tasks-template.md: ✅ updated
  Follow-up TODOs: None
-->
# Hackathon I: Create a Textbook for Teaching Physical AI & Humanoid Robotics Course Constitution

## Core Principles

### I. Spec-Driven Development (SDD)
All development, especially book content and RAG chatbot features, must follow a spec-driven approach using Spec-Kit Plus and Claude Code. This ensures clarity, traceability, and alignment with project goals.

### II. Modular Architecture
Design the book platform (Docusaurus), RAG chatbot, and authentication system with modularity in mind. Each component should be independently deployable and scalable.

### III. AI-Native & Agent-First
Prioritize the use of AI agents and subagents, including Claude Code Subagents and Agent Skills, for book generation, content management, and RAG chatbot functionality. Embrace AI-native tools and methodologies.

### IV. Robust Data Management
Implement reliable data management for the RAG chatbot using Neon Serverless Postgres and Qdrant Cloud. Ensure data integrity, security, and efficient retrieval.

### V. Secure and Personalized User Experience
Implement secure user authentication and authorization using Better-Auth. Focus on personalizing content based on user background and enabling features like content personalization and translation for logged-in users.

### VI. Deployment and Operational Excellence
Ensure the book is deployable to GitHub Pages with CI/CD considerations. Emphasize observability, maintainability, and clear runbooks for all deployed components.

## Technology Stack and Integration

*   **Book Platform**: Docusaurus, deployed to GitHub Pages.
*   **RAG Chatbot**: OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier.
*   **Authentication**: Better-Auth.
*   **Development Tools**: Claude Code, Spec-Kit Plus.
*   **Interoperability**: Components must integrate seamlessly and communicate via defined APIs.

## Quality Assurance and Continuous Improvement

*   **Testing**: Implement comprehensive testing for all components, including unit, integration, and end-to-end tests for the book content, RAG chatbot, and authentication.
*   **Code Review**: All code changes must undergo peer review.
*   **Documentation**: Maintain up-to-date documentation for all features and architectural decisions.
*   **Feedback Integration**: Regularly collect and integrate user feedback to improve the book content and platform.

## Governance
All development must align with the principles outlined in this constitution.
Amendments to this constitution require a clear rationale, approval from project leads, and documentation of changes.
Compliance with these rules will be reviewed periodically, especially during major feature implementations and releases.
All code contributions must pass automated tests and adhere to established coding standards.

**Version**: 1.0.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29
