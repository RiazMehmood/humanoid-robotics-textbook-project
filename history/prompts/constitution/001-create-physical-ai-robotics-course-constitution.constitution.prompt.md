---
id: 001
title: Create Physical AI Robotics Course Constitution
stage: constitution
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution", "initial-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:

---

## Prompt

create a constitution for this project Hackathon I: Create a Textbook for Teaching Physical AI & Humanoid Robotics Course
The future of work will be a partnership between people, intelligent agents (AI software), and robots. This shift won't necessarily eliminate jobs but will change what humans do, leading to a massive demand for new skills. We have already written a book on AI agents. Therefore, we want you to write a textbook to teach a course in Physical AI & Humanoid Robotics (The course details are documented below).

Excel in the Hackathon and Launch Your Journey as an AI Startup Founder 🚀
We’ve recently launched Panaversity (panaversity.org), an initiative focused on teaching cutting-edge AI courses. Alongside this, we’re working on publishing our first book, which you can explore at ai-native.panaversity.org. Our next milestone is to build a portal where authors can create AI-native technical textbooks, and readers can easily access and learn from them using AI Agents. We also plan to publish O/A Level, Science, Engineering, and Medical AI-native books to support students and professionals across disciplines. If you perform well in this hackathon, you may be invited for an interview to join the Panaversity core team and potentially step into the role of a startup founder within this growing ecosystem. You will get a chance to work with Panaversity founders Zia, Rehan, Junaid, and Wania and become the very best. You may also get a chance to teach at Panaversity, PIAIC, and GIAIC.
Requirements

You are required to complete a unified book project using Claude Code and Spec-Kit Plus. The core deliverables are:

1. AI/Spec-Driven Book Creation: Write a book using Docusaurus and deploy it to GitHub Pages. You will use Spec-Kit Plus ( https://github.com/panaversity/spec-kit-plus/ ) and Claude Code ( https://www.claude.com/product/claude-code ) to write the book.

2. Integrated RAG Chatbot Development: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user.

3. Participants will receive points out of 100, for base functionality defined above.

4. Participants can earn up to 50 extra bonus points by creating and using reusable intelligence via Claude Code Subagents and Agent Skills in the book project.

5. Participants can receive up to 50 extra bonus points if they also implement Signup and Signin using https://www.better-auth.com/ At signup you will ask questions from the user about their software and hardware background. Knowing the background of the user we will be able to personalize the content.

6.  Participants can receive up to 50 extra bonus points if the logged user can personalise the content in the chapters by pressing a button at the start of each chapter.

7. Participants can receive up to 50 extra bonus points if the logged user can translate the content in Urdu in the chapters by pressif each chapter.

## Response snapshot

The project constitution has been created and updated at `.specify/memory/constitution.md`.

New constitution version: 1.0.0
Bump rationale: Initial creation of the project constitution based on the provided requirements.
Files flagged for manual follow-up: None.

Suggested commit message:
```
docs: create constitution to v1.0.0 (initial creation + principles + governance)

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

## Outcome

- ✅ Impact: Project constitution created and integrated with core principles and governance.
- 🧪 Tests: No explicit tests were run for this constitution creation.
- 📁 Files: `.specify/memory/constitution.md` updated. `.specify/templates/plan-template.md`, `.specify/templates/spec-template.md`, `.specify/templates/tasks-template.md` checked for consistency.
- 🔁 Next prompts: Continue with project specification based on the new constitution.
- 🧠 Reflection: Successfully created the project constitution following the provided guidelines and handled the absence of the PHR script gracefully by falling back to agent-native creation.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` script not found.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure `create-phr.sh` or similar scripts are present and executable in future environments, or improve the error handling for script execution failure to provide more informative messages.
