---
id: 0002
title: Clarification on PHR Meaning
stage: general
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: feature/constitution-update
user: unknown
command: what is phr?
labels: ["phr", "clarification"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
null
tests:
null
---

## Prompt

what is phr?

## Response snapshot

PHR stands for Prompt History Record. It's a mechanism used to document every user input verbatim and the key assistant output for learning and traceability. These records are stored in the `history/prompts/` directory, organized by context (e.g., constitution, feature-specific, or general).

## Outcome

- ✅ Impact: Provided a clear explanation of PHR.
- 🧪 Tests: No tests were run.
- 📁 Files: No files were directly modified or created as part of the explanation.
- 🔁 Next prompts: User acknowledged clarification.
- 🧠 Reflection: Successfully provided a concise and accurate definition of PHR as requested.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
