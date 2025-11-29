# Tasks: Create Textbook for Teaching Physical AI & Humanoid Robotics Course

**Input**: Design documents from `/specs/001-textbook-ai-robotics/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/, quickstart.md

**Tests**: No explicit request for TDD in the spec, so test tasks will be minimal/implied within implementation tasks for now.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create `backend/` and `frontend/` base directories
- [ ] T002 Initialize Python backend project in `backend/` with FastAPI, uvicorn
- [ ] T003 [P] Initialize Node.js frontend project in `frontend/` with Docusaurus
- [ ] T004 [P] Create `requirements.txt` for backend dependencies in `backend/`
- [ ] T005 [P] Configure `package.json` for frontend dependencies in `frontend/`
- [ ] T006 [P] Configure linting and formatting for Python in `backend/` (e.g., `pyproject.toml`)
- [ ] T007 [P] Configure linting and formatting for TypeScript/JavaScript in `frontend/` (e.g., `.eslintrc.js`)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T008 Setup PostgreSQL database schema and migrations framework for `User` entity in `backend/src/models/user.py` and `backend/src/database/migrations/`
- [ ] T009 Setup Vector Database (e.g., Qdrant) client configuration in `backend/src/database/vector_db.py`
- [ ] T010 Implement authentication/authorization framework using `authentication-api.openapi.yaml` in `backend/src/services/auth_service.py` and `backend/src/api/auth.py`
- [ ] T011 Create `User` data model (entity) in `backend/src/models/user.py`
- [ ] T012 Create `Authentication Token/Session` data model (entity) in `backend/src/models/auth_token.py`
- [ ] T013 [P] Configure global API routing and middleware structure in `backend/src/main.py`
- [ ] T014 [P] Implement common error handling and logging infrastructure in `backend/src/utils/errors.py` and `backend/src/utils/logger.py`
- [ ] T015 [P] Setup environment configuration management (e.g., `.env` handling) in `backend/src/config.py`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Textbook Access & Reading (Priority: P1) 🎯 MVP

**Goal**: A user can access the online textbook, browse chapters, and read content effectively.

**Independent Test**: Navigate to the deployed Docusaurus site, browse through chapters and sections, and verify content readability and navigation.

### Implementation for User Story 1

- [ ] T016 [P] [US1] Set up Docusaurus site configuration in `frontend/docusaurus.config.js`
- [ ] T017 [P] [US1] Integrate static textbook content (markdown files) into Docusaurus structure in `frontend/docs/`
- [ ] T018 [US1] Implement basic navigation and routing for textbook chapters in `frontend/src/pages/` and `frontend/sidebars.js`
- [ ] T019 [US1] Develop search functionality for textbook content in `frontend/src/components/Search.js`
- [ ] T020 [P] [US1] Create frontend services for content fetching (if dynamic) in `frontend/src/services/content_service.js`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - RAG Chatbot Interaction (Priority: P1)

**Goal**: A user can ask questions about the textbook content and receive accurate, context-aware answers from an AI chatbot.

**Independent Test**: Deploy the AI chatbot and its integration with the frontend, ask diverse questions related to textbook content, and verify accuracy and relevance of responses.

### Implementation for User Story 2

- [ ] T021 [P] [US2] Implement AI chatbot core service logic (integration with Vector DB, LLM) in `backend/src/services/chatbot_service.py`
- [ ] T022 [P] [US2] Create API endpoint for chatbot queries based on `ai-chatbot-api.openapi.yaml` in `backend/src/api/chatbot.py`
- [ ] T023 [P] [US2] Create `Chatbot Query` data model in `backend/src/models/chatbot_query.py`
- [ ] T024 [P] [US2] Create `Chatbot Response` data model in `backend/src/models/chatbot_response.py`
- [ ] T025 [US2] Develop frontend chatbot UI component in `frontend/src/components/Chatbot.js`
- [ ] T026 [US2] Integrate chatbot UI with backend API in `frontend/src/services/chatbot_api.js`
- [ ] T027 [US2] Implement context passing from current textbook page to chatbot queries in `frontend/src/services/chatbot_api.js` and `frontend/src/pages/`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Personalized Content (Priority: P2)

**Goal**: A logged-in user can experience personalized content recommendations or have content translated based on their preferences.

**Independent Test**: Create a user account, log in, set preferences, and observe if content recommendations or translations are applied correctly to textbook sections.

### Implementation for User Story 3

- [ ] T028 [P] [US3] Implement content personalization logic in `backend/src/services/personalization_service.py`
- [ ] T029 [P] [US3] Implement content translation service in `backend/src/services/translation_service.py`
- [ ] T030 [P] [US3] Create API endpoints for preferences management, content translation, and recommendations based on `content-personalization-api.openapi.yaml` in `backend/src/api/personalization.py`
- [ ] T031 [US3] Develop frontend UI for user preferences management in `frontend/src/components/UserSettings.js`
- [ ] T032 [US3] Integrate frontend with personalization API for fetching and updating preferences in `frontend/src/services/personalization_api.js`
- [ ] T033 [US3] Implement dynamic display of personalized content recommendations on frontend in `frontend/src/pages/index.js`
- [ ] T034 [US3] Integrate frontend with translation API for displaying translated content in `frontend/src/components/ContentRenderer.js`

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T035 [P] Update `quickstart.md` with final setup instructions and any new configurations.
- [ ] T036 Refactor shared utilities and common code patterns in `backend/src/utils/` and `frontend/src/utils/`
- [ ] T037 Perform overall performance optimization for frontend loading and backend API responses.
- [ ] T038 Review and harden security across all components, paying attention to authentication and data handling.
- [ ] T039 Implement comprehensive end-to-end tests for critical user journeys across all stories in `e2e_tests/` (new directory).
- [ ] T040 Final documentation updates for `README.md`, developer guides, and API documentation.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 (e.g., using content data) but should be independently testable.
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 (e.g., personalizing content) but should be independently testable.

### Within Each User Story

- Models before services
- Services before endpoints/components
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, User Story 1 and User Story 2 can start in parallel (if team capacity allows).
- Tasks within each user story marked [P] can run in parallel (e.g., creating models or API endpoints if independent).
- Different user stories can be worked on in parallel by different team members.

---

## Parallel Example: User Story 1

```bash
# Frontend setup:
Task: "Set up Docusaurus site configuration in frontend/docusaurus.config.js"
Task: "Integrate static textbook content (markdown files) into Docusaurus structure in frontend/docs/"

# Backend (if dynamic content is needed, which it will be for search)
# These depend on Foundational tasks
Task: "Create frontend services for content fetching (if dynamic) in frontend/src/services/content_service.js"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
