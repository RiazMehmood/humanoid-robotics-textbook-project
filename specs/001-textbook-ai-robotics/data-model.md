# Data Model: Create Textbook for Teaching Physical AI & Humanoid Robotics Course

**Feature Branch**: `001-textbook-ai-robotics`
**Date**: 2025-11-29
**Source**: Derived from `specs/001-textbook-ai-robotics/spec.md`

## Entities

### User

Represents an individual accessing the platform.

*   **Attributes:**
    *   `id` (Unique identifier)
    *   `email` (Authentication credential)
    *   `password_hash` (Secured authentication credential)
    *   `preferences` (e.g., `language_code`: string, `theme`: string)
    *   `interaction_history` (e.g., `read_chapters`: list of strings, `chatbot_queries`: list of strings)
*   **Relationships:**
    *   One-to-many with `Authentication Token/Session`

### Textbook Content

Structured educational material.

*   **Attributes:**
    *   `id` (Unique identifier, e.g., chapter_id-section_id)
    *   `title` (Chapter/Section title)
    *   `content` (Main text body, markdown format)
    *   `images` (List of image URLs or references)
    *   `metadata` (e.g., `chapter_number`: int, `section_number`: int, `keywords`: list of strings)
*   **Relationships:**
    *   Accessed by `User`

### Chatbot Query

User input question to the AI chatbot.

*   **Attributes:**
    *   `id` (Unique identifier)
    *   `user_id` (Reference to `User`)
    *   `timestamp` (When the query was made)
    *   `query_text` (The actual question text)
    *   `context` (e.g., `current_page_url`: string, `selected_text`: string)
*   **Relationships:**
    *   Many-to-one with `User`
    *   One-to-one with `Chatbot Response`

### Chatbot Response

AI-generated answer provided by the AI chatbot.

*   **Attributes:**
    *   `id` (Unique identifier)
    *   `query_id` (Reference to `Chatbot Query`)
    *   `timestamp` (When the response was generated)
    *   `response_text` (The AI-generated answer)
    *   `source_references` (e.g., `textbook_sections`: list of strings, `external_links`: list of URLs)
    *   `confidence_score` (Numeric score indicating response confidence)
*   **Relationships:**
    *   One-to-one with `Chatbot Query`

### Authentication Token/Session

Data used by the authentication service to manage user sessions and permissions.

*   **Attributes:**
    *   `id` (Unique identifier)
    *   `user_id` (Reference to `User`)
    *   `token_value` (Secured token string)
    *   `expiration_timestamp` (When the token expires)
    *   `is_active` (Boolean indicating active session status)
*   **Relationships:**
    *   Many-to-one with `User`
