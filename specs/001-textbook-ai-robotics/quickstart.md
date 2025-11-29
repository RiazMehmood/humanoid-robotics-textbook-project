# Quickstart Guide: Create Textbook for Teaching Physical AI & Humanoid Robotics Course

**Feature Branch**: `001-textbook-ai-robotics`
**Date**: 2025-11-29

This guide provides instructions to set up the development environment and run the online textbook application locally.

## 1. Prerequisites

Ensure you have the following installed:

*   **Git**: For cloning the repository.
*   **Node.js** (v18 or higher) and **npm/yarn**: For the Docusaurus frontend.
*   **Python 3.10+** and **pip**: For the FastAPI backend.
*   **Docker** (optional but recommended): For easily setting up local PostgreSQL and a Vector Database (like Qdrant).

## 2. Clone the Repository

First, clone the project repository:

```bash
git clone <repository-url>
cd <repository-name>
git checkout 001-textbook-ai-robotics
```

## 3. Frontend Setup (Docusaurus)

Navigate to the `frontend/` directory, install dependencies, and start the Docusaurus development server.

```bash
cd frontend
npm install  # or yarn install
npm start    # or yarn start
```

This will usually open the textbook in your browser at `http://localhost:3000`.

## 4. Backend Setup (FastAPI)

Navigate to the `backend/` directory, set up a Python virtual environment, install dependencies, and start the FastAPI server.

```bash
cd backend
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -r requirements.txt # Create this file with FastAPI, uvicorn, openai, psycopg2, qdrant-client etc.
uvicorn main:app --reload
```

The backend API will typically be available at `http://localhost:8000`.

## 5. Database Setup (PostgreSQL & Vector DB)

For local development, you can use Docker to run PostgreSQL and a Vector Database (e.g., Qdrant).

### PostgreSQL

```bash
docker run --name some-postgres -e POSTGRES_PASSWORD=mysecretpassword -p 5432:5432 -d postgres
```

### Vector Database (Qdrant example)

```bash
docker run -p 6333:6333 -p 6334:6334 -d qdrant/qdrant
```

Ensure your backend configuration connects to these local database instances.

## 6. Running Tests

### Frontend Tests

```bash
cd frontend
npm test # or yarn test
```

### Backend Tests

```bash
cd backend
source .venv/bin/activate
pytest
```
