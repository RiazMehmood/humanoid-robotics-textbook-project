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
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\Activate.ps1
pip install -r requirements.txt  # Installs FastAPI, uvicorn, google-generativeai, qdrant-client, etc.
python -m uvicorn main:app --reload
```

The backend API will typically be available at `http://localhost:8000`.

**Note**: The backend can run without a database. If you don't have PostgreSQL installed, the chatbot and personalization features will work with mock data. To use the full features:
- Set up PostgreSQL (see Database Setup below)
- Configure `GEMINI_API_KEY` in `backend/.env` for AI chatbot functionality
- Set up Qdrant for vector database (optional, chatbot works without it)

## 5. Environment Configuration

Create a `.env` file in the `backend/` directory with the following variables:

```env
# Optional: Database configuration (if using PostgreSQL)
DATABASE_URL=postgresql://user:password@localhost:5432/textbook_db

# Optional: Vector Database (if using Qdrant)
QDRANT_URL=http://localhost:6333
QDRANT_COLLECTION_NAME=textbook_content

# Optional: AI Chatbot (Google Gemini API)
GEMINI_API_KEY=your-gemini-api-key-here
GEMINI_MODEL=gemini-2.5-flash
GEMINI_EMBEDDING_MODEL=models/embedding-gecko-001

# Application settings
ENVIRONMENT=development
DEBUG=true
SECRET_KEY=your-secret-key-here
CORS_ORIGINS=["http://localhost:3000","http://127.0.0.1:3000"]
```

**Note**: The application can run without these configurations. The chatbot will use mock responses if `GEMINI_API_KEY` is not set.

## 6. Database Setup (PostgreSQL & Vector DB) - Optional

For local development with full database features, you can use Docker to run PostgreSQL and a Vector Database (e.g., Qdrant).

### PostgreSQL

```bash
docker run --name some-postgres -e POSTGRES_PASSWORD=mysecretpassword -p 5432:5432 -d postgres
```

### Vector Database (Qdrant example)

```bash
docker run -p 6333:6333 -p 6334:6334 -d qdrant/qdrant
```

Ensure your backend configuration connects to these local database instances by updating the `.env` file.

## 7. API Documentation

Once the backend is running, you can access:
- **Swagger UI**: `http://localhost:8000/docs`
- **ReDoc**: `http://localhost:8000/redoc`
- **OpenAPI JSON**: `http://localhost:8000/openapi.json`

## 8. Running Tests

### Frontend Tests

```bash
cd frontend
npm test # or yarn test
```

### Backend Tests

```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\Activate.ps1
pytest
```

## 9. Features Overview

The application includes:

1. **Textbook Access & Reading (User Story 1)**: Browse and read textbook content via Docusaurus
2. **RAG Chatbot (User Story 2)**: Ask questions about textbook content with AI-powered responses
3. **Personalized Content (User Story 3)**: 
   - User preferences (language, theme)
   - Content recommendations based on reading history
   - Content translation to preferred language

## 10. Troubleshooting

### Backend won't start
- Ensure Python 3.10+ is installed
- Activate the virtual environment: `source venv/bin/activate` (or `venv\Scripts\Activate.ps1` on Windows)
- Install dependencies: `pip install -r requirements.txt`
- Check if port 8000 is available

### Frontend won't start
- Ensure Node.js 18+ is installed
- Install dependencies: `npm install`
- Check if port 3000 is available

### Chatbot returns mock responses
- Set `GEMINI_API_KEY` in `backend/.env`
- Get a free API key at: https://makersuite.google.com/app/apikey
- Restart the backend server

### Database connection errors
- The application can run without a database
- For full features, ensure PostgreSQL is running and configured in `.env`
- Vector DB (Qdrant) is optional for chatbot functionality
