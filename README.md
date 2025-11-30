# Physical AI & Humanoid Robotics Textbook Platform

A comprehensive, AI-powered online textbook platform designed to teach Physical AI and Humanoid Robotics. This platform combines static textbook content with an interactive AI chatbot (RAG-based) and personalized learning features.

## Features

-   **Interactive Textbook**: Structured modules on ROS 2, Simulation, and Physical AI.
-   **AI Chatbot (RAG)**: Context-aware answers based on textbook content.
-   **Personalization**: Content recommendations based on user background and reading history.
-   **Authentication**: User accounts to save progress and preferences.

## Project Structure

-   `frontend/`: React-based Docusaurus static site generator.
-   `backend/`: FastAPI Python backend with RAG logic (LangChain/Gemini).
-   `specs/`: Project documentation and specifications.

## Quick Start

### 1. Backend Setup

```bash
cd backend
python -m venv venv
# Windows
.\venv\Scripts\activate
# Mac/Linux
source venv/bin/activate

pip install -r requirements.txt
```

Create a `.env` file in `backend/` (see `backend/src/config.py` for required variables):
```
DATABASE_URL=postgresql://...
GEMINI_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...
SECRET_KEY=...
CORS_ORIGINS=http://localhost:3000
```

Run the server:
```bash
uvicorn main:app --reload
```

### 2. Content Ingestion (Important!)

To populate the AI's knowledge base with the textbook content:

```bash
# Make sure backend is running
python backend/ingest_content.py --api-url http://localhost:8000
```

### 3. Frontend Setup

```bash
cd frontend
npm install
npm start
```

The site will open at `http://localhost:3000`.

## Deployment

-   **Frontend**: Deployed on Vercel.
-   **Backend**: Deployed on Railway.
-   **Database**: Neon Postgres (via Railway).
-   **Vector DB**: Qdrant Cloud.

See `DEPLOYMENT_QUICK_START.md` for detailed deployment instructions.

## License

MIT
