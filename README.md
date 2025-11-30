# Physical AI & Humanoid Robotics Textbook

An online textbook platform for teaching Physical AI and Humanoid Robotics, featuring interactive content, AI-powered chatbot assistance, and personalized learning experiences.

## Features

### 🎓 Textbook Access & Reading (User Story 1)
- Browse and navigate through comprehensive course content
- Search functionality for quick content discovery
- Responsive design optimized for learning

### 🤖 RAG Chatbot Interaction (User Story 2)
- AI-powered Q&A about textbook content
- Context-aware responses using Retrieval-Augmented Generation (RAG)
- Integration with Google Gemini for natural language understanding
- Source references and confidence scores

### 🎯 Personalized Content (User Story 3)
- User preferences management (language, theme)
- Personalized content recommendations based on reading history
- Content translation to preferred languages
- Adaptive learning experience

## Tech Stack

### Frontend
- **Docusaurus 3.9+**: Static site generator for textbook content
- **React 18+**: UI framework
- **TypeScript**: Type-safe development

### Backend
- **FastAPI 0.122+**: High-performance Python web framework
- **Google Gemini API**: AI chatbot and translation services
- **Qdrant**: Vector database for RAG (optional)
- **PostgreSQL**: User data and authentication (optional)

## Quick Start

### Prerequisites
- Node.js 18+ and npm/yarn
- Python 3.10+
- (Optional) Docker for PostgreSQL and Qdrant

### Frontend Setup

```bash
cd frontend
npm install
npm start
```

Frontend runs at `http://localhost:3000`

### Backend Setup

```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m uvicorn main:app --reload
```

Backend runs at `http://localhost:8000`

### Environment Configuration

Create `backend/.env`:

```env
GEMINI_API_KEY=your-api-key-here
GEMINI_MODEL=gemini-2.5-flash
ENVIRONMENT=development
DEBUG=true
```

See `specs/001-textbook-ai-robotics/quickstart.md` for detailed setup instructions.

## API Documentation

When the backend is running:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

## Project Structure

```
.
├── backend/              # FastAPI backend
│   ├── src/
│   │   ├── api/         # API endpoints
│   │   ├── services/    # Business logic
│   │   ├── models/      # Data models
│   │   └── utils/       # Utilities
│   └── main.py         # Application entry point
├── frontend/            # Docusaurus frontend
│   ├── src/
│   │   ├── components/  # React components
│   │   ├── pages/       # Docusaurus pages
│   │   └── services/    # API clients
│   └── docs/           # Textbook content
└── specs/              # Specification and planning docs
```

## Development

### Running Tests

**Frontend:**
```bash
cd frontend
npm test
```

**Backend:**
```bash
cd backend
source venv/bin/activate
pytest
```

### Code Quality

- TypeScript/ESLint for frontend
- Python type hints and linting for backend
- Pre-commit hooks (if configured)

## Deployment

### Frontend
The Docusaurus site can be deployed to:
- GitHub Pages
- Netlify
- Vercel
- Any static hosting service

Build command: `npm run build`

### Backend
The FastAPI application can be deployed to:
- Cloud platforms (AWS, GCP, Azure)
- Container platforms (Docker, Kubernetes)
- Serverless functions

## Contributing

1. Follow the specification in `specs/001-textbook-ai-robotics/`
2. Review `specs/001-textbook-ai-robotics/tasks.md` for implementation tasks
3. Ensure all tests pass before submitting

## License

[Add your license here]

## Support

For issues and questions, please refer to:
- Quickstart guide: `specs/001-textbook-ai-robotics/quickstart.md`
- API documentation: http://localhost:8000/docs (when backend is running)


