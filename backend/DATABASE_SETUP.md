# Quick Database Setup

See `../DATABASE_SETUP.md` for detailed instructions.

## Quick Start

1. **Get Neon Postgres URL**: https://neon.tech → Create project → Copy connection string
2. **Get Qdrant Cloud URL & API Key**: https://cloud.qdrant.io → Create cluster → Copy URL and API key
3. **Create `backend/.env`**:
   ```env
   DATABASE_URL=your-neon-connection-string
   QDRANT_URL=your-qdrant-cluster-url
   QDRANT_API_KEY=your-qdrant-api-key
   GEMINI_API_KEY=your-gemini-api-key
   ```
4. **Install dependencies**:
   ```bash
   pip install sqlalchemy psycopg2-binary alembic python-jose[cryptography] passlib[bcrypt]
   ```
5. **Run migrations**:
   ```bash
   alembic upgrade head
   ```
6. **Start server**:
   ```bash
   python -m uvicorn main:app --reload
   ```

Done! ✅


