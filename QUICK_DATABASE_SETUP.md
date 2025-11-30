# Quick Database Setup Guide

## 🚀 Quick Start (5 minutes)

### Step 1: Neon Postgres (2 minutes)
1. Go to https://neon.tech → Sign up (free)
2. Create new project → Copy connection string
3. Add to `backend/.env`:
   ```env
   DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/neondb?sslmode=require
   ```

### Step 2: Qdrant Cloud (2 minutes)
1. Go to https://cloud.qdrant.io → Sign up (free tier)
2. Create cluster → Copy URL and create API key
3. Add to `backend/.env`:
   ```env
   QDRANT_URL=https://xxx.cloud.qdrant.io:6333
   QDRANT_API_KEY=your-api-key-here
   ```

### Step 3: Install & Setup (1 minute)
```bash
cd backend
pip install sqlalchemy "psycopg[binary]" alembic python-jose[cryptography] passlib[bcrypt]
alembic upgrade head
python -m uvicorn main:app --reload
```

**Note**: Using `psycopg[binary]` (3.x) instead of `psycopg2-binary` for Python 3.13 compatibility.

✅ Done! Your databases are connected.

---

## 📋 Complete Environment File

Create `backend/.env`:

```env
# Neon Postgres
DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/neondb?sslmode=require

# Qdrant Cloud
QDRANT_URL=https://xxx.cloud.qdrant.io:6333
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION_NAME=textbook_content

# Gemini API
GEMINI_API_KEY=your-gemini-key
GEMINI_MODEL=gemini-2.5-flash

# Auth
SECRET_KEY=change-this-to-random-string
```

---

## ✅ Verification

Check logs when starting backend:
- ✅ Database connection initialized
- ✅ Collection textbook_content created

If you see errors, check `DATABASE_SETUP.md` for troubleshooting.

