# Database Setup Guide

This guide will help you set up the required databases for the textbook project.

## Required Databases

1. **Neon Serverless Postgres** - For user authentication and data
2. **Qdrant Cloud Free Tier** - For RAG chatbot vector storage

---

## Step 1: Set Up Neon Serverless Postgres

### 1.1 Create Neon Account
1. Go to https://neon.tech
2. Sign up for a free account
3. Create a new project

### 1.2 Get Connection String
1. In your Neon dashboard, go to your project
2. Click on "Connection Details" or "Connection String"
3. Copy the connection string (it will look like):
   ```
   postgresql://username:password@ep-xxx-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

### 1.3 Add to Backend Environment
1. Create/update `backend/.env` file:
   ```env
   DATABASE_URL=postgresql://username:password@ep-xxx-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

### 1.4 Install Database Dependencies
```bash
cd backend
pip install sqlalchemy==2.0.23 psycopg2-binary==2.9.9 alembic==1.13.1 python-jose[cryptography]==3.3.0 passlib[bcrypt]==1.7.4
```

### 1.5 Run Database Migrations
```bash
cd backend
alembic upgrade head
```

This will create all the necessary tables (users, auth_tokens, chatbot_queries, chatbot_responses).

---

## Step 2: Set Up Qdrant Cloud Free Tier

### 2.1 Create Qdrant Cloud Account
1. Go to https://cloud.qdrant.io
2. Sign up for a free account
3. Create a new cluster (Free tier is available)

### 2.2 Get API Key and URL
1. In your Qdrant Cloud dashboard, go to your cluster
2. Copy the **Cluster URL** (e.g., `https://xxxxx-xxxxx.us-east-1-0.aws.cloud.qdrant.io:6333`)
3. Go to "API Keys" section and create a new API key
4. Copy the API key

### 2.3 Add to Backend Environment
Update `backend/.env` file:
```env
QDRANT_URL=https://xxxxx-xxxxx.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-api-key-here
QDRANT_COLLECTION_NAME=textbook_content
```

### 2.4 Test Connection
The collection will be automatically created when the backend starts. Check the logs to confirm:
```
✅ Collection textbook_content created successfully
```

---

## Step 3: Complete Environment File

Create `backend/.env` with all required variables:

```env
# Neon Postgres Database
DATABASE_URL=postgresql://username:password@ep-xxx-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# Qdrant Cloud Vector Database
QDRANT_URL=https://xxxxx-xxxxx.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=textbook_content

# Google Gemini API (for chatbot and translation)
GEMINI_API_KEY=your-gemini-api-key-here
GEMINI_MODEL=gemini-2.5-flash
GEMINI_EMBEDDING_MODEL=models/text-embedding-004

# Authentication
SECRET_KEY=your-secret-key-change-this-in-production
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=7

# Application
ENVIRONMENT=production
DEBUG=false
CORS_ORIGINS=http://localhost:3000,http://localhost:8000
```

---

## Step 4: Verify Setup

### 4.1 Test Database Connection
```bash
cd backend
python -c "from src.database.base import init_database; init_database(); print('✅ Database connected')"
```

### 4.2 Test Qdrant Connection
```bash
cd backend
python -c "from src.database.vector_db import get_vector_db; db = get_vector_db(); print('✅ Qdrant connected')"
```

### 4.3 Start Backend Server
```bash
cd backend
python -m uvicorn main:app --reload
```

Check the logs for:
- ✅ Database connection initialized
- ✅ Collection textbook_content created successfully (or already exists)

---

## Troubleshooting

### Database Connection Issues
- **Error: "connection refused"**
  - Check your DATABASE_URL is correct
  - Ensure Neon project is active
  - Verify SSL mode is set to `require`

### Qdrant Connection Issues
- **Error: "connection refused"**
  - Check QDRANT_URL includes the port (6333)
  - Verify QDRANT_API_KEY is correct
  - Ensure cluster is running in Qdrant Cloud dashboard

### Migration Issues
- **Error: "No module named 'alembic'"**
  - Run: `pip install alembic==1.13.1`
- **Error: "No module named 'psycopg2'" or psycopg installation issues**
  - Run: `pip install "psycopg[binary]"` (uses psycopg 3.x with binary for Python 3.13)
  - The `[binary]` extra includes the compiled C extension needed for Windows

---

## Next Steps

After setting up databases:
1. ✅ User registration/signin will work
2. ✅ Chatbot will store and retrieve from Qdrant
3. ✅ User preferences will be saved to Neon Postgres
4. ✅ All personalization features will be functional

