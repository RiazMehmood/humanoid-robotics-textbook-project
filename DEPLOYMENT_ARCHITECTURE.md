# Deployment Architecture for Fully Automated System

## Recommended Architecture

### Frontend: Vercel (Recommended) or GitHub Pages
- **Vercel**: Better for this use case (automatic deployments, environment variables, better integration)
- **GitHub Pages**: Free but static only, requires manual builds

### Backend: Railway or Render (Recommended)
Both support FastAPI/uvicorn with continuous running and automated deployments.

---

## Option 1: Railway (Easiest & Recommended) ⭐

### Why Railway?
- ✅ **Fully automated** - Connect GitHub repo, auto-deploys on push
- ✅ **Free tier** - $5 credit/month (enough for small projects)
- ✅ **Easy FastAPI setup** - Detects Python automatically
- ✅ **Environment variables** - Easy UI management
- ✅ **Custom domain** - Free subdomain + custom domain support
- ✅ **Database integration** - Can provision Postgres (but you already have Neon)

### Setup Steps:
1. **Backend Deployment**:
   - Go to [railway.app](https://railway.app)
   - Connect GitHub repository
   - Select `backend/` directory
   - Railway auto-detects Python and installs dependencies
   - Add environment variables (see below)
   - Railway provides a URL like: `https://your-app.railway.app`

2. **Frontend Deployment** (Vercel):
   - Go to [vercel.com](https://vercel.com)
   - Connect GitHub repository
   - Select `frontend/` directory
   - Set build command: `npm run build`
   - Set output directory: `build`
   - Add environment variables (see below)
   - Vercel provides URL like: `https://your-app.vercel.app`

---

## Option 2: Render (Free Tier Available)

### Why Render?
- ✅ **Free tier** - 750 hours/month (enough for always-on service)
- ✅ **Fully automated** - GitHub integration
- ✅ **Easy setup** - Web service template for FastAPI
- ⚠️ **Spins down** - Free tier spins down after 15min inactivity (wakes on request)

### Setup Steps:
1. **Backend Deployment**:
   - Go to [render.com](https://render.com)
   - Create new "Web Service"
   - Connect GitHub repository
   - Select `backend/` directory
   - Build command: `pip install -r requirements.txt`
   - Start command: `uvicorn main:app --host 0.0.0.0 --port $PORT`
   - Add environment variables
   - Render provides URL like: `https://your-app.onrender.com`

2. **Frontend Deployment** (Vercel or GitHub Pages):
   - Same as Option 1

---

## Option 3: Fly.io (Global Performance)

### Why Fly.io?
- ✅ **Global edge** - Deploys close to users
- ✅ **Free tier** - 3 shared VMs
- ✅ **FastAPI support** - Good documentation
- ⚠️ **More complex** - Requires `fly.toml` configuration

---

## Environment Variables Setup

### Backend (Railway/Render/Fly.io):

```bash
# Database
DATABASE_URL=postgresql+psycopg://user:password@neon-host/dbname

# Vector DB
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook_content

# Authentication
SECRET_KEY=your-very-secure-secret-key-here
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=7

# AI/LLM
GEMINI_API_KEY=your-gemini-api-key
GEMINI_MODEL=gemini-2.5-flash
GEMINI_EMBEDDING_MODEL=models/text-embedding-004

# CORS - IMPORTANT: Add your frontend URL
CORS_ORIGINS=https://your-frontend.vercel.app,https://your-frontend.pages.dev

# Environment
ENVIRONMENT=production
DEBUG=false
```

### Frontend (Vercel):

```bash
# API URL - Point to your backend
REACT_APP_API_URL=https://your-backend.railway.app
# or
REACT_APP_API_URL=https://your-backend.onrender.com
```

---

## Recommended: Railway + Vercel Setup

### Step-by-Step:

#### 1. Backend on Railway:
```bash
# In Railway dashboard:
1. New Project → Deploy from GitHub
2. Select your repository
3. Add Service → Select backend/ directory
4. Railway auto-detects Python
5. Add all environment variables (see above)
6. Deploy → Get URL: https://your-backend.railway.app
```

#### 2. Frontend on Vercel:
```bash
# In Vercel dashboard:
1. New Project → Import from GitHub
2. Select your repository
3. Root Directory: frontend
4. Framework Preset: Docusaurus
5. Build Command: npm run build
6. Output Directory: build
7. Environment Variables:
   - REACT_APP_API_URL=https://your-backend.railway.app
8. Deploy → Get URL: https://your-frontend.vercel.app
```

#### 3. Update Backend CORS:
```bash
# In Railway environment variables:
CORS_ORIGINS=https://your-frontend.vercel.app
```

#### 4. Update Frontend Config:
```typescript
// frontend/docusaurus.config.ts
url: 'https://your-frontend.vercel.app',
baseUrl: '/',
```

---

## Fully Automated Workflow

### What Happens:
1. **Push to GitHub** → Triggers both deployments
2. **Railway** → Auto-builds and deploys backend
3. **Vercel** → Auto-builds and deploys frontend
4. **Both update** → New code live in minutes

### No Manual Steps:
- ✅ No SSH needed
- ✅ No server management
- ✅ No manual deployments
- ✅ Environment variables managed in UI
- ✅ Automatic HTTPS certificates
- ✅ Automatic scaling

---

## Cost Comparison

| Platform | Free Tier | Paid Tier |
|----------|-----------|-----------|
| **Railway** | $5 credit/month | $0.013/GB RAM-hour |
| **Render** | 750 hours/month | $7/month (always-on) |
| **Fly.io** | 3 shared VMs | $1.94/month per VM |
| **Vercel** | Unlimited (hobby) | Free for personal projects |
| **GitHub Pages** | Free | Free |

**Recommendation**: Railway (backend) + Vercel (frontend) = **$0/month** for small projects

---

## Next Steps

1. **Choose Railway for backend** (easiest setup)
2. **Choose Vercel for frontend** (best integration)
3. **Set up environment variables** in both platforms
4. **Update CORS** in backend to allow frontend domain
5. **Test deployment** - Push to GitHub and watch it deploy!

---

## Files to Create/Update

### 1. Railway Configuration (Optional - Railway auto-detects):
```python
# backend/railway.json (optional)
{
  "build": {
    "builder": "NIXPACKS"
  },
  "deploy": {
    "startCommand": "uvicorn main:app --host 0.0.0.0 --port $PORT"
  }
}
```

### 2. Vercel Configuration:
```json
// frontend/vercel.json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": null,
  "rewrites": [
    { "source": "/(.*)", "destination": "/index.html" }
  ]
}
```

### 3. Update Frontend API URL:
```typescript
// frontend/src/services/auth_api.ts
// Already uses REACT_APP_API_URL from environment
```

---

## Troubleshooting

### Backend not accessible:
- Check CORS_ORIGINS includes frontend URL
- Check Railway/Render logs
- Verify PORT environment variable (Railway/Render sets automatically)

### Frontend can't reach backend:
- Verify REACT_APP_API_URL is set correctly
- Check browser console for CORS errors
- Verify backend is running (check Railway/Render dashboard)

### Database connection issues:
- Verify DATABASE_URL is correct (Neon connection string)
- Check if database allows connections from Railway/Render IPs
- Neon allows all IPs by default, so should work

---

## Summary

**Best Choice**: **Railway (Backend) + Vercel (Frontend)**
- ✅ Fully automated
- ✅ Free for small projects
- ✅ Easy setup
- ✅ No server management
- ✅ Automatic HTTPS
- ✅ GitHub integration

**Alternative**: **Render (Backend) + Vercel (Frontend)**
- ✅ Free tier available
- ⚠️ Spins down on free tier (wakes on request)

