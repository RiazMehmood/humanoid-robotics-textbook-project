# Quick Start: Deploy to Production

## 🚀 Recommended: Railway (Backend) + Vercel (Frontend)

### Step 1: Deploy Backend to Railway (5 minutes)

1. **Sign up**: Go to [railway.app](https://railway.app) and sign up with GitHub
2. **New Project**: Click "New Project" → "Deploy from GitHub repo"
3. **Select Repo**: Choose your repository (`humanoid-robotics-textbook-project`)
4. **Add Service**: Click "New" → "GitHub Repo" → Select your repository
5. **⚠️ IMPORTANT - Set Root Directory**:
   - After adding the service, click on the service name
   - Go to **Settings** tab
   - Find **"Root Directory"** option
   - Set it to: `backend` (without the slash)
   - Click **Save**
   - This tells Railway to only deploy the `backend/` folder
6. **Railway auto-detects**: It will detect Python and FastAPI automatically from the `backend/` directory
7. **Add Environment Variables**:
   - Click on your service → "Variables" tab
   - Add these (get values from your `.env` file):
     ```
     DATABASE_URL=postgresql+psycopg://... (your Neon connection string)
     QDRANT_URL=https://your-cluster.qdrant.io
     QDRANT_API_KEY=your-qdrant-key
     GEMINI_API_KEY=your-gemini-key
     SECRET_KEY=generate-a-secure-random-key
     CORS_ORIGINS=https://your-frontend.vercel.app (add after frontend deploys)
     ENVIRONMENT=production
     DEBUG=false
     ```
7. **Deploy**: Railway automatically deploys and gives you a URL like:
   ```
   https://your-backend-production.up.railway.app
   ```

### Step 2: Deploy Frontend to Vercel (5 minutes)

1. **Sign up**: Go to [vercel.com](https://vercel.com) and sign up with GitHub
2. **New Project**: Click "Add New" → "Project"
3. **Import**: Select your GitHub repository
4. **Configure**:
   - **Root Directory**: `frontend`
   - **Framework Preset**: Docusaurus (auto-detected)
   - **Build Command**: `npm run build` (auto-filled)
   - **Output Directory**: `build` (auto-filled)
5. **Environment Variables**:
   - Add: `REACT_APP_API_URL` = `humanoid-robotics-textbook-project-production.up.railway.app`
6. **Deploy**: Click "Deploy" → Vercel gives you a URL like:
   ```
   https://your-frontend.vercel.app
   ```

### Step 3: Update Backend CORS (2 minutes)

1. Go back to Railway dashboard
2. Update `CORS_ORIGINS` environment variable:
   ```
   https://your-frontend.vercel.app
   ```
3. Railway automatically redeploys

### Step 4: Update Frontend Config (2 minutes)

1. Update `frontend/docusaurus.config.ts`:
   ```typescript
   url: 'https://your-frontend.vercel.app',
   baseUrl: '/',
   ```
2. Commit and push to GitHub
3. Vercel automatically redeploys

### ✅ Done! Your app is live and fully automated!

---

## 🔄 Automated Workflow

**What happens now:**
- Push to GitHub → Both Railway and Vercel auto-deploy
- No manual steps needed
- Environment variables managed in dashboards
- Automatic HTTPS certificates
- Automatic scaling

---

## 📋 Checklist

- [ ] Backend deployed on Railway
- [ ] Frontend deployed on Vercel
- [ ] Environment variables set in Railway
- [ ] Environment variables set in Vercel
- [ ] CORS_ORIGINS updated with frontend URL
- [ ] Frontend config updated with production URL
- [ ] Test: Visit frontend URL and verify it connects to backend
- [ ] Test: Sign up/Sign in works
- [ ] Test: Chatbot works

---

## 🆘 Troubleshooting

### Backend not accessible:
- Check Railway logs: Service → "Deployments" → Click latest → "View Logs"
- Verify all environment variables are set
- Check CORS_ORIGINS includes frontend URL

### Frontend can't reach backend:
- Open browser console (F12)
- Check for CORS errors
- Verify `REACT_APP_API_URL` is set correctly in Vercel
- Verify backend URL is accessible (try in browser)

### Database connection fails:
- Verify `DATABASE_URL` is correct (Neon connection string)
- Check Neon dashboard - database should be active
- Neon allows all IPs by default, so Railway should work

---

## 💰 Cost

**Free tier covers:**
- Railway: $5 credit/month (enough for small projects)
- Vercel: Unlimited for personal projects
- **Total: $0/month** for small projects

---

## 🔗 Alternative: Render (Free Tier)

If you prefer Render for backend:

1. Go to [render.com](https://render.com)
2. New → Web Service
3. Connect GitHub repo
4. Settings:
   - **Root Directory**: `backend`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables (same as Railway)
6. Deploy

**Note**: Render free tier spins down after 15min inactivity (wakes on first request)

