# Railway Setup Guide - Backend Deployment

## Problem: Railway Deploying Entire Repo Instead of Backend Only

If Railway is deploying the entire repository instead of just the `backend/` directory, follow these steps:

## Solution: Set Root Directory in Railway Settings

### Step-by-Step Instructions

1. **Create/Select Project in Railway**
   - Go to [railway.app](https://railway.app)
   - Click "New Project" → "Deploy from GitHub repo"
   - Select your repository: `humanoid-robotics-textbook-project`

2. **Add Service**
   - Click "New" → "GitHub Repo"
   - Select your repository
   - Railway will create a new service

3. **⚠️ CRITICAL: Set Root Directory**
   - Click on your service name (the one you just created)
   - Go to the **"Settings"** tab (gear icon)
   - Scroll down to find **"Root Directory"** field
   - Enter: `backend` (without quotes, without trailing slash)
   - Click **"Save"** or **"Update"**
   - Railway will now only use files from the `backend/` directory

4. **Verify Root Directory is Set**
   - After saving, you should see the root directory displayed in the settings
   - The service will restart/redeploy automatically
   - Check the build logs to confirm it's using `backend/` directory

5. **Configure Environment Variables**
   - Go to the **"Variables"** tab
   - Add all required environment variables:
     ```
     DATABASE_URL=postgresql+psycopg://...
     QDRANT_URL=https://your-cluster.qdrant.io
     QDRANT_API_KEY=your-key
     GEMINI_API_KEY=your-key
     SECRET_KEY=your-secure-key
     CORS_ORIGINS=https://your-frontend.vercel.app
     ENVIRONMENT=production
     DEBUG=false
     ```

6. **Deploy**
   - Railway will automatically detect Python from `backend/requirements.txt`
   - It will run: `uvicorn main:app --host 0.0.0.0 --port $PORT`
   - Check the **"Deployments"** tab to see the build progress

## Visual Guide

```
Railway Dashboard
├── Your Project
    └── Your Service
        ├── Settings Tab ← Click here
        │   └── Root Directory: backend ← Set this!
        ├── Variables Tab ← Add env vars here
        ├── Deployments Tab ← Check build status
        └── Metrics Tab ← Monitor after deployment
```

## Alternative: Using railway.json

If the Root Directory setting doesn't work, Railway should also respect the `railway.json` file in the `backend/` directory. We've already created this file at `backend/railway.json`.

However, **the Root Directory setting in the dashboard is the recommended approach** for monorepo deployments.

## Troubleshooting

### Issue: Still deploying entire repo
- **Solution**: Double-check the Root Directory setting is exactly `backend` (no slash, no quotes)
- **Solution**: Make sure you're in the Settings tab of the service, not the project

### Issue: Build fails with "main.py not found"
- **Solution**: Root Directory is not set correctly
- **Solution**: Verify `backend/main.py` exists in your repo

### Issue: Python dependencies not found
- **Solution**: Ensure `backend/requirements.txt` exists
- **Solution**: Railway should auto-detect Python from this file

### Issue: Port binding errors
- **Solution**: Railway sets `$PORT` automatically - don't hardcode port numbers
- **Solution**: Our start command uses `--port $PORT` which is correct

## Verification Checklist

- [ ] Root Directory set to `backend` in Railway Settings
- [ ] `backend/main.py` exists
- [ ] `backend/requirements.txt` exists
- [ ] `backend/railway.json` exists (optional but helpful)
- [ ] All environment variables added
- [ ] Build succeeds in Deployments tab
- [ ] Service is accessible via Railway-provided URL

## Next Steps

After Railway deployment succeeds:
1. Copy the Railway service URL (e.g., `https://your-app.up.railway.app`)
2. Use this URL in Vercel frontend deployment as `REACT_APP_API_URL`
3. Update Railway `CORS_ORIGINS` with your Vercel frontend URL

## Reference

- [Railway Monorepo Deployment Docs](https://docs.railway.com/tutorials/deploying-a-monorepo)
- [Railway Root Directory Setting](https://docs.railway.com/develop/configuration#root-directory)

