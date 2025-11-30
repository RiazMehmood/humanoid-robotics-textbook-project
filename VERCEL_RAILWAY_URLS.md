# Production URLs Configuration

## Deployed URLs

### Frontend (Vercel)
- **URL**: https://humanoid-robotics-textbook-project.vercel.app/

### Backend (Railway)
- **URL**: https://humanoid-robotics-textbook-project-production.up.railway.app

## Required Configuration

### 1. Vercel Environment Variables

Go to: Vercel Dashboard → Your Project → Settings → Environment Variables

**Required Variable:**
- **Key**: `REACT_APP_API_URL`
- **Value**: `https://humanoid-robotics-textbook-project-production.up.railway.app`
- **Environments**: Production, Preview, Development (all)

### 2. Railway Environment Variables

Go to: Railway Dashboard → Your Service → Variables

**Required Variable:**
- **Key**: `CORS_ORIGINS`
- **Value**: `https://humanoid-robotics-textbook-project.vercel.app`
- (Or if you have multiple: `https://humanoid-robotics-textbook-project.vercel.app,http://localhost:3000`)

**Other Required Variables:**
- `DATABASE_URL` - Your Neon Postgres connection string
- `QDRANT_URL` - Your Qdrant Cloud URL
- `QDRANT_API_KEY` - Your Qdrant API key
- `GEMINI_API_KEY` - Your Gemini API key
- `SECRET_KEY` - A secure random key for JWT tokens
- `ENVIRONMENT=production`
- `DEBUG=false`

## Verification Steps

1. **Check Vercel Environment Variable:**
   - Settings → Environment Variables
   - Verify `REACT_APP_API_URL` = `https://humanoid-robotics-textbook-project-production.up.railway.app`

2. **Check Railway Environment Variable:**
   - Variables tab
   - Verify `CORS_ORIGINS` = `https://humanoid-robotics-textbook-project.vercel.app`

3. **Test Backend:**
   - Visit: https://humanoid-robotics-textbook-project-production.up.railway.app/docs
   - Should see FastAPI Swagger UI

4. **Test Frontend:**
   - Visit: https://humanoid-robotics-textbook-project.vercel.app
   - Open browser console (F12)
   - Should see: `API Base URL configured: https://humanoid-robotics-textbook-project-production.up.railway.app`

5. **Test Sign Up:**
   - Try to sign up
   - Should work without CORS errors
   - Should connect to Railway backend

## Troubleshooting

### If API URL not configured error:
- Verify `REACT_APP_API_URL` is set in Vercel
- Redeploy Vercel project after setting the variable
- Check build logs to see if the variable is being read

### If CORS errors:
- Verify `CORS_ORIGINS` in Railway includes the Vercel URL
- Make sure Railway has redeployed after updating CORS_ORIGINS
- Check Railway logs for CORS-related errors

### If 405 Method Not Allowed:
- Verify the Railway URL is correct (with https://)
- Check that the backend is running (visit /docs endpoint)
- Verify the endpoint path is correct (/auth/register, not /api/v1/auth/register)

