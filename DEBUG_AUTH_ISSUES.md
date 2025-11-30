# Debug Sign Up / Sign In Issues After Deployment

## Common Issues and Solutions

### Issue 1: CORS Errors (Most Common)

**Symptoms:**
- Browser console shows: `CORS policy: No 'Access-Control-Allow-Origin' header`
- Network tab shows OPTIONS request failing
- Sign up/Sign in requests blocked

**Solution:**
1. **Get your Vercel frontend URL** (e.g., `https://your-app.vercel.app`)
2. **Go to Railway Dashboard** → Your Service → Variables
3. **Update `CORS_ORIGINS`** environment variable:
   ```
   https://your-app.vercel.app
   ```
   (Or if you have multiple: `https://your-app.vercel.app,http://localhost:3000`)
4. **Save** - Railway will auto-redeploy
5. **Wait for redeployment** (check Railway Deployments tab)
6. **Test again**

### Issue 2: Environment Variable Not Set

**Symptoms:**
- Frontend trying to connect to `http://localhost:8000` instead of Railway URL
- Network requests going to wrong URL
- Console shows localhost URLs

**Solution:**
1. **Go to Vercel Dashboard** → Your Project → Settings → Environment Variables
2. **Verify `REACT_APP_API_URL` exists**:
   - Key: `REACT_APP_API_URL`
   - Value: `https://your-railway-app.up.railway.app` (your actual Railway URL)
   - Environments: All (Production, Preview, Development)
3. **If missing, add it** (see instructions below)
4. **Redeploy** Vercel project:
   - Go to Deployments tab
   - Click three dots (⋯) on latest deployment
   - Click "Redeploy"

### Issue 3: Docusaurus Environment Variables

**Important:** Docusaurus builds at build time, so environment variables need special handling.

**Check if variable is accessible:**
1. Open browser console on your deployed site
2. Type: `process.env.REACT_APP_API_URL`
3. If it shows `undefined`, the variable isn't being injected

**Solution:**
Docusaurus may need the variable in a different format. Check:
- Vercel environment variables are set correctly
- Project was redeployed after adding the variable
- Variable name is exactly `REACT_APP_API_URL` (case-sensitive)

### Issue 4: Backend Not Accessible

**Symptoms:**
- Network requests timing out
- 502/503 errors
- Connection refused errors

**Solution:**
1. **Test backend directly:**
   - Visit: `https://your-railway-app.up.railway.app/docs`
   - Should show FastAPI Swagger UI
   - If not accessible, Railway service might be down
2. **Check Railway logs:**
   - Railway Dashboard → Your Service → Deployments
   - Click latest deployment → View Logs
   - Look for errors
3. **Verify Railway service is running:**
   - Check Railway dashboard shows service as "Active"
   - Check if service needs to be restarted

### Issue 5: Database Connection Issues

**Symptoms:**
- Sign up fails with database errors
- Backend logs show connection errors
- 500 errors on auth endpoints

**Solution:**
1. **Check Railway environment variables:**
   - `DATABASE_URL` is set correctly (Neon Postgres connection string)
   - Connection string format: `postgresql+psycopg://user:password@host/dbname`
2. **Verify database is accessible:**
   - Check Neon dashboard - database should be active
   - Test connection from Railway logs
3. **Check migrations:**
   - Railway should run migrations automatically
   - Check logs for migration errors

## Step-by-Step Debugging

### Step 1: Check Browser Console

1. Open your deployed frontend
2. Open Developer Tools (F12)
3. Go to Console tab
4. Try to sign up/sign in
5. Look for errors:
   - CORS errors
   - Network errors
   - API URL errors

### Step 2: Check Network Tab

1. Open Developer Tools (F12)
2. Go to Network tab
3. Try to sign up/sign in
4. Look for the request:
   - What URL is it trying to hit?
   - What's the response status?
   - What's the response body?

### Step 3: Verify Environment Variables

**In Vercel:**
- Settings → Environment Variables
- `REACT_APP_API_URL` = `https://your-railway-app.up.railway.app`

**In Railway:**
- Variables tab
- `CORS_ORIGINS` = `https://your-vercel-app.vercel.app`
- `DATABASE_URL` = (your Neon connection string)
- `SECRET_KEY` = (set)
- `GEMINI_API_KEY` = (set)
- `QDRANT_URL` = (set)
- `QDRANT_API_KEY` = (set)

### Step 4: Test Backend Directly

1. Visit: `https://your-railway-app.up.railway.app/docs`
2. Should see FastAPI Swagger UI
3. Try the `/auth/register` endpoint:
   - Click "Try it out"
   - Enter test data
   - Click "Execute"
   - Check response

### Step 5: Check Railway Logs

1. Railway Dashboard → Your Service
2. Go to Deployments tab
3. Click latest deployment
4. View Logs
5. Look for:
   - Startup errors
   - Database connection errors
   - API request errors

## Quick Fix Checklist

- [ ] CORS_ORIGINS in Railway includes Vercel frontend URL
- [ ] REACT_APP_API_URL in Vercel is set to Railway backend URL
- [ ] Both services have been redeployed after env var changes
- [ ] Backend is accessible at Railway URL (test /docs endpoint)
- [ ] Database connection is working (check Railway logs)
- [ ] No CORS errors in browser console
- [ ] Network requests show correct backend URL

## Still Not Working?

If issues persist:

1. **Check exact error message** in browser console
2. **Check Railway logs** for backend errors
3. **Verify URLs** are correct (no typos, correct protocol https://)
4. **Test backend API directly** using Swagger UI
5. **Check if services are running** (Railway shows Active, Vercel shows Ready)

## Common Error Messages

### "Failed to fetch"
- **Cause:** CORS issue or backend not accessible
- **Fix:** Update CORS_ORIGINS in Railway

### "Network request failed"
- **Cause:** Backend URL incorrect or backend down
- **Fix:** Verify REACT_APP_API_URL and check Railway service status

### "User already exists"
- **Cause:** User already registered
- **Fix:** Try different email or sign in instead

### "Invalid credentials"
- **Cause:** Wrong password or user doesn't exist
- **Fix:** Verify credentials or sign up first

### "Database connection error"
- **Cause:** DATABASE_URL incorrect or database down
- **Fix:** Check DATABASE_URL in Railway and Neon database status

