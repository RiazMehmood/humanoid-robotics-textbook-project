# Fix CORS Error - Railway Backend

## Error Message
```
Access to fetch at 'https://humanoid-robotics-textbook-project-production.up.railway.app/auth/register' 
from origin 'https://humanoid-robotics-textbook-project.vercel.app' has been blocked by CORS policy: 
Response to preflight request doesn't pass access control check: 
No 'Access-Control-Allow-Origin' header is present on the requested resource.
```

## Solution: Update CORS_ORIGINS in Railway

### Step-by-Step Instructions

1. **Go to Railway Dashboard:**
   - Visit: https://railway.app/dashboard
   - Sign in if needed

2. **Select Your Service:**
   - Click on your backend service (the one you deployed)

3. **Go to Variables Tab:**
   - Click on **"Variables"** tab (or **"Environment Variables"**)

4. **Find or Add CORS_ORIGINS:**
   - Look for `CORS_ORIGINS` environment variable
   - If it exists, click to edit it
   - If it doesn't exist, click **"New Variable"**

5. **Set the Value:**
   - **Key**: `CORS_ORIGINS`
   - **Value**: `https://humanoid-robotics-textbook-project.vercel.app`
   - (Or if you want to keep localhost for testing: `https://humanoid-robotics-textbook-project.vercel.app,http://localhost:3000`)

6. **Save:**
   - Click **"Save"** or **"Update"**
   - Railway will automatically redeploy your service

7. **Wait for Redeployment:**
   - Go to **"Deployments"** tab
   - Wait for the latest deployment to complete (usually 1-2 minutes)
   - Check the logs to verify it started successfully

8. **Verify CORS is Working:**
   - Check Railway logs - you should see: `🌐 CORS allowed origins: ['https://humanoid-robotics-textbook-project.vercel.app', ...]`
   - Try signing up again from your Vercel frontend
   - CORS error should be gone

## Quick Checklist

- [ ] Go to Railway Dashboard
- [ ] Select your backend service
- [ ] Go to Variables tab
- [ ] Set `CORS_ORIGINS` = `https://humanoid-robotics-textbook-project.vercel.app`
- [ ] Save (Railway auto-redeploys)
- [ ] Wait for redeployment to complete
- [ ] Check logs for CORS origins confirmation
- [ ] Test sign up/sign in from Vercel frontend

## Verification

After updating CORS_ORIGINS, check Railway logs:

1. Go to Railway Dashboard → Your Service → Deployments
2. Click on the latest deployment
3. View Logs
4. Look for: `🌐 CORS allowed origins: [...]`
5. Should include: `https://humanoid-robotics-textbook-project.vercel.app`

## Still Not Working?

If CORS error persists after updating:

1. **Verify the exact URL:**
   - Make sure there are no typos
   - Make sure it's `https://` not `http://`
   - No trailing slash: `https://humanoid-robotics-textbook-project.vercel.app` (not `.app/`)

2. **Check Railway Logs:**
   - Look for the CORS origins log message
   - Verify it includes your Vercel URL

3. **Clear Browser Cache:**
   - Hard refresh: Ctrl+Shift+R (Windows) or Cmd+Shift+R (Mac)
   - Or clear browser cache

4. **Check Browser Console:**
   - Open Developer Tools (F12)
   - Network tab
   - Look at the OPTIONS request (preflight)
   - Check response headers for `Access-Control-Allow-Origin`

## Alternative: Temporary Allow All Origins (NOT RECOMMENDED FOR PRODUCTION)

If you need a quick test, you can temporarily allow all origins:

**In Railway Variables:**
- Set `CORS_ORIGINS` = `*`

**⚠️ WARNING:** This is insecure for production! Only use for testing. Change back to your specific Vercel URL after testing.

