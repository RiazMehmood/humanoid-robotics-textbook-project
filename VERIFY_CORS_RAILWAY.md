# Verify and Fix CORS in Railway

## Current Error
```
Access to fetch at 'https://humanoid-robotics-textbook-project-production.up.railway.app/auth/register' 
from origin 'https://humanoid-robotics-textbook-project.vercel.app' has been blocked by CORS policy: 
Response to preflight request doesn't pass access control check: 
No 'Access-Control-Allow-Origin' header is present on the requested resource.
```

## Step 1: Set CORS_ORIGINS in Railway

1. **Go to Railway Dashboard:**
   - Visit: https://railway.app/dashboard
   - Sign in if needed

2. **Select Your Backend Service:**
   - Click on your backend service (the one deployed)

3. **Go to Variables Tab:**
   - Click on **"Variables"** tab (or **"Environment Variables"**)

4. **Add/Update CORS_ORIGINS:**
   - **Key**: `CORS_ORIGINS`
   - **Value**: `https://humanoid-robotics-textbook-project.vercel.app`
   - (Or if you want to keep localhost: `https://humanoid-robotics-textbook-project.vercel.app,http://localhost:3000`)

5. **Save:**
   - Click **"Save"** or **"Update"**
   - Railway will automatically redeploy your service

6. **Wait for Redeployment:**
   - Go to **"Deployments"** tab
   - Wait for the latest deployment to complete (usually 1-2 minutes)
   - Check the logs to verify it started successfully

## Step 2: Verify CORS Configuration

After Railway redeploys, test the CORS configuration:

1. **Check Railway Logs:**
   - Go to Railway Dashboard → Your Service → Deployments
   - Click on the latest deployment
   - View Logs
   - Look for: `🌐 CORS allowed origins: [...]`
   - Should include: `https://humanoid-robotics-textbook-project.vercel.app`

2. **Test CORS Info Endpoint:**
   - Open browser and go to:
     ```
     https://humanoid-robotics-textbook-project-production.up.railway.app/cors-info
     ```
   - You should see JSON with CORS configuration
   - Verify `cors_origins` includes your Vercel URL

3. **Test Health Endpoint:**
   - Go to:
     ```
     https://humanoid-robotics-textbook-project-production.up.railway.app/health
     ```
   - Should return: `{"status": "healthy"}`

## Step 3: Test from Browser Console

Open your Vercel frontend and run this in the browser console:

```javascript
fetch('https://humanoid-robotics-textbook-project-production.up.railway.app/health', {
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
  },
})
  .then(response => response.json())
  .then(data => console.log('✅ CORS working!', data))
  .catch(error => console.error('❌ CORS error:', error));
```

If this works, CORS is configured correctly!

## Step 4: Test Auth Endpoint

Try signing up again from your Vercel frontend. The CORS error should be gone.

## Troubleshooting

### Still Getting CORS Error?

1. **Verify the exact URL:**
   - Make sure there are no typos in `CORS_ORIGINS`
   - Make sure it's `https://` not `http://`
   - No trailing slash: `https://humanoid-robotics-textbook-project.vercel.app` (not `.app/`)

2. **Check Railway Logs:**
   - Look for the CORS origins log message
   - Verify it includes your Vercel URL
   - Check for any errors during startup

3. **Clear Browser Cache:**
   - Hard refresh: Ctrl+Shift+R (Windows) or Cmd+Shift+R (Mac)
   - Or clear browser cache completely

4. **Check Browser Console:**
   - Open Developer Tools (F12)
   - Network tab
   - Look at the OPTIONS request (preflight)
   - Check response headers for `Access-Control-Allow-Origin`
   - Should show: `Access-Control-Allow-Origin: https://humanoid-robotics-textbook-project.vercel.app`

5. **Verify Railway Redeployed:**
   - Check the deployment timestamp
   - Make sure it's after you updated `CORS_ORIGINS`
   - If not, manually trigger a redeployment

### Railway Not Redeploying?

If Railway doesn't auto-redeploy after updating environment variables:

1. Go to Railway Dashboard → Your Service
2. Click **"Deployments"** tab
3. Click **"Redeploy"** button (or **"Deploy"** if available)
4. Wait for deployment to complete

### Still Not Working?

If CORS is still not working after following all steps:

1. **Check if Railway service is running:**
   - Go to Railway Dashboard → Your Service
   - Check if the service is "Active" and "Running"
   - Check logs for any errors

2. **Verify environment variable is set:**
   - Go to Variables tab
   - Confirm `CORS_ORIGINS` is exactly: `https://humanoid-robotics-textbook-project.vercel.app`
   - No extra spaces or quotes

3. **Try temporary wildcard (TESTING ONLY):**
   - Set `CORS_ORIGINS` = `*` (allows all origins)
   - **⚠️ WARNING:** This is insecure! Only use for testing
   - If this works, the issue is with the specific URL format
   - Change back to your specific Vercel URL after testing

## Quick Checklist

- [ ] Set `CORS_ORIGINS` in Railway Variables
- [ ] Value is exactly: `https://humanoid-robotics-textbook-project.vercel.app`
- [ ] Railway redeployed successfully
- [ ] Checked Railway logs for CORS origins confirmation
- [ ] Tested `/cors-info` endpoint
- [ ] Tested `/health` endpoint
- [ ] Tested from browser console
- [ ] Tested sign up from Vercel frontend
- [ ] CORS error is resolved

