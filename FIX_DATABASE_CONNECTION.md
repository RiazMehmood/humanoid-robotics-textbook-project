# Fix Database Connection and "MagicMock" Error

## Error Analysis
```
Error: hash must be unicode or bytes, not unittest.mock.MagicMock
```
This error happens because your backend **failed to connect to the database** and fell back to a "Mock" database. When the code tries to check the user's password hash, it gets a "Mock" object instead of a real string, causing the crash.

## Critical Fix: Add DATABASE_URL to Railway

This is almost certainly missing in your Railway project.

1. **Go to Railway Dashboard:** https://railway.app/dashboard
2. **Select your Backend Service**
3. **Go to Variables Tab**
4. **Add `DATABASE_URL`:**
   - You need a PostgreSQL database.
   - If you created a Postgres service in Railway, click on it → "Connect" tab → Copy "Postgres Connection URL".
   - Paste that as `DATABASE_URL` in your **backend service variables**.
   - It should look like: `postgresql://postgres:password@roundhouse.proxy.rlwy.net:12345/railway`

## Verify Database Connection

After adding the variable and redeploying:

1. Check the deployment logs.
2. Look for: `✅ Database connection verified successfully`
3. If you see `❌ Database connection failed`, read the specific error message.

## Summary of Required Variables

Ensure ALL these are set in Railway:

- `DATABASE_URL` (Crucial! Missing this causes the MagicMock error)
- `CORS_ORIGINS`: `https://humanoid-robotics-textbook-project.vercel.app`
- `GEMINI_API_KEY`: (Your Google API key)
- `QDRANT_URL`: (Your Qdrant URL, or remove if not using RAG yet)
- `QDRANT_API_KEY`: (Your Qdrant Key)
- `SECRET_KEY`: (Any random long string)

