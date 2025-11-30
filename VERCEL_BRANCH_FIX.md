# Vercel Branch Selection Fix

## Problem: Vercel Showing Wrong Branch

When importing your repository to Vercel, it might default to `feature/constitution-update` or another branch instead of `master`.

## Solution: Select Master Branch During Import

### Step-by-Step Fix

1. **In Vercel Import Screen**:
   - After selecting your repository (`humanoid-robotics-textbook-project`)
   - Look for the **"Branch"** dropdown/selector
   - It might be labeled as:
     - "Branch"
     - "Git Branch"
     - "Production Branch"
     - Or shown as a dropdown next to the repository name

2. **Select Master Branch**:
   - Click on the branch dropdown
   - Select `master` from the list
   - This is the branch that contains all your merged code

3. **Continue Configuration**:
   - Set Root Directory to `frontend`
   - Configure other settings
   - Add environment variables
   - Deploy

### Visual Guide

```
Vercel Import Screen:
┌─────────────────────────────────────┐
│ Repository: humanoid-robotics...    │
│ Branch: [feature/constitution-update] ▼ ← Click here!
│                                     │
│   Select: master                    │ ← Choose this
│                                     │
│ Root Directory: [frontend]          │
│ Framework: Docusaurus              │
└─────────────────────────────────────┘
```

### Alternative: Change Branch After Project Creation

If you've already created the project with the wrong branch:

1. **Go to Project Settings**:
   - In Vercel dashboard, click on your project
   - Go to **"Settings"** tab
   - Click **"Git"** section

2. **Change Production Branch**:
   - Find **"Production Branch"** setting
   - Change it from `feature/constitution-update` to `master`
   - Click **"Save"**

3. **Redeploy**:
   - Go to **"Deployments"** tab
   - Click **"Redeploy"** on the latest deployment
   - Or push a new commit to `master` branch to trigger auto-deploy

### Verify Correct Branch

After deployment, verify you're using the correct branch:

1. Go to **"Deployments"** tab
2. Click on the latest deployment
3. Check the **"Source"** or **"Git Commit"** section
4. It should show commits from `master` branch, not `feature/constitution-update`

### Why This Matters

- `master` branch contains all merged code (backend + frontend + all features)
- `feature/constitution-update` is an old feature branch that was merged
- Using `master` ensures you have the latest, complete codebase

### Quick Checklist

- [ ] Selected `master` branch during Vercel import
- [ ] Root Directory set to `frontend`
- [ ] Environment variable `REACT_APP_API_URL` added with Railway URL
- [ ] Deployment successful
- [ ] Verified deployment is using `master` branch commits

## Still Having Issues?

If Vercel still shows the wrong branch:

1. **Check GitHub Default Branch**:
   - Go to your GitHub repository
   - Settings → Branches
   - Ensure `master` is set as the default branch

2. **Delete and Recreate Vercel Project**:
   - Delete the current Vercel project
   - Create a new one
   - This time, explicitly select `master` branch

3. **Manual Branch Selection**:
   - In Vercel, you can also manually specify the branch in the import URL or settings

