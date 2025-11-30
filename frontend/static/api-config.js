// API Configuration - Injected at build time
// This file is automatically updated by scripts/inject-api-url.js during build
// The placeholder below will be replaced with the actual API URL from REACT_APP_API_URL env var
(function() {
  // This placeholder will be replaced by the build script
  var apiUrl = 'REACT_APP_API_URL_PLACEHOLDER';
  
  // Safety check: if placeholder wasn't replaced (shouldn't happen in production)
  if (apiUrl === 'REACT_APP_API_URL_PLACEHOLDER') {
    // Check if we're in production
    var isProduction = window.location.hostname !== 'localhost' && 
                       window.location.hostname !== '127.0.0.1';
    
    if (isProduction) {
      console.error('REACT_APP_API_URL not set in Vercel environment variables!');
      console.error('Please set REACT_APP_API_URL in Vercel project settings and redeploy.');
      apiUrl = null; // Will cause clear error
    } else {
      // Development fallback
      apiUrl = 'http://localhost:8000';
      console.log('Using development API URL:', apiUrl);
    }
  }
  
  // Inject into window for runtime access
  if (apiUrl) {
    window.__API_BASE_URL__ = apiUrl;
    console.log('API Base URL configured:', apiUrl);
  } else {
    console.error('API Base URL not configured. Please set REACT_APP_API_URL in Vercel.');
    window.__API_BASE_URL__ = 'API_URL_NOT_CONFIGURED';
  }
})();

