// API Configuration - Injected at build time
// This file is generated during build with the REACT_APP_API_URL environment variable
(function() {
  // Get API URL from environment variable (set during Vercel build)
  // Docusaurus will replace this during build if REACT_APP_API_URL is set
  var apiUrl = 'REACT_APP_API_URL_PLACEHOLDER';
  
  // If placeholder wasn't replaced, try to get from window location
  if (apiUrl === 'REACT_APP_API_URL_PLACEHOLDER') {
    // Check if we're in production
    var isProduction = window.location.hostname !== 'localhost' && 
                       window.location.hostname !== '127.0.0.1';
    
    if (isProduction) {
      console.error('REACT_APP_API_URL not set in Vercel environment variables!');
      apiUrl = null; // Will cause clear error
    } else {
      apiUrl = 'http://localhost:8000';
    }
  }
  
  // Inject into window for runtime access
  if (apiUrl) {
    window.__API_BASE_URL__ = apiUrl;
    console.log('API Base URL configured:', apiUrl);
  } else {
    console.error('API Base URL not configured. Please set REACT_APP_API_URL in Vercel.');
  }
})();

