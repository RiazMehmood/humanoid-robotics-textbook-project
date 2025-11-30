#!/usr/bin/env node

/**
 * Script to inject API URL into api-config.js during build
 * This runs before Docusaurus build to replace the placeholder
 */

const fs = require('fs');
const path = require('path');

// Get API URL from environment variable
let apiUrl = process.env.REACT_APP_API_URL || process.env.API_URL || 'http://localhost:8000';

// Clean the URL - remove any quotes, backticks, or whitespace
apiUrl = String(apiUrl).trim();
// Remove backticks, single quotes, double quotes from start/end
apiUrl = apiUrl.replace(/^[`'"]+|[`'"]+$/g, '');

// Ensure URL has protocol
if (apiUrl && !apiUrl.startsWith('http://') && !apiUrl.startsWith('https://')) {
  apiUrl = 'https://' + apiUrl;
}

// Path to api-config.js
const configPath = path.join(__dirname, '..', 'static', 'api-config.js');

// Read the template
let configContent = fs.readFileSync(configPath, 'utf8');

// Replace the placeholder with actual API URL (properly escaped)
// Escape single quotes in the URL
const escapedApiUrl = apiUrl.replace(/'/g, "\\'");

configContent = configContent.replace(
  "var apiUrl = 'REACT_APP_API_URL_PLACEHOLDER';",
  `var apiUrl = '${escapedApiUrl}';`
);

// Write back
fs.writeFileSync(configPath, configContent, 'utf8');

console.log(`✅ Injected API URL: ${apiUrl}`);

