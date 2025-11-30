#!/usr/bin/env node

/**
 * Script to inject API URL into api-config.js during build
 * This runs before Docusaurus build to replace the placeholder
 */

const fs = require('fs');
const path = require('path');

// Get API URL from environment variable
const apiUrl = process.env.REACT_APP_API_URL || process.env.API_URL || 'http://localhost:8000';

// Path to api-config.js
const configPath = path.join(__dirname, '..', 'static', 'api-config.js');

// Read the template
let configContent = fs.readFileSync(configPath, 'utf8');

// Replace the placeholder with actual API URL
configContent = configContent.replace(
  "var apiUrl = 'REACT_APP_API_URL_PLACEHOLDER';",
  `var apiUrl = '${apiUrl}';`
);

// Write back
fs.writeFileSync(configPath, configContent, 'utf8');

console.log(`✅ Injected API URL: ${apiUrl}`);

