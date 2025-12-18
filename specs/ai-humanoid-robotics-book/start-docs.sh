#!/bin/bash

# Script to start the Docusaurus documentation server

echo "Starting Docusaurus documentation server..."

# Navigate to the project directory
cd "$(dirname "$0")"

# Start the development server
npm run start

echo "Documentation server started. Visit http://localhost:3000"