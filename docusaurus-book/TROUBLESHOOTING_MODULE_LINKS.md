# Troubleshooting Guide: Resolving 404 Errors on Module Links

## Problem
When clicking on module links in your Docusaurus documentation, you're getting a "Page Not Found" error.

## Root Cause Analysis
The most likely causes are:
1. Docusaurus build cache issues
2. Incorrect file path resolution in sidebar configuration
3. Missing or incorrect frontmatter in documentation files

## Solution Steps

### Step 1: Clean Build Cache
```bash
cd docusaurus-book
rm -rf build/
rm -rf .docusaurus/
rm -rf node_modules/.cache/
```

### Step 2: Reinstall Dependencies
```bash
npm install
```

### Step 3: Verify Documentation Structure
Ensure these files exist with proper content:
- `docs/modules/module-1-ros-nervous-system/index.md`
- `docs/modules/module-2-digital-twin/index.md`
- `docs/modules/module-3-ai-brain/index.md`
- `docs/modules/module-4-vla/index.md`

### Step 4: Fix Sidebar Configuration (Critical)
Edit `docusaurus-book/sidebars.js` to ensure proper structure:

```javascript
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'modules/module-1-ros-nervous-system/index',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'modules/module-2-digital-twin/index',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/module-3-ai-brain/index',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'modules/module-4-vla/index',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-project/pipeline-overview',
      ],
    },
  ],
};

module.exports = sidebars;
```

### Step 5: Verify Frontmatter Format
Ensure each index.md file has proper frontmatter:
```markdown
---
id: modules/module-1-ros-nervous-system/index
title: Module 1 - The Robotic Nervous System (ROS 2)
sidebar_label: Module 1 - The Robotic Nervous System (ROS 2)
---
```

### Step 6: Restart Development Server
```bash
npm start
```

## Additional Debugging Steps

1. **Check browser console** for specific error messages
2. **Try direct URL access** to see if pages load directly
3. **Verify file permissions** in the docs directory
4. **Check if all directories have proper read permissions**

## If Issue Persists

If you're still experiencing issues after these steps, try:
1. Adding a simple test page to verify the documentation system works
2. Checking the Docusaurus version compatibility
3. Verifying the GitHub Pages deployment settings if you're deploying there

This solution addresses the most common causes of 404 errors in Docusaurus documentation sites.