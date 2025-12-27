## Manual Fix for Logo Issue

Since I'm unable to directly edit the configuration file, here's how to manually fix the logo issue:

1. Open the file: `E:/hackthon_ai_book/robotics_ai_book/docusaurus-book/docusaurus.config.js`

2. Find the line containing: `src: 'img/logo.svg',`

3. Change it to: `src: 'img/logo.jpg',`

4. Save the file

5. Restart your Docusaurus development server:
   ```bash
   npm start
   ```

## Alternative Method Using Command Line

If you prefer a command-line approach, you can use PowerShell to make the change:

```powershell
# Navigate to the project directory
cd E:\hackthon_ai_book\robotics_ai_book\docusaurus-book

# Replace the line in the config file
(Get-Content docusaurus.config.js) -replace "src: 'img/logo.svg'", "src: 'img/logo.jpg'" | Set-Content docusaurus.config.js
```

## Verification Steps

After making the change:
1. Check that the image file exists at: `static/img/logo.jpg`
2. Verify the file size is approximately 6.4MB (which matches your ai-robot-library.jpg)
3. Restart your development server
4. Refresh your browser to see the updated logo

The logo should now appear in the top navigation bar of your landing page.