# Define file paths
$tmpFile = ".\tmp.tmp"
$filesList = ".\all-files.txt"
$importsFile = ".\include\imports.py"
$outputFile = ".\main.py"

# Create and clear a temporary file
if (Test-Path $tmpFile) {
    Clear-Content -Path $tmpFile
} else {
    New-Item -Path $tmpFile -ItemType File | Out-Null
}

# Add specified files to the temporary file
Get-Content -Path $filesList | ForEach-Object {
    $inputFile = $_
    # Add a header before each file
    Add-Content -Path $tmpFile -Value "`n`n`n# $inputFile ---"
    # Add file content
    Add-Content -Path $tmpFile -Value (Get-Content -Path $inputFile -Raw)
}

# Read the temporary file, remove import lines, and store in a variable
$content = Get-Content -Path $tmpFile | Where-Object { $_ -notmatch "^from " -and $_ -notmatch "^import " }

# Write the stdlib imports to the final output file
Get-Content -Path $importsFile | Set-Content -Path $outputFile

# Append the processed content to the final output file
Add-Content -Path $outputFile -Value $content

# Remove the temporary file
Remove-Item -Path $tmpFile

Write-Host "Successfully created $outputFile"
