try {
    # Temporarily allow script execution
    $currentPolicy = Get-ExecutionPolicy
    if ($currentPolicy -ne "Unrestricted" -and $currentPolicy -ne "Bypass") {
        Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
    }

    $destination = "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\Fusion_URDF_Exporter_ROS2"

    Write-Host "Starting installation process..." -ForegroundColor Green

    # Check if the destination folder exists and remove it
    if (Test-Path -Path $destination) {
        Write-Host "Existing folder found. Removing..." -ForegroundColor Yellow
        Remove-Item -Path $destination -Recurse -Force
    }

    # Copy the new folder to the destination
    Copy-Item ".\Fusion_URDF_Exporter_ROS2\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
    Write-Host "Installation completed successfully!" -ForegroundColor Green
} catch {
    # Print the error if something goes wrong
    Write-Host "An error occurred during installation: $($_.Exception.Message)" -ForegroundColor Red
} finally {
    # Restore the original execution policy
    if ($currentPolicy -ne "Unrestricted" -and $currentPolicy -ne "Bypass") {
        Set-ExecutionPolicy -Scope Process -ExecutionPolicy $currentPolicy -Force
    }
    Write-Host "Execution policy restored to original state." -ForegroundColor Green
}

# Exit immediately
exit
