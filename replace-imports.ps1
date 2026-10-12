# CLI script to replace edu.wpi.first and edu.wpilib.first imports with org.wpilib
param (
    [string]$Path = "src"
)

Write-Host "Reemplazando imports en $Path..." -ForegroundColor Cyan

Get-ChildItem -Path $Path -Recurse -Filter "*.java" | ForEach-Object {
    $content = Get-Content $_.FullName -Raw
    if ($content -match 'edu\.(wpi|wpilib)\.first') {
        $updated = $content -replace 'edu\.(wpi|wpilib)\.first', 'org.wpilib'
        Set-Content -Path $_.FullName -Value $updated -NoNewline
        Write-Host "Actualizado: $($_.FullName)" -ForegroundColor Green
    }
}

Write-Host "Listo! Todos los imports fueron actualizados a org.wpilib." -ForegroundColor Green

