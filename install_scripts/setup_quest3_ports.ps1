# Forward Quest teleop ports from Windows LAN -> WSL (run PowerShell as Administrator once).
# Usage: powershell -ExecutionPolicy Bypass -File install_scripts\setup_quest3_ports.ps1

$ErrorActionPreference = "Stop"

$ports = @(8766)
$wslIp = (wsl -d Ubuntu-22.04 -- hostname -I).Trim().Split()[0]
if (-not $wslIp) { throw "Could not get WSL IP. Is Ubuntu-22.04 running?" }

Write-Host "WSL IP: $wslIp"

foreach ($port in $ports) {
    netsh interface portproxy delete v4tov4 listenport=$port listenaddress=0.0.0.0 2>$null
    netsh interface portproxy add v4tov4 listenport=$port listenaddress=0.0.0.0 connectport=$port connectaddress=$wslIp
    $ruleName = "GR00T Quest port $port"
    Remove-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue
    New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Action Allow -Protocol TCP -LocalPort $port | Out-Null
    Write-Host "Forwarded 0.0.0.0:$port -> ${wslIp}:$port"
}

$lanIp = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object {
    $_.IPAddress -match '^192\.168\.\d+\.\d+$' -and $_.IPAddress -notmatch '^192\.168\.56\.'
    $_.InterfaceAlias -notmatch 'Loopback|vEthernet|VirtualBox|VMware|WSL|Hyper-V'
} | Select-Object -First 1).IPAddress

if (-not $lanIp) { $lanIp = "192.168.1.235" }

Write-Host ""
Write-Host "Quest Browser URL (HTTPS):"
Write-Host "  https://${lanIp}:8766/webxr_client.html?host=${lanIp}"
Write-Host ""
Write-Host "Port proxy status:"
netsh interface portproxy show v4tov4
