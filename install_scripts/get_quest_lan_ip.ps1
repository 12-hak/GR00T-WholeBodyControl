# Print best LAN IPv4 for Quest (single line, no banner).
$ErrorActionPreference = "SilentlyContinue"
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$ip = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object {
    $_.IPAddress -match '^192\.168\.1\.\d+$' -and
    $_.InterfaceAlias -notmatch 'Loopback|vEthernet|VirtualBox|VMware|WSL|Hyper-V'
} | Select-Object -First 1).IPAddress
if ($ip) { Write-Output $ip.Trim() }
