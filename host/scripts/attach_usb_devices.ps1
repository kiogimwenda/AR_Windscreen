<#
.SYNOPSIS
    Shares the USB camera, the LiDAR's USB-Ethernet adapter and the STM32 hub with WSL2
    (docs/BUILD_GUIDE.md Part 2.2). Run on Windows at the start of every session: WSL2 loses USB
    attachments on every restart.

.DESCRIPTION
    Each device is named by its bus id (as `usbipd list` shows it, e.g. 2-3) or by its USB hardware
    id (VID:PID, e.g. 0483:5740), which survives being plugged into another port. The hub defaults
    to 0483:5740 (the STM32's USB serial port); the camera's and the adapter's ids are given once
    and remembered (-Save) in %LOCALAPPDATA%\ar_windscreen\usb_devices.json.

    A device not yet shared ("bound") is bound first: that alone needs an ADMIN PowerShell. A
    device already attached is left as it is. A device not plugged in is reported and skipped.
    At the end the devices are listed from inside WSL2 (/dev/video*, /dev/ttyACM*, the adapter).

.EXAMPLE
    # The first time (admin), finding the ids with `usbipd list`:
    .\attach_usb_devices.ps1 -Camera 0c45:6366 -Ethernet 0bda:8153 -Save
.EXAMPLE
    # Every session after:
    .\attach_usb_devices.ps1
.EXAMPLE
    .\attach_usb_devices.ps1 -Detach      # give the devices back to Windows
#>
[CmdletBinding()]
param(
    [string]$Camera,                # bus id (2-3) or hardware id (VID:PID)
    [string]$Ethernet,
    [string]$Hub = "0483:5740",
    [string]$Distribution = "Debian",
    [switch]$Save,
    [switch]$Detach
)
$ErrorActionPreference = "Stop"

$configPath = Join-Path $env:LOCALAPPDATA "ar_windscreen\usb_devices.json"

function Test-Admin {
    $p = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
    return $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
    throw "usbipd is not installed: winget install usbipd (Part 2.2)"
}

# Remembered ids fill in what was not given.
$saved = @{}
if (Test-Path $configPath) {
    (Get-Content $configPath -Raw | ConvertFrom-Json).PSObject.Properties | ForEach-Object { $saved[$_.Name] = $_.Value }
}
$devices = [ordered]@{
    camera   = $(if ($Camera) { $Camera } else { $saved["camera"] })
    ethernet = $(if ($Ethernet) { $Ethernet } else { $saved["ethernet"] })
    hub      = $(if ($PSBoundParameters.ContainsKey("Hub")) { $Hub } elseif ($saved["hub"]) { $saved["hub"] } else { $Hub })
}
if ($Save) {
    New-Item -ItemType Directory -Force -Path (Split-Path $configPath) | Out-Null
    $devices | ConvertTo-Json | Set-Content $configPath
    Write-Host "remembered in $configPath"
}

# usbipd 4.x: `usbipd state` is JSON (bus id, instance id with VID/PID, bound, attached).
function Get-UsbState { (usbipd state | Out-String | ConvertFrom-Json).Devices }

function Find-Device($state, [string]$id) {
    if ($id -match '^[0-9a-fA-F]{4}:[0-9a-fA-F]{4}$') {
        $vid, $vidpid = $id.Split(":")
        $pattern = "VID_$($vid.ToUpper())&PID_$($vidpid.ToUpper())"
        return $state | Where-Object { $_.BusId -and $_.InstanceId -match [regex]::Escape($pattern) } | Select-Object -First 1
    }
    return $state | Where-Object { $_.BusId -eq $id } | Select-Object -First 1
}

$state = Get-UsbState
$problems = 0
foreach ($name in $devices.Keys) {
    $id = $devices[$name]
    if (-not $id) {
        Write-Warning "$name`: no id given (-$((Get-Culture).TextInfo.ToTitleCase($name)) <busid or VID:PID>, see usbipd list)"
        $problems++
        continue
    }
    $d = Find-Device $state $id
    if (-not $d) {
        Write-Warning "$name ($id): not plugged in"
        $problems++
        continue
    }
    $label = "$name ($id, bus $($d.BusId), $($d.Description))"
    if ($Detach) {
        if ($d.ClientIPAddress) { usbipd detach --busid $d.BusId; Write-Host "detached $label" }
        else { Write-Host "$label was not attached" }
        continue
    }
    if (-not $d.PersistedGuid) {
        if (-not (Test-Admin)) {
            Write-Warning "$label is not shared yet: run this once from an ADMIN PowerShell to bind it"
            $problems++
            continue
        }
        usbipd bind --busid $d.BusId
        Write-Host "bound $label"
    }
    if ($d.ClientIPAddress) {
        Write-Host "already attached: $label"
        continue
    }
    usbipd attach --wsl $Distribution --busid $d.BusId
    if ($LASTEXITCODE -ne 0) { Write-Warning "attach failed: $label"; $problems++ }
    else { Write-Host "attached $label" }
}

if (-not $Detach) {
    Start-Sleep -Seconds 2  # let WSL2 enumerate them
    Write-Host "`nInside WSL2 ($Distribution):"
    wsl -d $Distribution -- sh -c 'ls /dev/video* /dev/ttyACM* 2>/dev/null; ip -br link | grep -v -e "^lo" -e "^eth0" || true'
}
if ($problems -gt 0) {
    Write-Warning "$problems device(s) $(if ($Detach) { 'not found' } else { 'not attached' })"
    exit 1
}
