param(
  [string]$Port = 'COM4',
  [int]$Baud = 115200
)

$ErrorActionPreference = 'Stop'

function Read-All {
  param(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$Loops
  )

  $acc = New-Object System.Collections.Generic.List[byte]
  for($i = 0; $i -lt $Loops; $i++) {
    $buf = New-Object byte[] 256
    $n = 0
    try { $n = $Serial.Read($buf, 0, $buf.Length) } catch { $n = 0 }
    if($n -gt 0) {
      for($k = 0; $k -lt $n; $k++) { [void]$acc.Add($buf[$k]) }
    }
    Start-Sleep -Milliseconds 120
  }

  return [Text.Encoding]::ASCII.GetString($acc.ToArray())
}

$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$sp.ReadTimeout = 250
$sp.WriteTimeout = 250
$sp.DtrEnable = $true
$sp.RtsEnable = $false

try {
  $sp.Open()
  Start-Sleep -Milliseconds 350
  $sp.DiscardInBuffer()

  $sp.Write('#')
  Start-Sleep -Milliseconds 200
  $enter = Read-All -Serial $sp -Loops 8

  $sp.Write("status`r`n")
  Start-Sleep -Milliseconds 150
  $status = Read-All -Serial $sp -Loops 14

  $sp.Write("mode_debug`r`n")
  Start-Sleep -Milliseconds 150
  $modeDebug = Read-All -Serial $sp -Loops 20

  $sp.Write("exit`r`n")
  Start-Sleep -Milliseconds 120
  $exit = Read-All -Serial $sp -Loops 6
}
finally {
  if($sp.IsOpen) { $sp.Close() }
}

Write-Output '===== CLI ENTER ====='
Write-Output $enter
Write-Output '===== STATUS ====='
Write-Output $status
Write-Output '===== MODE_DEBUG ====='
Write-Output $modeDebug
Write-Output '===== EXIT ====='
Write-Output $exit
