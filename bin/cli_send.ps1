param(
  [string]$Port = 'COM4',
  [int]$Baud = 115200,
  [string[]]$Commands = @('status', 'mode_debug')
)

$ErrorActionPreference = 'Stop'

function Read-All {
  param(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$Loops = 10
  )

  $acc = New-Object System.Collections.Generic.List[byte]
  for($i = 0; $i -lt $Loops; $i++) {
    $buf = New-Object byte[] 1024
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
  Write-Output '===== CLI ENTER ====='
  Write-Output (Read-All -Serial $sp -Loops 8)

  foreach($cmd in $Commands) {
    $sp.Write(($cmd + "`r`n"))
    Start-Sleep -Milliseconds 150
    Write-Output ("===== " + $cmd + " =====")
    Write-Output (Read-All -Serial $sp -Loops 14)
  }

  $sp.Write("exit`r`n")
  Start-Sleep -Milliseconds 120
  Write-Output '===== EXIT ====='
  Write-Output (Read-All -Serial $sp -Loops 6)
}
finally {
  if($sp.IsOpen) { $sp.Close() }
}
