param(
  [string]$VidPid = 'VID_303A&PID_1001',
  [int]$Baud = 115200,
  [int]$Attempts = 8,
  [int]$ReadyChecks = 3,
  [switch]$WaitForTinyUsb,
  [int]$WaitTimeoutSeconds = 20,
  [int]$StressCount = 0,
  [int]$StressGapMs = 120,
  [int]$SessionCount = 0,
  [switch]$SessionOnly
)

$ErrorActionPreference = 'Stop'

function Get-BusDesc {
  param([string]$InstanceId)
  try {
    return (Get-PnpDeviceProperty -InstanceId $InstanceId -KeyName 'DEVPKEY_Device_BusReportedDeviceDesc' -ErrorAction Stop).Data
  } catch {
    return ''
  }
}

function Get-PresentPorts {
  $ports = Get-PnpDevice -Class Ports -PresentOnly | Where-Object { $_.InstanceId -match $VidPid }
  foreach($p in $ports) {
    $bus = Get-BusDesc -InstanceId $p.InstanceId
    $com = ([regex]::Match($p.FriendlyName, 'COM\d+')).Value
    [pscustomobject]@{
      FriendlyName = $p.FriendlyName
      InstanceId = $p.InstanceId
      BusDesc = $bus
      Com = $com
    }
  }
}

function Probe-MspApi {
  param(
    [string]$Com,
    [bool]$Dtr,
    [int]$BaudRate
  )

  $sp = New-Object System.IO.Ports.SerialPort $Com,$BaudRate,'None',8,'One'
  $sp.ReadTimeout = 900
  $sp.WriteTimeout = 900
  $sp.DtrEnable = $Dtr
  $sp.RtsEnable = $false

  $buf = New-Object byte[] 64
  $n = 0
  try {
    $sp.Open()
    Start-Sleep -Milliseconds 180
    $cmd = [byte[]](0x24,0x4D,0x3C,0x00,0x01,0x01)
    $sp.DiscardInBuffer()
    $sp.Write($cmd,0,$cmd.Length)
    Start-Sleep -Milliseconds 180
    try { $n = $sp.Read($buf,0,$buf.Length) } catch { $n = 0 }
  } finally {
    if($sp.IsOpen) { $sp.Close() }
  }

  $hex = if($n -gt 0) { [BitConverter]::ToString($buf,0,$n) } else { '' }
  $ok = $hex -like '24-4D-3E-03-01-00-01-30-33*'

  [pscustomobject]@{
    Port = $Com
    Dtr = $Dtr
    Bytes = $n
    Hex = $hex
    MspApiOk = $ok
  }
}

function Probe-MspApiSession {
  param(
    [string]$Com,
    [int]$BaudRate,
    [int]$Count
  )

  $sp = New-Object System.IO.Ports.SerialPort $Com,$BaudRate,'None',8,'One'
  $sp.ReadTimeout = 900
  $sp.WriteTimeout = 900
  $sp.DtrEnable = $true
  $sp.RtsEnable = $false

  $ok = 0
  $fail = 0
  $rows = @()
  try {
    $sp.Open()
    Start-Sleep -Milliseconds 220
    for($k = 1; $k -le $Count; $k++) {
      $cmd = [byte[]](0x24,0x4D,0x3C,0x00,0x01,0x01)
      $buf = New-Object byte[] 64
      $n = 0
      $hex = ''

      $sp.DiscardInBuffer()
      $sp.Write($cmd,0,$cmd.Length)
      Start-Sleep -Milliseconds 140
      try { $n = $sp.Read($buf,0,$buf.Length) } catch { $n = 0 }
      if($n -gt 0) { $hex = [BitConverter]::ToString($buf,0,$n) }
      $isOk = $hex -like '24-4D-3E-03-01-00-01-30-33*'
      if($isOk) { $ok++ } else { $fail++ }
      $rows += [pscustomobject]@{ Step = $k; Bytes = $n; MspApiOk = $isOk; Hex = $hex }
      Start-Sleep -Milliseconds 120
    }
  } finally {
    if($sp.IsOpen) { $sp.Close() }
  }

  [pscustomobject]@{
    Ok = $ok
    Fail = $fail
    Rows = $rows
  }
}

function Select-TargetPort {
  param([array]$Ports)
  $target = $Ports | Where-Object { $_.BusDesc -like '*TinyUSB CDC*' } | Select-Object -First 1
  if(-not $target) {
    $target = $Ports | Select-Object -First 1
  }
  return $target
}

function Wait-ForTinyUsbPort {
  param(
    [int]$TimeoutSeconds
  )

  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while((Get-Date) -lt $deadline) {
    $ports = @(Get-PresentPorts)
    $tiny = $ports | Where-Object { $_.BusDesc -like '*TinyUSB CDC*' } | Select-Object -First 1
    if($tiny) {
      return $tiny
    }
    Start-Sleep -Milliseconds 200
  }

  return $null
}

function Confirm-Ready {
  param(
    [string]$Com,
    [int]$BaudRate,
    [int]$Checks
  )

  $ok = 0
  for($k = 1; $k -le $Checks; $k++) {
    try {
      $p = Probe-MspApi -Com $Com -Dtr $true -BaudRate $BaudRate
      if($p.MspApiOk) {
        $ok++
      }
    } catch {
      # ignore and count as failed check
    }
    Start-Sleep -Milliseconds 220
  }
  return $ok -eq $Checks
}

$present = @(Get-PresentPorts)
if($present.Count -eq 0) {
  Write-Host 'No matching ESP32-S3 serial ports are currently present.' -ForegroundColor Yellow
  exit 1
}

Write-Host 'Present ESP32-S3 ports:' -ForegroundColor Cyan
$present | Sort-Object Com | Format-Table Com,BusDesc,FriendlyName -AutoSize

if($WaitForTinyUsb) {
  Write-Host ''
  Write-Host ("Waiting up to {0}s for TinyUSB CDC endpoint. Power-cycle the FC now..." -f $WaitTimeoutSeconds) -ForegroundColor Cyan
  $tinyNow = Wait-ForTinyUsbPort -TimeoutSeconds $WaitTimeoutSeconds
  if($tinyNow) {
    Write-Host ("Detected TinyUSB CDC on {0}. Starting probe attempts..." -f $tinyNow.Com) -ForegroundColor Green
  } else {
    Write-Host 'TinyUSB CDC did not appear in wait window. Continuing with currently available endpoint(s).' -ForegroundColor Yellow
  }
}

$target = $present | Where-Object { $_.BusDesc -like '*TinyUSB CDC*' } | Select-Object -First 1
if(-not $target) {
  $target = $present | Select-Object -First 1
}

Write-Host ''
Write-Host 'Note: COM ports may alternate between reset states and are often not present at the same time.' -ForegroundColor DarkYellow

$stressOk = 0
$stressFail = 0
$history = @()
$seenBus = @()
for($i = 1; $i -le $Attempts; $i++) {
  $portsNow = @(Get-PresentPorts)
  if($portsNow.Count -eq 0) {
    $history += [pscustomobject]@{ Attempt = $i; Port = ''; BusDesc = ''; Dtr = $true; Bytes = 0; MspApiOk = $false; Hex = 'NO_PORT' }
    Start-Sleep -Milliseconds 220
    continue
  }

  $targetNow = Select-TargetPort -Ports $portsNow
  if($targetNow.BusDesc) {
    $seenBus += $targetNow.BusDesc
  }
  Write-Host ("Attempt {0}/{1}: probing {2} ({3})" -f $i, $Attempts, $targetNow.Com, $targetNow.BusDesc) -ForegroundColor Gray

  if($SessionOnly -and $SessionCount -gt 0) {
    try {
      $sessOnly = Probe-MspApiSession -Com $targetNow.Com -BaudRate $Baud -Count $SessionCount
      Write-Host ''
      Write-Host ("Single-session-only summary on {0}: OK={1} FAIL={2}" -f $targetNow.Com, $sessOnly.Ok, $sessOnly.Fail) -ForegroundColor Cyan
      $sessOnly.Rows | Format-Table Step,Bytes,MspApiOk,Hex -AutoSize
      if($sessOnly.Ok -gt 0) {
        Write-Host ''
        Write-Host ("Use {0} in Configurator now." -f $targetNow.Com) -ForegroundColor Green
        exit 0
      }
    } catch {
      $history += [pscustomobject]@{
        Attempt = $i
        Port = $targetNow.Com
        BusDesc = $targetNow.BusDesc
        Dtr = $true
        Bytes = 0
        MspApiOk = $false
        Hex = ('SESSION_ONLY_ERR: ' + $_.Exception.Message)
      }
    }

    Start-Sleep -Milliseconds 220
    continue
  }

  try {
    $dtrOn = Probe-MspApi -Com $targetNow.Com -Dtr $true -BaudRate $Baud
    $dtrOn | Add-Member -NotePropertyName Attempt -NotePropertyValue $i
    $dtrOn | Add-Member -NotePropertyName BusDesc -NotePropertyValue $targetNow.BusDesc
    $history += $dtrOn

    if($dtrOn.MspApiOk) {
      $ready = Confirm-Ready -Com $targetNow.Com -BaudRate $Baud -Checks $ReadyChecks
      if(-not $ready) {
        $history += [pscustomobject]@{
          Attempt = $i
          Port = $targetNow.Com
          BusDesc = $targetNow.BusDesc
          Dtr = $true
          Bytes = 0
          MspApiOk = $false
          Hex = ("READY_CHECK_FAILED: need " + $ReadyChecks + " consecutive replies")
        }
        Start-Sleep -Milliseconds 220
        continue
      }

      if($StressCount -gt 0) {
        for($s = 1; $s -le $StressCount; $s++) {
          try {
            $stress = Probe-MspApi -Com $targetNow.Com -Dtr $true -BaudRate $Baud
            if($stress.MspApiOk) {
              $stressOk++
            } else {
              $stressFail++
              $history += [pscustomobject]@{
                Attempt = $i
                Port = $targetNow.Com
                BusDesc = $targetNow.BusDesc
                Dtr = $true
                Bytes = $stress.Bytes
                MspApiOk = $false
                Hex = ('STRESS_FAIL[' + $s + ']: ' + $stress.Hex)
              }
            }
          } catch {
            $stressFail++
            $history += [pscustomobject]@{
              Attempt = $i
              Port = $targetNow.Com
              BusDesc = $targetNow.BusDesc
              Dtr = $true
              Bytes = 0
              MspApiOk = $false
              Hex = ('STRESS_ERR[' + $s + ']: ' + $_.Exception.Message)
            }
          }
          Start-Sleep -Milliseconds $StressGapMs
        }
      }

      if($SessionCount -gt 0) {
        try {
          $sess = Probe-MspApiSession -Com $targetNow.Com -BaudRate $Baud -Count $SessionCount
          Write-Host ''
          Write-Host ("Single-session summary on {0}: OK={1} FAIL={2}" -f $targetNow.Com, $sess.Ok, $sess.Fail) -ForegroundColor Cyan
          $sess.Rows | Format-Table Step,Bytes,MspApiOk,Hex -AutoSize
        } catch {
          Write-Host ''
          Write-Host ("Single-session test failed on {0}: {1}" -f $targetNow.Com, $_.Exception.Message) -ForegroundColor Yellow
        }
      }

      try {
        $dtrOff = Probe-MspApi -Com $targetNow.Com -Dtr $false -BaudRate $Baud
        $dtrOff | Add-Member -NotePropertyName Attempt -NotePropertyValue $i
        $dtrOff | Add-Member -NotePropertyName BusDesc -NotePropertyValue $targetNow.BusDesc
        $history += $dtrOff
      } catch {
        $history += [pscustomobject]@{
          Attempt = $i
          Port = $targetNow.Com
          BusDesc = $targetNow.BusDesc
          Dtr = $false
          Bytes = 0
          MspApiOk = $false
          Hex = ('WARN: DTR-off probe failed after valid MSP: ' + $_.Exception.Message)
        }
      }

      Write-Host ''
      Write-Host 'MSP probe results:' -ForegroundColor Cyan
      $history | Select-Object Attempt,Port,BusDesc,Dtr,Bytes,MspApiOk,Hex | Format-Table -AutoSize

      if($StressCount -gt 0) {
        Write-Host ''
        Write-Host ("Stress summary on {0}: OK={1} FAIL={2}" -f $targetNow.Com, $stressOk, $stressFail) -ForegroundColor Cyan
      }

      Write-Host ''
      Write-Host ("Use {0} in Configurator now." -f $targetNow.Com) -ForegroundColor Green
      exit 0
    }
  } catch {
    $history += [pscustomobject]@{
      Attempt = $i
      Port = $targetNow.Com
      BusDesc = $targetNow.BusDesc
      Dtr = $true
      Bytes = 0
      MspApiOk = $false
      Hex = ('ERROR: ' + $_.Exception.Message)
    }
  }

  Start-Sleep -Milliseconds 220
}

Write-Host ''
Write-Host 'MSP probe results:' -ForegroundColor Cyan
$history | Select-Object Attempt,Port,BusDesc,Dtr,Bytes,MspApiOk,Hex | Format-Table -AutoSize

Write-Host ''
$seenUnique = @($seenBus | Where-Object { $_ } | Sort-Object -Unique)
if($seenUnique.Count -eq 1 -and $seenUnique[0] -like '*JTAG*') {
  Write-Host 'Only USB JTAG/serial debug endpoint was seen. MSP handshake usually works on TinyUSB CDC endpoint.' -ForegroundColor Yellow
  Write-Host 'Power-cycle the FC and rerun this script. As soon as TinyUSB CDC appears, connect with that COM in Configurator.' -ForegroundColor Yellow
} else {
  Write-Host 'No valid MSP API response found during retry window. Power-cycle the FC and run this command again, then connect immediately on the reported COM.' -ForegroundColor Yellow
}
exit 2
