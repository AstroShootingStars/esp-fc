function Get-RpState {
  $devs = Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -match 'VID_2E8A' }
  $runtime = $devs | Where-Object { $_.DeviceID -match 'PID_000A|PID_010A|PID_400A|PID_410A|PID_800A|PID_810A|PID_C00A|PID_C10A' }
  $boot = $devs | Where-Object { $_.DeviceID -match 'PID_0003' }
  [PSCustomObject]@{
    RuntimeCount = @($runtime).Count
    BootselCount = @($boot).Count
    RuntimeNames = (@($runtime | Select-Object -ExpandProperty Name) -join '; ')
    BootselNames = (@($boot | Select-Object -ExpandProperty Name) -join '; ')
  }
}

function Wait-RpRuntime([int]$timeoutMs = 12000) {
  $deadline = (Get-Date).AddMilliseconds($timeoutMs)
  while((Get-Date) -lt $deadline) {
    $s = Get-RpState
    if($s.RuntimeCount -gt 0) {
      return $s
    }
    Start-Sleep -Milliseconds 250
  }
  return $null
}

$initial = Wait-RpRuntime 12000
if(-not $initial) {
  $snap = Get-RpState
  Write-Output ("INITIAL Runtime={0} Bootsel={1}" -f $snap.RuntimeCount, $snap.BootselCount)
  Write-Output "No RP runtime device present to test."
  exit 2
}
Write-Output ("INITIAL Runtime={0} Bootsel={1}" -f $initial.RuntimeCount, $initial.BootselCount)

$comName = ($initial.RuntimeNames -split '; ' | Where-Object { $_ -match 'COM\d+' } | Select-Object -First 1)
if(-not $comName){
  Write-Output "Could not parse runtime COM port from RP device name."
  exit 3
}

$null = $comName -match 'COM\d+'
$com = $matches[0]
Write-Output ("TEST_PORT={0}" -f $com)

for($i=1; $i -le 12; $i++){
  try {
    $sp = New-Object System.IO.Ports.SerialPort $com,1200,'None',8,'One'
    $sp.DtrEnable = $false
    $sp.RtsEnable = $false
    $sp.ReadTimeout = 200
    $sp.WriteTimeout = 200
    $sp.Open()
    Start-Sleep -Milliseconds 120
  } catch {
    Write-Output ("ITER {0} OPEN_ERR: {1}" -f $i, $_.Exception.Message)
  } finally {
    if($sp -and $sp.IsOpen){ $sp.Close() }
  }

  Start-Sleep -Milliseconds 220
  $s = Get-RpState
  Write-Output ("ITER {0} Runtime={1} Bootsel={2}" -f $i, $s.RuntimeCount, $s.BootselCount)
  if($s.BootselCount -gt 0){
    Write-Output ("ITER {0} BOOTSEL_DETECTED" -f $i)
    exit 0
  }
}

Write-Output "NO_BOOTSEL_DETECTED"
exit 1
