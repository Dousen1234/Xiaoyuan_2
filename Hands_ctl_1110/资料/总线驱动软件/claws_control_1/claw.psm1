# Import-Module .\claw.psm1 -ArgumentList $dll -Force
# Only wraps the host CLI. Importing this module never opens CAN.
param(
    [string]$DllPath = $env:CXCANFD_DLL,
    [string]$Interface = 'cxcanfd',
    [string]$Channel = '0',
    [int]$Group = 2
)
$script:ClawDll = $DllPath
$script:ClawInterface = $Interface
$script:ClawChannel = $Channel
$script:ClawGroup = $Group

function claw {
    # No declared parameters: forward all CLI switches literally to Python.
    $forward = @($args)
    if ($forward.Count -eq 0) { $forward = @('--help') }
    switch ($forward[0]) {
        'seek' { $forward[0] = 'seek-open' }
        'p' { $forward = @('params') + @($forward | Select-Object -Skip 1) }
    }
    $options = @('--interface', $script:ClawInterface, '--channel', $script:ClawChannel,
                 '--claw-group', "$script:ClawGroup")
    if ($script:ClawDll) { $options += @('--dll', $script:ClawDll) }
    $isHelp = ($forward -contains '--help') -or ($forward -contains '-h')
    if (-not $isHelp -and -not ($forward -contains '--log')) {
        $logDir = Join-Path $PSScriptRoot 'logs'
        New-Item -ItemType Directory -Path $logDir -Force | Out-Null
        $filename = 'claw_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff') + '_' +
                    ([guid]::NewGuid().ToString('N').Substring(0,8)) + '.jsonl'
        $options += @('--log', (Join-Path $logDir $filename))
    }
    # Absolute script path works even after changing the terminal directory.
    & python (Join-Path $PSScriptRoot 'host_demo\claw_demo.py') @options @forward
}
Export-ModuleMember -Function claw
