$ErrorActionPreference = 'Stop'
# Stub Python; these tests must never open CAN.
function global:python { $global:ClawTestArguments = @($args) }
try {
    Import-Module (Join-Path $PSScriptRoot '..\claw.psm1') -ArgumentList 'C:\test path\ControlCANFD.dll' -Force
    claw seek 2 --clear --log dummy.jsonl
    $a = $global:ClawTestArguments
    if ($a[-5..-1] -join '|' -ne 'seek-open|2|--clear|--log|dummy.jsonl') { throw 'seek forwarding failed' }
    if ($a -notcontains 'C:\test path\ControlCANFD.dll') { throw 'DLL quoting failed' }
    claw p set 2 seek_pwm 420 --help
    if ($global:ClawTestArguments[-6..-1] -join '|' -ne 'params|set|2|seek_pwm|420|--help') { throw 'params alias failed' }
    claw
    if ($global:ClawTestArguments[-1] -ne '--help') { throw 'empty command should show help' }
    if ($global:ClawTestArguments -contains '--clear') { throw 'must not clear implicitly' }
    'Short CLI forwarding passed (no CAN opened).'
} finally {
    Remove-Module claw -ErrorAction SilentlyContinue
    Remove-Item Function:\python -ErrorAction SilentlyContinue
    Remove-Variable ClawTestArguments -Scope Global -ErrorAction SilentlyContinue
}
