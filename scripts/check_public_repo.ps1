$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Push-Location $repoRoot

try {
    $files = @(git ls-files --cached --others --exclude-standard)
    $findings = New-Object System.Collections.Generic.List[string]

    $forbiddenPath = '(?i)(^|/)(_local|Output|OBJ|backup|实验报告|运行记录|render_check|历史版本)(/|$)'
    $forbiddenExtension = '(?i)\.(docx|xlsx|xls|pptx|pdf|pem|p12|pfx|exe|pyc)$'

    foreach ($file in $files) {
        $normalized = $file.Replace('\', '/')
        if ($normalized -match $forbiddenPath -or $normalized -match $forbiddenExtension) {
            $findings.Add("forbidden path or artifact: $normalized")
        }
    }

    $patterns = [ordered]@{
        private_key = '-----BEGIN (RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----'
        openai_key = '\bsk-(proj-|svcacct-)?[A-Za-z0-9_-]{20,}\b'
        github_token = '\b(gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,})\b'
        aws_access_key = '\b(AKIA|ASIA)[0-9A-Z]{16}\b'
        google_api_key = '\bAIza[0-9A-Za-z_-]{30,}\b'
        slack_token = '\bxox[baprs]-[0-9A-Za-z-]{10,}\b'
        stripe_key = '\b[rs]k_(live|test)_[0-9A-Za-z]{16,}\b'
        jwt = '\beyJ[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}\b'
        credentialed_database_url = '(?i)\b(postgres(ql)?|mysql|mongodb(\+srv)?|redis)://[^/\s:@]+:[^@\s/]+@'
        windows_user_path = '[A-Za-z]:\\Users\\[^\\\s<>:"|?*]+'
        wechat_id = '\bwxid_[A-Za-z0-9_]+\b'
    }

    foreach ($file in $files) {
        if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { continue }
        $item = Get-Item -LiteralPath $file
        if ($item.Length -gt 5MB) { continue }
        $latin1 = [System.Text.Encoding]::GetEncoding(28591)
        $text = $latin1.GetString([System.IO.File]::ReadAllBytes($item.FullName))
        foreach ($entry in $patterns.GetEnumerator()) {
            if ($text -match $entry.Value) {
                $findings.Add("$($entry.Key): $($file.Replace('\', '/'))")
            }
        }
    }

    if ($findings.Count -gt 0) {
        $findings | Sort-Object -Unique | ForEach-Object { Write-Error $_ }
        exit 1
    }

    Write-Output "Public repository check passed: $($files.Count) candidate files."
}
finally {
    Pop-Location
}
