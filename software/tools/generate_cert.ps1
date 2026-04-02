# 生成自签名证书（Windows PowerShell 版本）
# 使用方法: powershell -ExecutionPolicy Bypass -File tools/generate_cert.ps1

$certDir = "$PSScriptRoot\..\certs"
if (-not (Test-Path $certDir)) {
    New-Item -ItemType Directory -Path $certDir | Out-Null
}

$certPath = "$certDir\server.crt"
$keyPath = "$certDir\server.key"

# 生成证书
$cert = New-SelfSignedCertificate `
    -DnsName "localhost", "*.local" `
    -CertStoreLocation "cert:\LocalMachine\My" `
    -KeyAlgorithm RSA `
    -KeyLength 2048 `
    -NotAfter (Get-Date).AddYears(1) `
    -FriendlyName "HomeBot Mahjong"

# 导出证书
$certThumbprint = $cert.Thumbprint
$pfxPath = "$certDir\temp.pfx"
$password = ConvertTo-SecureString -String "homebot" -Force -AsPlainText

Export-PfxCertificate `
    -Cert "cert:\LocalMachine\My\$certThumbprint" `
    -FilePath $pfxPath `
    -Password $password | Out-Null

# 使用 OpenSSL 转换（如果安装了）或提示用户
Write-Host "证书已生成到: $certDir"
Write-Host "Thumbprint: $certThumbprint"
Write-Host ""
Write-Host "注意: Windows 自带证书格式与 Flask 不兼容"
Write-Host "推荐方案: 使用 ngrok 进行 HTTPS 内网穿透"
Write-Host "  ngrok http 5100"

# 清理
Remove-Item "cert:\LocalMachine\My\$certThumbprint" -Force
