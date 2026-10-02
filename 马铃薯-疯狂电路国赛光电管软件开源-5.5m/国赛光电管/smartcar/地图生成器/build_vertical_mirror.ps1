param(
    [string]$Source = "juesai.json",
    [string]$Destination = "juesai_shuangjuesai.json",
    [double]$GapMm = 450
)

$ErrorActionPreference = "Stop"
$UpperPrefix = "U_"

function Copy-MapPoint {
    param(
        [Parameter(Mandatory)]$Point,
        [Parameter(Mandatory)][string]$Name,
        [Parameter(Mandatory)][double]$X,
        [Parameter(Mandatory)][double]$Y,
        [string]$LabelPrefix = ""
    )

    $copy = [ordered]@{}
    foreach ($property in $Point.PSObject.Properties) {
        switch ($property.Name) {
            "name" { $copy.name = $Name }
            "coord" { $copy.coord = @($X, $Y) }
            "label" { $copy.label = $LabelPrefix + [string]$property.Value }
            default { $copy[$property.Name] = $property.Value }
        }
    }
    if (-not $copy.Contains("name")) { $copy.name = $Name }
    if (-not $copy.Contains("coord")) { $copy.coord = @($X, $Y) }
    return [pscustomobject]$copy
}

$sourcePath = Join-Path $PSScriptRoot $Source
$destinationPath = Join-Path $PSScriptRoot $Destination
$map = Get-Content -Raw -LiteralPath $sourcePath | ConvertFrom-Json

$allY = @()
foreach ($property in $map.nodes.PSObject.Properties) { $allY += [double]$property.Value[1] }
foreach ($point in @($map.required_points) + @($map.nocal_points)) {
    if ($null -ne $point -and $null -ne $point.coord) { $allY += [double]$point.coord[1] }
}
if ($null -ne $map.start.coord) { $allY += [double]$map.start.coord[1] }
if ($allY.Count -eq 0) { throw "Source map has no coordinates." }

$minY = ($allY | Measure-Object -Minimum).Minimum
$maxY = ($allY | Measure-Object -Maximum).Maximum
$trackHeight = $maxY - $minY
$lowerOffset = $trackHeight + $GapMm - $minY

$nodes = [ordered]@{}
foreach ($property in $map.nodes.PSObject.Properties) {
    # The start object is imported as its own point. Do not keep the redundant
    # normal-node copy from older JSON exports.
    if ($property.Name -eq $map.start.name) { continue }
    $nodes[$property.Name] = @([double]($property.Value[0]), ([double]($property.Value[1]) + $lowerOffset))
}
foreach ($property in $map.nodes.PSObject.Properties) {
    $nodes[$UpperPrefix + $property.Name] = @([double]($property.Value[0]), ($maxY - [double]($property.Value[1])))
}

$edges = @()
foreach ($edge in $map.edges) { $edges += ,@([string]$edge[0], [string]$edge[1]) }
foreach ($edge in $map.edges) {
    $upperA = $UpperPrefix + [string]($edge[0])
    $upperB = $UpperPrefix + [string]($edge[1])
    $edges += ,@($upperA, $upperB)
}

$topBoundaryNodes = @(
    $map.nodes.PSObject.Properties | Where-Object { [double]$_.Value[1] -eq $minY }
)
if ($topBoundaryNodes.Count -lt 2) { throw "Need at least two nodes on the source map's top boundary." }
$leftConnector = $topBoundaryNodes | Sort-Object { [double]$_.Value[0] } | Select-Object -First 1
$rightConnector = $topBoundaryNodes | Sort-Object { [double]$_.Value[0] } | Select-Object -Last 1
$upperLeftConnector = $UpperPrefix + $leftConnector.Name
$upperRightConnector = $UpperPrefix + $rightConnector.Name
$edges += ,@($upperLeftConnector, [string]$leftConnector.Name)
$edges += ,@($upperRightConnector, [string]$rightConnector.Name)

$requiredPoints = [System.Collections.Generic.List[object]]::new()
foreach ($point in @($map.required_points)) {
    $requiredPoints.Add((Copy-MapPoint -Point $point -Name $point.name -X $point.coord[0] -Y ([double]($point.coord[1]) + $lowerOffset)))
}
foreach ($point in @($map.required_points)) {
    $requiredPoints.Add((Copy-MapPoint -Point $point -Name ($UpperPrefix + $point.name) -X $point.coord[0] -Y ($maxY - [double]($point.coord[1])) -LabelPrefix "上-"))
}

$nocalPoints = [System.Collections.Generic.List[object]]::new()
foreach ($point in @($map.nocal_points)) {
    if ($null -eq $point) { continue }
    $nocalPoints.Add((Copy-MapPoint -Point $point -Name $point.name -X $point.coord[0] -Y ([double]($point.coord[1]) + $lowerOffset)))
}
foreach ($point in @($map.nocal_points)) {
    if ($null -eq $point) { continue }
    $nocalPoints.Add((Copy-MapPoint -Point $point -Name ($UpperPrefix + $point.name) -X $point.coord[0] -Y ($maxY - [double]($point.coord[1])) -LabelPrefix "上-"))
}

$map.title = "上下镜像并联决赛赛道"
$map.nodes = [pscustomobject]$nodes
$map.edges = @($edges)
$map.start.coord = @([double]($map.start.coord[0]), ([double]($map.start.coord[1]) + $lowerOffset))
$map.required_points = @($requiredPoints)
if ($map.PSObject.Properties.Name -contains "nocal_points") { $map.nocal_points = @($nocalPoints) }
$map.routes = @()
if (-not ($map.PSObject.Properties.Name -contains "route_settings") -or $null -eq $map.route_settings) {
    $map | Add-Member -NotePropertyName route_settings -NotePropertyValue ([pscustomobject]@{})
}
if (-not ($map.route_settings.PSObject.Properties.Name -contains "swerve_gap_cm")) {
    $map.route_settings | Add-Member -NotePropertyName swerve_gap_cm -NotePropertyValue 10
}

$json = $map | ConvertTo-Json -Depth 100
[System.IO.File]::WriteAllText($destinationPath, $json + [Environment]::NewLine, [System.Text.UTF8Encoding]::new($false))

Write-Host "Generated $Destination"
Write-Host "  track bounds: y=$minY..$maxY mm"
Write-Host "  lower offset: $lowerOffset mm"
Write-Host "  gap: $GapMm mm"
Write-Host "  connectors: $($leftConnector.Name), $($rightConnector.Name)"
