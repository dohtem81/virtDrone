#!/usr/bin/env bash
set -euo pipefail

root=/workspace
reportDir="$root/reports"
scenarioDir="$reportDir/scenarios"
chartDir="$root/docs/tutorials/charts"

mkdir -p "$reportDir" "$scenarioDir"

ctest --test-dir "$root/build" --output-on-failure > "$reportDir/ctest-output.txt" 2>&1 || true

missionList=""
for m in "$root"/config/missions/*.yaml; do
  [ -f "$m" ] || continue
  name=$(basename "$m" .yaml)
  stdout="$scenarioDir/${name}-run.log"
  csv="$scenarioDir/${name}.csv"

  if [ -x "$root/build/simulator_app" ]; then
    "$root/build/simulator_app" 10000 0.01 "$root/config/altitude_controller.yaml" "$root/config/attitude_controller.yaml" "$root/config/weather.yaml" "$m" "$root/docs/tutorials" > "$stdout" 2>&1 || true
  fi

  if [ -f "$root/docs/tutorials/simulation_telemetry.csv" ]; then
    cp "$root/docs/tutorials/simulation_telemetry.csv" "$csv"
  fi

  missionList="${missionList}<li>${name} | ${stdout} | ${csv}</li>"
done

chartList=""
if [ -d "$chartDir" ]; then
  for img in "$chartDir"/*; do
    [ -f "$img" ] || continue
    case "${img##*.}" in
      png|jpg|jpeg|webp)
        name=$(basename "$img")
        chartList="${chartList}<li>../docs/tutorials/charts/${name}</li>"
        ;;
    esac
  done
fi

{
  printf '<!doctype html><html><head><meta charset="utf-8" /><title>virtDrone Refactor Validation Summary</title><style>body{font-family:Segoe UI,Arial,sans-serif;margin:24px;line-height:1.4}h1,h2{margin:0 0 12px}section{margin:24px 0}pre{background:#111;color:#eee;padding:12px;overflow:auto;max-height:420px}</style></head><body>'
  printf '<h1>virtDrone Refactor Validation Summary</h1><p>Generated inside Docker.</p>'
  printf '<section><h2>Test Results (ctest)</h2><pre>'
  sed -e 's/&/\&amp;/g' -e 's/</\&lt;/g' -e 's/>/\&gt;/g' "$reportDir/ctest-output.txt"
  printf '</pre></section>'
  printf '<section><h2>Tested Scenarios</h2><ul>%s</ul></section>' "$missionList"
  printf '<section><h2>Charts</h2><ul>%s</ul></section>' "$chartList"
  printf '</body></html>'
} > "$reportDir/refactor-validation-summary.html"

echo "$reportDir/refactor-validation-summary.html"
