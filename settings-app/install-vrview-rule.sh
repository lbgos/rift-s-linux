#!/usr/bin/env bash
# KDE Window Rules: Minimized / Force, exact VR View title and class.
# https://docs.kde.org/stable/en/kwin/kcontrol/windowspecific/window-attributes.html
# Disable this rule in KDE Window Rules to restore VR View. No size rule.
set -euo pipefail
command -v kwriteconfig6 >/dev/null
command -v kreadconfig6 >/dev/null
command -v qdbus6 >/dev/null
rule_file=$(dirname -- "${BASH_SOURCE[0]}")/vrview.kwinrule
group=rifts-settings-vrview
while IFS='=' read -r key value; do
    [[ $key == \[* || -z $key ]] && continue
    kwriteconfig6 --file kwinrulesrc --group "$group" --key "$key" "$value"
done < "$rule_file"
# Preserve the user's rule order. Plasma 6 uses rules/count; newer KWin uses
# Order. Write both formats so the rule is loaded by either version.
order=$(kreadconfig6 --file kwinrulesrc --group General --key rules)
if [[ -z $order ]]; then
    order=$(kreadconfig6 --file kwinrulesrc --group General --key Order)
fi
if [[ -z $order ]]; then
    count=$(kreadconfig6 --file kwinrulesrc --group General --key count --default 0)
    for ((i=1; i<=count; i++)); do order="${order:+$order,}$i"; done
fi
if [[ ,$order, != *,$group,* ]]; then
    order="${order:+$order,}$group"
fi
kwriteconfig6 --file kwinrulesrc --group General --key Order "$order"
kwriteconfig6 --file kwinrulesrc --group General --key rules "$order"
IFS=',' read -r -a groups <<< "$order"
kwriteconfig6 --file kwinrulesrc --group General --key count "${#groups[@]}"
# Reload configuration only. Never restart the desktop compositor.
qdbus6 org.kde.KWin /KWin org.kde.KWin.reconfigure
