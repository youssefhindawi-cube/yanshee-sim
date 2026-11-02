#!/bin/sh
# Installed by yanshee_sim (tools/robot_setup.py). At boot: wait for the Wi-Fi, switch its power saving off (it
# added 0.3-2 s stalls, measured 2 Nov 2026; the driver turns it back on at every connection, so
# /etc/dhcpcd.exit-hook switches it off again then) and sync the clock (systemd-timesyncd) as soon as the network is
# there, so the clock does not jump in the middle of a logged session. The result goes to the system log:
# journalctl -t yanshee-net-tune
IW=/sbin/iw
w=0
while [ $w -lt 300 ]; do
    $IW dev wlan0 set power_save off 2>/dev/null
    $IW dev wlan0 link 2>/dev/null | grep -q "^Connected" && break
    sleep 1
    w=$((w + 1))
done
$IW dev wlan0 set power_save off 2>/dev/null
systemctl restart systemd-timesyncd
t=0
while [ $t -lt 90 ]; do
    timedatectl 2>/dev/null | grep -q "synchronized: yes" && break
    sleep 1
    t=$((t + 1))
done
logger -t yanshee-net-tune "Wi-Fi up after $w s, $($IW dev wlan0 get power_save); clock $(timedatectl | grep -i synchronized | tr -s ' ') $t s later"
