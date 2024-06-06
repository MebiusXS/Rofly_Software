#!/usr/bin/expect
set timeout 30
spawn bash /home/fast/mavros.sh
# spawn bash /home/fast/Downloads/code/Rofly_Software/Onboard_codes/start_script.sh
expect "password"
send "fast\r"
interact
