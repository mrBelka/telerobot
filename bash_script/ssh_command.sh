#!/usr/bin/expect -f

set timeout -1

set user [lindex $argv 0]
set host [lindex $argv 1]
set password [lindex $argv 2]
set command [lindex $argv 3]

spawn ssh $user@$host

expect "password:"

send "$password\r"

expect "$ "

send "$command\r"

interact

