cat test.sh
!/bin/bash
# Commands to run in each terminal
commands=(
  "echo 'Hello from Terminal 1'"
  "ls -l"
  "date"
  "ping -c 3 google.com"
)
# Open a new terminal window for each command
for ((i=0; i<${#commands[@]}; i++))
do
  gnome-terminal --tab --title="Terminal $((i+1))" -- bash -c "${commands[i]}; exec bash"
done