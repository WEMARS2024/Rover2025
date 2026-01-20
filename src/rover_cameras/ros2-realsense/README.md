Docker Container for realsense camera
Runs processess in different tmux panes

# How to use
1. Install docker for your OS
2. Build the Dockerfile using build.sh or the docker desktop app
3. Run the container once built (default name: ros2-realsense)
4. You are done! the ros2 packages will automatically run on different tmux panes, and will open on an open terminal
    - go to localhost:8080 for video output on browser

If you need to see the outputs of each pane, then use `Ctrl+b, then n` to go to the next pane, or `Ctrl+b, then <number>` to go to a specific numbered pane. Google "tmux" if that is not enough for you
