cbsps() {
    colcon build --symlink-install --packages-select "$@"
}

cbs() {
    colcon build --symlink-install
}
