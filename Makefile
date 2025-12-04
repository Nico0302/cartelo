.PHONY: deps
deps:
	cd ~/ros2_ws \
	&& rosdep update \
	&& rosdep install --from-paths src --ignore-src -r -y

.PHONY: build
build:
	cd ~/ros2_ws \
    && colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUSE_HDIAPI=ON --symlink-install

.PHONY: resetvive
resetvive:
	rm -rf ~/.config/libsurvive/config.json