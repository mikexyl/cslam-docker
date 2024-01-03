.PHONY: build-swarm-slam build-kimera-multi build-decoslam build-d2slam

build-swarm-slam:
	@docker build -t swarm-slam -f swarm-slam/Dockerfile\
		--progress=plain \
		.

build-d2slam:
	@cd d2slam/D2SLAM && git apply ../d2slam_dockerfile.patch
	@docker build -t d2slam \
		-f d2slam/D2SLAM/docker/Dockerfile\
		--progress plain \
		./d2slam/D2SLAM
	
build-kimera-multi:
	@echo $(shell cat /home/mikexyl/.ssh/id_ed25519.base64)
	@docker build -t kimera-multi -f kimera-multi/Dockerfile \
		--build-arg SSH_PRIVATE_KEY_BASE64="$(shell cat /home/mikexyl/.ssh/id_ed25519.base64)" \
		.

build-decoslam:
	@docker build -t decoslam -f decoslam/Dockerfile .

build-ros-ros2-bridge:
	@docker build -t ros-ros2-bridge -f tools/Dockerfile.ros-ros2-bridge .
	
run-kimera-multi:
	@docker run -it --rm \
		--name kimera-multi \
		--cap-add=NET_ADMIN \
		-u root \
		--mount type=bind,source=/datasets,target=/datasets \
		kimera-multi bash
	
run-d2slam:
	@docker run -it --rm \
		--name d2slam \
		--cap-add=NET_ADMIN \
		-u root \
		d2slam bash
	
run-ros-ros2-bridge:
	@docker run -it --rm \
		--name ros-ros2-bridge \
		-u root \
		--mount type=bind,source=/home/mikexyl/workspaces/datasets,target=/datasets \
		ros-ros2-bridge bash