.PHONY: build-swarm-slam build-kimera-multi build-decoslam build-d2slam

build-swarm-slam:
	@docker build -t swarm-slam -f swarm-slam/Dockerfile .

build-d2slam:
	@docker build -t d2slam -f d2slam/Dockerfile .
	
build-kimera-multi:
	@echo $(shell cat /home/mikexyl/.ssh/id_ed25519.base64)
	@docker build -t kimera-multi -f kimera-multi/Dockerfile \
		--build-arg SSH_PRIVATE_KEY_BASE64="$(shell cat /home/mikexyl/.ssh/id_ed25519.base64)" \
		.

build-decoslam:
	@docker build -t decoslam -f decoslam/Dockerfile .
	
run-kimera-multi:
	@docker run -it --rm \
		--name kimera-multi \
		--cap-add=NET_ADMIN \
		-u root \
		--mount type=bind,source=$(HOME)/workspaces/datasets,target=/datasets \
		kimera-multi bash