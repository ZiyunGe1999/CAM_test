all:
	@echo nothing special

enter_q1_env:
	docker run --rm \
		-v `pwd`/Q1-Registration:/registration \
		-w /registration \
		-it ros:noetic bash