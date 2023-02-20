all:
	@echo nothing special

enter_q1_env:
	docker run --rm \
		-v `pwd`/Q1-Registration:/registration \
		-w /registration \
		-it ros:noetic bash

enter_q2_env:
	docker run --rm \
		-v `pwd`/Q2-PointCloud_Alignment:/Q2-PointCloud_Alignment \
		-w /Q2-PointCloud_Alignment \
		-it ros:noetic bash