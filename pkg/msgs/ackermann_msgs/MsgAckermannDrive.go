package ackermann_msgs //nolint:golint

import "github.com/sumin963/goroslib/pkg/msg"

type AckemannDrive struct {
	msg.Package `ros:"ackermann_msgs"`
	Steering_angel float32
	Steering_angel_velocitiy float32
	Speed float32
	Acceleration float32
	Jerk float32
}
