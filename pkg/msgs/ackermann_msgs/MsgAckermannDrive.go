package ackermann_msgs //nolint:golint

import "github.com/sumin963/goroslib/pkg/msg"

type AckemannDrive struct {
	msg.Package `ros:"ackermann_msgs"`
	steering_angel float32
	steering_angel_velocitiy float32
	speed float32
	acceleration float32
	jerk float32
}
