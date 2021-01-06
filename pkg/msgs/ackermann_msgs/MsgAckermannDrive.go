package ackermann_msgs

import "github.com/aler9/goroslib/pkg/msg"

type MsgAckemannDrive struct {
	msg.Package 'ros:"ackermann_msgs"'
	steering_angel float32
	steering_angel_velocitiy float32
	speed float32
	acceleration float32
	jerk float32
}
