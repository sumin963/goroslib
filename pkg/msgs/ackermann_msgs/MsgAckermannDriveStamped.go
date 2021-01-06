package ackermann_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/sumin963/goroslib/pkg/msgs/ackermann_msgs"
)

type AckemannDriveStamped struct {
	msg.Package `ros:"ackermann_msgs"`
	Header      std_msgs.Header
	AckermannDrive ackermann_msgs.MsgAckemannDrive
}
