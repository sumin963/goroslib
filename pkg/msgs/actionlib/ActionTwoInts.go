//nolint:golint
package actionlib

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TwoIntsGoal struct {
	msg.Package `ros:"actionlib"`
	A           int64
	B           int64
}

type TwoIntsResult struct {
	msg.Package `ros:"actionlib"`
	Sum         int64
}

type TwoIntsFeedback struct {
	msg.Package `ros:"actionlib"`
}
