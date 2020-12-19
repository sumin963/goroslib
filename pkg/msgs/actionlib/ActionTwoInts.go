package actionlib //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TwoIntsGoal struct { //nolint:golint
	msg.Package `ros:"actionlib"`
	A           int64 //nolint:golint
	B           int64 //nolint:golint
}

type TwoIntsResult struct { //nolint:golint
	msg.Package `ros:"actionlib"`
	Sum         int64 //nolint:golint
}

type TwoIntsFeedback struct { //nolint:golint
	msg.Package `ros:"actionlib"`
}
