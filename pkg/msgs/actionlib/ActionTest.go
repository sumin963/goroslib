package actionlib //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TestGoal struct { //nolint:golint
	msg.Package `ros:"actionlib"`
	Goal        int32 //nolint:golint
}

type TestResult struct { //nolint:golint
	msg.Package `ros:"actionlib"`
	Result      int32 //nolint:golint
}

type TestFeedback struct { //nolint:golint
	msg.Package `ros:"actionlib"`
	Feedback    int32 //nolint:golint
}
