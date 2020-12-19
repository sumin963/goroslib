//nolint:golint
package actionlib

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TestGoal struct {
	msg.Package `ros:"actionlib"`
	Goal        int32
}

type TestResult struct {
	msg.Package `ros:"actionlib"`
	Result      int32
}

type TestFeedback struct {
	msg.Package `ros:"actionlib"`
	Feedback    int32
}
