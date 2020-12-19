//nolint:golint
package actionlib

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

const (
	TestRequestGoal_TERMINATE_SUCCESS   int32 = 0
	TestRequestGoal_TERMINATE_ABORTED   int32 = 1
	TestRequestGoal_TERMINATE_REJECTED  int32 = 2
	TestRequestGoal_TERMINATE_LOSE      int32 = 3
	TestRequestGoal_TERMINATE_DROP      int32 = 4
	TestRequestGoal_TERMINATE_EXCEPTION int32 = 5
)

type TestRequestGoal struct {
	msg.Package     `ros:"actionlib"`
	msg.Definitions `ros:"int32 TERMINATE_SUCCESS=0,int32 TERMINATE_ABORTED=1,int32 TERMINATE_REJECTED=2,int32 TERMINATE_LOSE=3,int32 TERMINATE_DROP=4,int32 TERMINATE_EXCEPTION=5"`
	TerminateStatus int32
	IgnoreCancel    bool
	ResultText      string
	TheResult       int32
	IsSimpleClient  bool
	DelayAccept     time.Duration
	DelayTerminate  time.Duration
	PauseStatus     time.Duration
}

type TestRequestResult struct {
	msg.Package    `ros:"actionlib"`
	TheResult      int32
	IsSimpleServer bool
}

type TestRequestFeedback struct {
	msg.Package `ros:"actionlib"`
}
