package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Vector3 struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	X           float64 //nolint:golint
	Y           float64 //nolint:golint
	Z           float64 //nolint:golint
}
