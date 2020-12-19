package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetMapGoal struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
}

type GetMapResult struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Map         OccupancyGrid //nolint:golint
}

type GetMapFeedback struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
}
