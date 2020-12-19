//nolint:golint
package nav_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetMapGoal struct {
	msg.Package `ros:"nav_msgs"`
}

type GetMapResult struct {
	msg.Package `ros:"nav_msgs"`
	Map         OccupancyGrid
}

type GetMapFeedback struct {
	msg.Package `ros:"nav_msgs"`
}
