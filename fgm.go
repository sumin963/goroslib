package main

import (
	"bufio"
	"encoding/csv"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/ackermann_msgs"
	"github.com/aler9/goroslib/pkg/msgs/nav_msgs"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"math"
	"os"
	"strconv"
	"time"
)

type ICE_fgm struct {
	fgm interface{}
	ackermann_data ackermann_msgs.AckermannDriveStamped

	PI float64
	rf_distance float64
	RACECAR_LENGTH float64
	SPEED_MAX float64
	SPEED_MIN float64
	RATE float64
	ROBOT_SCALE float64
	THRESHOLD float64
	GAP_SIZE int
	FILTER_SCALE float64
	GAP_THETA_GAIN float64
	REF_THETA_GAIN float64

	scan_Hi []int

	scan_range int
	desired_gap []int
	speed_gain float64
	steering_gain float64
	gain_cont int
	speed_cont int
	desired_wp_rt []float64  //타입 맞는지 확

	speed_up int

	wp_num int
	waypoints [][]float64
	wp_index_current int
	current_position []float64
	nearest_distance float64

	max_angle float64
	wp_angle float64

	gaps [][]int
	for_gap []int
	for_point int
	interval float64
	front_idx int
	theta_for float64
	gap_cont int

	scan_origin []float64
	scan_filtered []float64
	scan_HI []float64

	pub *goroslib.Publisher
	err error
}

func newICE_fgm() *ICE_fgm {
	I := ICE_fgm{}
	fgm,err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "fgm",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer fgm.Close()

	I.PI =3.141592
	I.rf_distance = 2.5
	I.RACECAR_LENGTH = 0.325
	I.SPEED_MAX = 5.0
	I.SPEED_MIN = 2.0
	I.RATE = 100
	I.ROBOT_SCALE = 0.35
	I.THRESHOLD = 2.3
	I.GAP_SIZE = 1
	I.FILTER_SCALE = 1.1
	I.GAP_THETA_GAIN = 20.0
	I.REF_THETA_GAIN = 1.0

	I.scan_range = 0
	I.desired_gap = []int{}
	I.speed_gain = 0
	I.steering_gain = 0
	I.gain_cont = 0
	I.speed_cont = 0
	//desired_wp_rt [2]int

	I.speed_up =0

	I.wp_num =1
	I.waypoints = I.get_waypoint()
	I.wp_index_current =0
	I.current_position = []float64{}
	I.nearest_distance =0

	I.max_angle =0
	I.wp_angle =0

	I.gaps = [][]int{}
	I.for_gap =[]int{}
	I.for_point =0
	I.interval =0.00435
	I.front_idx =0
	I.theta_for = I.PI / 3
	I.gap_cont =0

	odomsub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     fgm,
		Topic:    "/odom",
		Callback: I.Odome,
	})
	if err != nil {
		panic(err)
	}
	defer odomsub.Close()

	lssub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     fgm,
		Topic:    "/scan",
		Callback: I.subCallback_scan,
	})
	if err != nil {
		panic(err)
	}
	defer lssub.Close()

	I.pub, I.err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  fgm,
		Topic: "/drive",
		Msg:   &ackermann_msgs.AckermannDriveStamped{},
	})
	if I.err != nil {
		panic(err)
	}

	defer I.pub.Close()

	I.ackermann_data.Drive.Speed = 0
	I.ackermann_data.Drive.Acceleration = 0
	I.ackermann_data.Drive.Jerk = 0
	I.ackermann_data.Drive.SteeringAngle = 0
	I.ackermann_data.Drive.SteeringAngleVelocity = 0


	return &I
}
func (i *ICE_fgm)get_waypoint() [][]float64 {
	file_wps,_ := os.Open("../f1tenth_ws/src/sumin/wp_vegas.csv")
	rdr := csv.NewReader(bufio.NewReader(file_wps))
	rows,_ := rdr.ReadAll()
	var temp_waypoint [][]float64
	for i, row := range rows {
		for j := range row {
			s, _:= strconv.ParseFloat(rows[i][j], 32)
			temp_waypoint[i][j] = s
		}
	}
	return temp_waypoint
}
func (i *ICE_fgm)find_desired_wp()  {
	wp_index_temp := i.wp_index_current
	i.nearest_distance = i.getDistance(i.waypoints[wp_index_temp],i.current_position)
	for true{
		wp_index_temp+=1
		if wp_index_temp >= i.wp_num-1{
			wp_index_temp = 0
		}
		temp_distance := i.getDistance(i.waypoints[wp_index_temp], i.current_position)
		if temp_distance < i.nearest_distance{
			i.nearest_distance = temp_distance
			i.wp_index_current = wp_index_temp
		}else if ((temp_distance > (i.nearest_distance + i.rf_distance*1.2)) || (wp_index_temp == i.wp_index_current)){
			break
		}
	}

	var temp_distance float64
	idx_temp := i.wp_index_current
	for true{
		if idx_temp >= i.wp_num-1{
			idx_temp = 0
		}
		temp_distance = i.getDistance(i.waypoints[idx_temp], i.current_position)
		if temp_distance > i.rf_distance{
			break
		}
		idx_temp+=1
	}
	transformed_nearest_point := i.transformPoint(i.current_position, i.waypoints[idx_temp])
	i.desired_wp_rt = i.xyt2rt(transformed_nearest_point)
}

func(i *ICE_fgm)find_gap(scan []float64){
	i.gaps = [][]int{}
	j:=0
	for j < i.scan_range - i.GAP_SIZE {
		if scan[j] > float64(i.THRESHOLD){
			start_idx_temp := j
			end_idx_temp := j
			max_temp := scan[j]
			max_idx_temp := j

			for k := 1;  k <=int(i.THRESHOLD); k++{
				if scan[j] > i.THRESHOLD{
					if scan[j] > max_temp{
						max_temp = scan[j]
						max_idx_temp = j
					}else {
						max_temp = -1
						break
					}
				}
			}
			if max_temp== -1{
				break
			}else{
				for ((scan[j] > i.THRESHOLD) && (j+1 < i.scan_range )){
					j+=1
					if scan[j] > max_temp{
						max_temp = scan[j]
						max_idx_temp = j
					}
				}
				j += 1
				end_idx_temp = j
				var gap_temp [3]int
				gap_temp[0] = start_idx_temp
				gap_temp[1] = end_idx_temp
				gap_temp[2] = max_idx_temp
				gaps_len := len(i.gaps)
				i.gaps[gaps_len][0] = gap_temp[0]
				i.gaps[gaps_len][1] = gap_temp[0]
				i.gaps[gaps_len][2] = gap_temp[0]
			}
		}
		j += 1
	}
}

func (i *ICE_fgm)getDistance( a []float64, b []float64) float64{
	dx := a[0] - b[0]
	dy := a[1] - b[1]
	result := math.Sqrt(math.Pow(dx,2) + math.Pow(dy,2) )
	return result
}

func (i *ICE_fgm) main_drive(goal []int) {
	i.max_angle = float64(goal[2] - i.front_idx)*i.interval
	i.wp_angle = i.desired_wp_rt[1]

	temp_avg := 0.0
	dmin := 0.0
	for j:=0; j<=9;j++{
		dmin += i.scan_filtered[j]
	}
	dmin /= 10

	j := 0
	for j < i.scan_range -7{
		k:=0
		for k < 10{
			if j+k > 1079{
				temp_avg +=0
			}else{
				temp_avg += i.scan_filtered[j+k]
			}
			k+=1
		}
		temp_avg /= 10

		if dmin > temp_avg{
			if temp_avg ==0{
				temp_avg = dmin
			}
			dmin = temp_avg
		}
		temp_avg = 0
		j += 3
	}
	if dmin ==0{
		dmin =1
	}
	controlled_angle := ( (i.GAP_THETA_GAIN/dmin)*i.max_angle + i.REF_THETA_GAIN*i.wp_angle)/(i. GAP_THETA_GAIN/dmin + i.REF_THETA_GAIN)
	distance := 1.5
	path_radius := distance / (2*math.Sin(controlled_angle))
	steering_angle := math.Atan(i.RACECAR_LENGTH/path_radius)

	M := 0
	for j:= 180; j<=900; j++{
		if i.scan_HI[j] < 15{
			M+=1
		}
	}
	a := i.scan_HI[0]
	b := i.scan_HI[180]
	c := i.scan_HI[360]
	d := i.scan_HI[540]
	e := i.scan_HI[720]
	f := i.scan_HI[900]
	g := i.scan_HI[1079]

	A := 0
	B := 0

	if d > 20{
		if (f*math.Sqrt(2) - 0.1 <= e) && (e <= f*math.Sqrt(2) + 0.1) && (f*math.Sqrt(2) - 0.1 <= g) && (g <= f*math.Sqrt(2) + 0.1) && (M < 680){
			B = 1
		}else if  (b*math.Sqrt(2) - 0.1 <= c) && (c <= b*math.Sqrt(2) + 0.1) && (b*math.Sqrt(2) - 0.1 <= a) && (a <= b*math.Sqrt(2) + 0.1) && (M < 680){
			A = 1
		}else {
		}
	}

	var speed float64
	if math.Abs(steering_angle) > i.PI/8{
		speed = i.SPEED_MIN
		i.speed_gain = 0
	}else{
		speed = float64(-(3/i.PI))*(i.SPEED_MAX-i.SPEED_MIN)*math.Abs(i.max_angle)+i.SPEED_MAX
		speed = math.Abs(speed)
	}

	accel := speed - i.speed_gain
	if accel < i.SPEED_MIN{
		accel = i.SPEED_MIN
	}
	if i.speed_up ==1{
		i.speed_up = 0
	}
	if math.Abs(steering_angle) < math.Abs(i.steering_gain){
		i.speed_gain =0
	}

	steering := steering_angle + i.steering_gain

	for j:= 537; j <= 544; j++{
		if i.scan_HI[j] < 13{
			accel = 8
			if i.scan_HI[j] < 7{
				accel = 6.5
				if i.scan_HI[j] < 5{
					accel = 5
					if i.scan_HI[j] < 3{
						accel = 2
						if i.scan_HI[j] < 1{
							accel = 1
						}
					}
				}
			}
		}
	}

	if (A ==1 || B ==1) && (i.scan_HI[540] > 4){
		accel = 12
		steering = 0
	}

	i.ackermann_data.Drive.SteeringAngle = float32(steering)
	i.ackermann_data.Drive.SteeringAngleVelocity = 0
	i.ackermann_data.Drive.Speed = float32(accel)
	i.ackermann_data.Drive.Acceleration= 0
	i.ackermann_data.Drive.Jerk = 0
	i.pub.Write(i.ackermann_data)
	//pub
	i.speed_gain = 0
	i.steering_gain = 0
	i.gain_cont = 0
}


func (i *ICE_fgm)subCallback_scan(msg *sensor_msgs.LaserScan) {
	//i.scan_angle_min = msg_sub.angle_min
	//i.scan_angle_max = msg_sub.angle_max
	i.interval = float64(msg.AngleIncrement)
	i.scan_range = len(msg.Ranges)
	i.front_idx = i.scan_range/2

	for j := 1; j <= i.scan_range; j++{
		i.scan_origin = append(i.scan_origin, 0)
		i.scan_filtered = append(i.scan_filtered, 0)
		i.scan_HI = append(i.scan_HI, 0)
	}
	for j := 1; j <= i.scan_range; j++{
		if i.scan_origin[j] == 0{
			var cont float64 =0
			var sum float64
			for k := 1; k <= 20; k++{
				if j -k >=0{
					if i.scan_origin[j-k] != 0{
						cont += 1
						sum += i.scan_origin[j-k]
					}
				}
				if j+k < i.scan_range{
					if i.scan_origin[j+k] != 0{
						cont += 1
						sum += i.scan_origin[j+k]
					}
				}
			}
			i.scan_origin[j] = sum/cont
			i.scan_filtered[j] = sum/cont
		}
	}
	for j := 0; j<= i.scan_range - 1;{
		if i.scan_origin[j]*i.FILTER_SCALE < i.scan_filtered[j+1]{
			unit_length := i.scan_origin[j]*i.interval
			filter_num := i.ROBOT_SCALE/unit_length

			k:=1
			for float64(k) < filter_num+1{
				if j+k < i.scan_range{
					if i.scan_filtered[j+k] > i.scan_origin[j]{
						i.scan_filtered[j+k] = i.scan_origin[j]
					}else {
						break
					}
				}else {
					break
				}
				k+=1
			}
		}else if i.scan_filtered[j] > i.scan_origin[j+1]*i.FILTER_SCALE{
			unit_length := i.scan_origin[j+1]*i.interval
			filter_num := i.ROBOT_SCALE / unit_length

			k := 0
			for float64(k) < filter_num +1 {
				if j-k >0{
					if i.scan_filtered[j-k] > i.scan_origin[j+1]{
						i.scan_filtered[j-k] = i.scan_origin[j+1]
					}else {
						break
					}
				}else {
					break
				}
				k += 1
			}
		}
	}
}


func (i *ICE_fgm)Odome(msg *nav_msgs.Odometry) {
	qx:= msg.Pose.Pose.Orientation.X
	qy:= msg.Pose.Pose.Orientation.Y
	qz:= msg.Pose.Pose.Orientation.Z
	qw:= msg.Pose.Pose.Orientation.W

	siny_cosp := 2.0 * (qw*qz + qx*qy)
	cosy_cosp := 1.0-2.0*(qy*qy + qz*qz)

	current_position_theta:= math.Atan2(siny_cosp, cosy_cosp)
	current_position_x := msg.Pose.Pose.Position.X
	current_position_y := msg.Pose.Pose.Position.Y
	i.current_position = append(i.current_position, current_position_x)
	i.current_position = append(i.current_position, current_position_y)
	i.current_position = append(i.current_position, current_position_theta)
	i.find_desired_wp()
}

func (i *ICE_fgm)GAP (){
	if i.gap_cont <=1{
		i.for_point = int(i.theta_for/i.interval)
		i.for_gap[0] = i.front_idx - i.for_point
		i.for_gap[1] = i.front_idx + i.for_point

		i.gap_cont += 1
	}
}

func (i *ICE_fgm)for_find_gap(scan []float64){
	i.for_point = int(i.theta_for/i.interval)
	start_idx_temp := i.front_idx - i.for_point
	end_idx_temp := i.front_idx + i.for_point

	max_idx_temp := start_idx_temp
	max_temp := scan[start_idx_temp]

	for j := start_idx_temp; j <= end_idx_temp; j++{
		if max_temp < scan[j]{
			max_temp = scan[j]
			max_idx_temp = j
		}
	}
	i.for_gap[0] = start_idx_temp
	i.for_gap[1] = end_idx_temp
	i.for_gap[2] = max_idx_temp
}

func (i *ICE_fgm)xyt2rt(origin []float64) []float64 {
	var rtpoint []float64

	x:= origin[0]
	y:= origin[1]

	rtpoint = append(rtpoint, math.Sqrt(x*x + y*y))
	rtpoint = append(rtpoint, math.Atan2(y,x) - (i.PI/2))
	return rtpoint
}

func (i *ICE_fgm)transformPoint(origin []float64, target []float64) []float64{
	theta:= i.PI - origin[2]

	dx := target[0] - origin[0]
	dy := target[1] - origin[1]
	dtheta := target[2] + theta

	tf_point_x := dx * math.Cos(theta) - dy * math.Sin(theta)
	tf_point_y := dx * math.Sin(theta) + dy * math.Cos(theta)
	tf_point_theta := dtheta
	var tf_point []float64
	tf_point = append(tf_point, tf_point_x)
	tf_point = append(tf_point, tf_point_y)
	tf_point = append(tf_point, tf_point_theta)

	return tf_point
}

func (i *ICE_fgm)find_best_gap(ref []float64)  []int{
	num := len(i.gaps)
	if num ==0{
		return i.for_gap
	}else{
		step := int(ref[1]/i.interval)
		ref_idx := i.front_idx + step

		gap_idx := 0

		distance := 0
		temp_distance := 0
		if i.gaps[0][0] > ref_idx{
			distance = i.gaps[0][0] - ref_idx
		}else if i.gaps[0][1]< ref_idx{
			distance = ref_idx - i.gaps[0][1]
		}else{
			distance =0
			gap_idx=0
		}

		j := 1
		for j < num{
			if i.gaps[j][0] > ref_idx{
				temp_distance = i.gaps[j][1] -ref_idx
				if temp_distance < distance{
					distance = temp_distance
					gap_idx = 1
				}
			}else if i.gaps[j][1] < ref_idx{
				temp_distance = ref_idx - i.gaps[j][1]
				if temp_distance < distance{
					distance = temp_distance
					gap_idx = 1
				}
			}else{
				temp_distance = 0
				distance = 0
				gap_idx = j
				break
			}
			j+=1
		}
		return i.gaps[gap_idx]
	}
}

func (i *ICE_fgm)driving(){
	for{ //조건 추가
		if i.scan_range == 0{
			continue
		}
		i.GAP()
		i.find_gap(i.scan_filtered)
		i.for_find_gap(i.scan_filtered)
		i.desired_gap = i.find_best_gap(i.desired_wp_rt)
		i.main_drive(i.desired_gap)

		time.Sleep(100*time.Millisecond)
	}
}
func main() {
	// create a node and connect to the master
	A:=newICE_fgm()
	A.driving()
	select {}
}
