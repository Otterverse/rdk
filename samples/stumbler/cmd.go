package main

import (
	"context"
	"flag"
	"time"
	"math"
	"sync"

	"github.com/edaniels/golog"
	"go.viam.com/utils"

	"go.viam.com/rdk/action"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/component/imu"
	"go.viam.com/rdk/component/motor"

	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/metadata/service"
	robotimpl "go.viam.com/rdk/robot/impl"
	"go.viam.com/rdk/services/web"
	webserver "go.viam.com/rdk/web/server"

	"go.einride.tech/pid"

	// "net/http"
	// _ "net/http/pprof"

)

var workers sync.WaitGroup
var cancelFunc func()
var balancing bool
var balancingMu sync.Mutex


var bP, bI, bD, sP, sI, sD, awG float64

func main() {
	logger := golog.NewDevelopmentLogger("stumbler")
	utils.ContextualMain(mainWithArgs, logger)
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) (err error) {

	// workers.Add(1)
	// go func() {
	// 	defer workers.Done()
	// 	logger.Debug(http.ListenAndServe(":6060", nil))
	// }()


	flag.Float64Var(&bP, "bP", 0.1, "Balance kP")
	flag.Float64Var(&bI, "bI", 0, "Balance kI")
	flag.Float64Var(&bD, "bD", 0.05, "Balance kD")

	flag.Float64Var(&sP, "sP", 4.0, "Speed kP")
	flag.Float64Var(&sI, "sI", 8.0, "Speed kI")
	flag.Float64Var(&sD, "sD", 6.0, "Speed kD")

	flag.Float64Var(&awG, "awG", 0, "Antiwindup Gain")

	flag.Parse()

	cfg, err := config.Read(ctx, flag.Arg(0), logger)
	if err != nil {
		return err
	}

	metadataSvc, err := service.New()
	if err != nil {
		return err
	}
	ctx = service.ContextWithService(ctx, metadataSvc)

	myRobot, err := robotimpl.New(ctx, cfg, logger)
	if err != nil {
		return err
	}
	defer myRobot.Close(ctx)

	action.RegisterAction("startBalance", startBalance)
	action.RegisterAction("stopBalance", stopBalance)


	// cancelCtx, debugCancelFunc := context.WithCancel(ctx)
	// workers.Add(1)
	// go debugOut(cancelCtx, myRobot)

	options := web.NewOptions()
	options.Network = cfg.Network
	err = webserver.RunWeb(ctx, myRobot, options, logger)

	stopBalance(ctx, myRobot)
	//debugCancelFunc()
	workers.Wait()
	return err
}

func startBalance(ctx context.Context, r robot.Robot) {
	balancingMu.Lock()
	defer balancingMu.Unlock()
	if balancing { return }
	var cancelCtx context.Context
	cancelCtx, cancelFunc = context.WithCancel(ctx)
	workers.Add(1)
	go balance(cancelCtx, r)
	balancing = true
}

func stopBalance(ctx context.Context, r robot.Robot) {
	balancingMu.Lock()
	defer balancingMu.Unlock()
	if cancelFunc != nil {
		cancelFunc()
	}
	workers.Wait()
	balancing = false
}


func balance(ctx context.Context, r robot.Robot) {
	defer workers.Done()
	i, err := imu.FromRobot(r, "imu")
	if err != nil {
		return
	}

	motorR, err := motor.FromRobot(r, "motorR")
	if err != nil {
		return
	}

	motorL, err := motor.FromRobot(r, "motorL")
	if err != nil {
		return
	}

	defer motorL.Stop(ctx)
	defer motorR.Stop(ctx)

	balance := pid.TrackingController{
		Config: pid.TrackingControllerConfig{
			ProportionalGain: bP, //0.1
			IntegralGain:     bI, //0.0
			DerivativeGain:   bD, //0.001
			AntiWindUpGain:   0, //0
			MaxOutput: 1.0,
			MinOutput: -1.0,
			LowPassTimeConstant: time.Second,
		},
	}

	speed := pid.TrackingController{
		Config: pid.TrackingControllerConfig{
			ProportionalGain: sP, //5.0
			IntegralGain:     sI, //0
			DerivativeGain:   sD, //0
			AntiWindUpGain:   awG, //0
			MaxOutput: 10,
			MinOutput: -10,
			LowPassTimeConstant: time.Second,
		},
	}

	// turn := pid.Controller{
	// 	Config: pid.ControllerConfig{
	// 		ProportionalGain: 2.5,
	// 		IntegralGain:     0,
	// 		DerivativeGain:   0.5,
	// 	},
	// }



	timer := time.NewTicker(time.Duration(10 * time.Millisecond))
	defer timer.Stop()
	var tick uint
	prevLPos, _ := motorL.GetPosition(ctx)
	prevRPos, _ := motorR.GetPosition(ctx)
	var prevSpeed float64
	startTime := time.Now()
	for {
		tick++
		select {
		case <-ctx.Done():
			return
		default:
		}
		select {
		case <-ctx.Done():
			return
		case <-timer.C:
		}

		if tick % 100 == 0 {
			r.Logger().Debug("AvgTime: ", time.Now().Sub(startTime) / time.Duration(tick) )
		}


		orientation, _ := i.ReadOrientation(ctx)
		o := orientation.EulerAngles()

		if math.Abs(o.Pitch * 180/math.Pi) > 30 || math.Abs(balance.State.UnsaturatedControlSignal) > 1.1 {
			balancing = false
			return
		}


		if tick % 5 == 0 {

			newLPos, _ := motorL.GetPosition(ctx)
			newRPos, _ := motorR.GetPosition(ctx)

			lSpeed := (newLPos - prevLPos) / 0.05
			rSpeed := (newRPos - prevRPos) / 0.05

			prevLPos, prevRPos = newLPos, newRPos

			// Filter the speed over time a tad more
			curSpeed := prevSpeed * 0.7 + ((lSpeed + rSpeed) / 2) * 0.3
			prevSpeed = curSpeed

			speed.Update(pid.TrackingControllerInput{
				ReferenceSignal:  0,
				ActualSignal:     curSpeed,
				SamplingInterval:  50 * time.Millisecond,
			})
			r.Logger().Debugf("SPID: %+v", speed.State)
			r.Logger().Debugf("Out: %.2f, Speed: %.2f, Pitch: %.2f",speed.State.ControlSignal, curSpeed, o.Pitch * 180/math.Pi)
		}


		balance.Update(pid.TrackingControllerInput{
			ReferenceSignal:  speed.State.ControlSignal,
			ActualSignal:     o.Pitch * 180/math.Pi,
			SamplingInterval: 10 * time.Millisecond,
		})

		//r.Logger().Debugf("Bal: %+v", balance.State)

		motorL.SetPower(ctx, (balance.State.ControlSignal) * -1)
		motorR.SetPower(ctx, (balance.State.ControlSignal) * -1)

	}
}


func debugOut(ctx context.Context, r robot.Robot) {
	defer workers.Done()
	// i, err := imu.FromRobot(r, "imu")
	// if err != nil {
	// 	return
	// }



	// motorR, err := motor.FromRobot(r, "motorR")
	// if err != nil {
	// 	return
	// }

	// motorL, err := motor.FromRobot(r, "motorL")
	// if err != nil {
	// 	return
	// }


	// for {
	// 	if !utils.SelectContextOrWait(ctx, time.Second) {
	// 		return
	// 	}

	// 	accel, _ := i.ReadAcceleration(ctx)
	// 	orient, _ := i.ReadOrientation(ctx)
	// 	spin, _ := i.ReadAngularVelocity(ctx)



	// 	r.Logger().Debugf("Speed R: %v, L: %v", rSpeed, lSpeed)

	// }
}