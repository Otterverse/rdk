package main

import (
	"context"
	"flag"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/utils"

	"go.viam.com/rdk/config"
	"go.viam.com/rdk/component/imu"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/metadata/service"
	robotimpl "go.viam.com/rdk/robot/impl"
	"go.viam.com/rdk/services/web"
	webserver "go.viam.com/rdk/web/server"
)


func main() {
	logger := golog.NewDevelopmentLogger("stumbler")
	utils.ContextualMain(mainWithArgs, logger)
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) (err error) {
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
	go debugOut(ctx, myRobot)

	options := web.NewOptions()
	options.Network = cfg.Network
	return webserver.RunWeb(ctx, myRobot, options, logger)
}




func debugOut(ctx context.Context, r robot.Robot) {
	i, err := imu.FromRobot(r, "imu")
	if err != nil {
		return
	}

	for {
		if !utils.SelectContextOrWait(ctx, time.Second) {
			return
		}

		accel, _ := i.ReadAcceleration(ctx)
		orient, _ := i.ReadOrientation(ctx)
		spin, _ := i.ReadAngularVelocity(ctx)

		r.Logger().Debugf("Readings:\nAccel: %+v\nOrient: %+v\nSpin: %+v", accel, orient.OrientationVectorDegrees(), spin)

	}
}