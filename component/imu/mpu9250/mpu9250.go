// Package mpu9250 implements an mpu9250 9-axis IMU
package mpu9250

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	ahrs "github.com/tracktum/go-ahrs"
	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.uber.org/multierr"

	"gonum.org/v1/gonum/num/quat"

	"go.viam.com/rdk/component/board"
	"go.viam.com/rdk/component/imu"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

const model = "mpu9250"

func init() {
	registry.RegisterComponent(imu.Subtype, model, registry.Component{
		Constructor: func(
			ctx context.Context,
			r robot.Robot,
			config config.Component,
			logger golog.Logger,
		) (interface{}, error) {
			return NewIMU(ctx, r, config, logger)
		},
	})
}

type mpu struct {
	angularVelocity spatialmath.AngularVelocity
	orientation     spatialmath.Orientation
	acceleration    r3.Vector
	magnetometer    r3.Vector
	i2c             board.I2C
	logger          golog.Logger
	aScale          float64
	gScale          float64
	ahrs            ahrs.Madgwick

	mu              sync.Mutex
	cancelFunc              func()
	activeBackgroundWorkers sync.WaitGroup
}

func (i *mpu) ReadAngularVelocity(ctx context.Context) (spatialmath.AngularVelocity, error) {
	i.mu.Lock()
	defer i.mu.Unlock()
	return i.angularVelocity, nil
}

func (i *mpu) ReadOrientation(ctx context.Context) (spatialmath.Orientation, error) {
	i.mu.Lock()
	defer i.mu.Unlock()
	return i.orientation, nil
}

func (i *mpu) ReadAcceleration(ctx context.Context) (r3.Vector, error) {
	i.mu.Lock()
	defer i.mu.Unlock()
	return i.acceleration, nil
}

func (i *mpu) GetReadings(ctx context.Context) ([]interface{}, error) {
	return []interface{}{i.angularVelocity, i.orientation, i.acceleration}, nil
}

// NewIMU creates a new mpu9250 IMU
func NewIMU(ctx context.Context, r robot.Robot, config config.Component, logger golog.Logger) (imu.IMU, error) {

	pi, err := board.FromRobot(r, config.Attributes["board"].(string))
	if err != nil {
		return nil, err
	}


	b, ok := pi.(board.LocalBoard)
	if !ok {
		return nil, fmt.Errorf("%s is not a local board", config.Attributes["board"].(string))
	}


	i2c, ok := b.I2CByName(config.Attributes["bus"].(string))
	if  !ok {
		return nil, fmt.Errorf("can't find I2C bus: %s", config.Attributes["bus"].(string))
	}

	dev := &mpu{i2c: i2c, logger: logger, aScale: 2.0, gScale: 250.0}

	dev.ahrs = ahrs.NewMadgwick(0.1, 100) // 0.1 beta, 100hz update

	err = multierr.Combine(dev.startMPU(ctx), dev.startMag(ctx))
	if err != nil {
		return nil, err
	}


	var cancelCtx context.Context
	cancelCtx, dev.cancelFunc = context.WithCancel(ctx)
	waitCh := make(chan struct{})
	s := 0.01 // 100hz
	dev.activeBackgroundWorkers.Add(1)
	utils.PanicCapturingGo(func() {
		defer dev.activeBackgroundWorkers.Done()
		timer := time.NewTicker(time.Duration(s * float64(time.Second)))
		defer timer.Stop()
		close(waitCh)
		for {
			select {
			case <-cancelCtx.Done():
				return
			default:
			}
			select {
			case <-cancelCtx.Done():
				return
			case <-timer.C:
				err := dev.doRead(ctx)
				if err != nil {
					return
				}
			}
		}
	})
	<-waitCh

	return dev, nil
}


func (i *mpu) startMPU(ctx context.Context) error {
	i.mu.Lock()
	defer i.mu.Unlock()
	h, errs := i.i2c.OpenHandle(MPU6050_ADDR)
	if errs != nil {
		return errs
	}
	defer h.Close()

	errs = multierr.Combine(errs, h.WriteByteData(ctx, SMPLRT_DIV, 0))
	if utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	errs = multierr.Combine(errs, h.WriteByteData(ctx, PWR_MGMT_1, 0))
	if utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	errs = multierr.Combine(errs, h.WriteByteData(ctx, PWR_MGMT_1, 1))
	if utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	errs = multierr.Combine(errs, h.WriteByteData(ctx, CONFIG, 0))
	if utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	errs = multierr.Combine(errs, h.WriteByteData(ctx, GYRO_CONFIG, 0))
	if utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	errs = multierr.Combine(errs, h.WriteByteData(ctx, INT_PIN_CFG, 2))
	if utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	errs = multierr.Combine(errs, h.WriteByteData(ctx, INT_ENABLE, 1))
	if utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	return errs
}

func (i *mpu) startMag(ctx context.Context) error {
	i.mu.Lock()
	defer i.mu.Unlock()
	h, errs := i.i2c.OpenHandle(AK8963_ADDR)
	if errs != nil {
		return errs
	}
	defer h.Close()

	errs = multierr.Combine(errs, h.WriteByteData(ctx, AK8963_CNTL, 0))
	time.Sleep(100 * time.Millisecond)

	res  := byte(0b0001)  // 0b0001 = 16-bit
	rate := byte(0b0110)  // 0b0010 = 8 Hz, 0b0110 = 100 Hz

	mode := (res <<4)+rate // bit conversion

	errs = multierr.Combine(errs, h.WriteByteData(ctx, AK8963_CNTL, mode))
	if utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	return errs
}



func (i *mpu) Close() {
	i.cancelFunc()
	i.activeBackgroundWorkers.Wait()
}


func (i *mpu) doRead(ctx context.Context) error {
	i.mu.Lock()
	defer i.mu.Unlock()
	h, errs := i.i2c.OpenHandle(Address)
	if errs != nil {
		return errs
	}
	defer h.Close()

	x, err := i.readRawBits(ctx, ACCEL_XOUT_H)
	errs = multierr.Combine(errs, err)
	y, err := i.readRawBits(ctx, ACCEL_YOUT_H)
	errs = multierr.Combine(errs, err)
	z, err := i.readRawBits(ctx, ACCEL_ZOUT_H)
	errs = multierr.Combine(errs, err)
	i.acceleration = r3.Vector{X: i.scaleAcceleration(x), Y: i.scaleAcceleration(y), Z: i.scaleAcceleration(z)}


	x, err = i.readRawBits(ctx, GYRO_XOUT_H)
	errs = multierr.Combine(errs, err)
	y, err = i.readRawBits(ctx, GYRO_YOUT_H)
	errs = multierr.Combine(errs, err)
	z, err = i.readRawBits(ctx, GYRO_ZOUT_H)
	errs = multierr.Combine(errs, err)
	i.angularVelocity = spatialmath.AngularVelocity{X: i.scaleGyro(x), Y: i.scaleGyro(y), Z: i.scaleGyro(z)}

	if errs != nil {
		return errs
	}


	for {
		var errs error
		x, err := i.readRawBitsMag(ctx, HXH)
		errs = multierr.Combine(errs, err)
		y, err := i.readRawBitsMag(ctx, HYH)
		errs = multierr.Combine(errs, err)
		z, err := i.readRawBitsMag(ctx, HZH)
		errs = multierr.Combine(errs, err)

		if errs == nil {
			i.magnetometer = r3.Vector{X: i.scaleMag(x), Y: i.scaleMag(y), Z: i.scaleMag(z)}
			break
		}
	}

	q := i.ahrs.Update9D(
		i.angularVelocity.X * (math.Pi/180),
		i.angularVelocity.Y * (math.Pi/180),
		i.angularVelocity.Z * (math.Pi/180),
		i.acceleration.X,
		i.acceleration.Y,
		i.acceleration.Z,
		i.magnetometer.X / 100, // microteslas to gauss
		i.magnetometer.Y / 100,
		i.magnetometer.Z / 100,
	)

	// i.logger.Debugf("SMURF %+v", q)

	i.orientation = spatialmath.NewOrientationFromQuat(quat.Number{q[0], q[1], q[2], q[3]})
	return nil
}


func (i *mpu) scaleAcceleration(raw int16) float64 {
	// Default 16-bit range is +/- 2G
	// Scaling factor: g (in m/s^2) * scale range / math.MaxInt16
	return float64(raw) * ((9.8065 * i.aScale) / math.MaxInt16)
}

func (i *mpu) scaleGyro(raw int16) float64 {
	// Default 16-bit range is +/- 250 deg/s
	return float64(raw) * ((i.gScale) / math.MaxInt16)
}

func (i *mpu) scaleMag(raw int16) float64 {
	// Fixed scale, 0.15 uT/bit (4900 +/- / math.MaxInt16)
	return float64(raw) * 0.15
}

func (i *mpu) readRawBits(ctx context.Context, register uint8) (int16, error) {
	h, errs := i.i2c.OpenHandle(MPU6050_ADDR)
	if errs != nil {
		return 0, errs
	}
	defer h.Close()

	xH, err1 := h.ReadByteData(ctx, register)
	xL, err2 := h.ReadByteData(ctx, register+1)

	return int16((uint16(xH)<<8)|uint16(xL)), multierr.Combine(err1, err2)
}

func (i *mpu) readRawBitsMag(ctx context.Context, register uint8) (int16, error) {
	h, errs := i.i2c.OpenHandle(AK8963_ADDR)
	if errs != nil {
		return 0, errs
	}
	defer h.Close()

	xH, err1 := h.ReadByteData(ctx, register)
	xL, err2 := h.ReadByteData(ctx, register-1)

	status, err3 := h.ReadByteData(ctx, AK8963_ST2)
	if status != 0b10000 {
		return 0, fmt.Errorf("imu magnetometer overflow: %x", status)
	}

	return int16((uint16(xH)<<8)|uint16(xL)), multierr.Combine(err1, err2, err3)
}