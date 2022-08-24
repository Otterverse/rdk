// Code generated by protoc-gen-go-grpc. DO NOT EDIT.
// versions:
// - protoc-gen-go-grpc v1.2.0
// - protoc             (unknown)
// source: proto/api/component/board/v1/board.proto

package v1

import (
	context "context"
	grpc "google.golang.org/grpc"
	codes "google.golang.org/grpc/codes"
	status "google.golang.org/grpc/status"
)

// This is a compile-time assertion to ensure that this generated file
// is compatible with the grpc package it is being compiled against.
// Requires gRPC-Go v1.32.0 or later.
const _ = grpc.SupportPackageIsVersion7

// BoardServiceClient is the client API for BoardService service.
//
// For semantics around ctx use and closing/ending streaming RPCs, please refer to https://pkg.go.dev/google.golang.org/grpc/?tab=doc#ClientConn.NewStream.
type BoardServiceClient interface {
	Status(ctx context.Context, in *StatusRequest, opts ...grpc.CallOption) (*StatusResponse, error)
	SetGPIO(ctx context.Context, in *SetGPIORequest, opts ...grpc.CallOption) (*SetGPIOResponse, error)
	// GetGPIO gets the high/low state of the given pin of a board of the underlying robot.
	GetGPIO(ctx context.Context, in *GetGPIORequest, opts ...grpc.CallOption) (*GetGPIOResponse, error)
	// PWM gets the duty cycle of the given pin of a board of the underlying robot.
	PWM(ctx context.Context, in *PWMRequest, opts ...grpc.CallOption) (*PWMResponse, error)
	// SetPWM sets the given pin of a board of the underlying robot to the given duty cycle.
	SetPWM(ctx context.Context, in *SetPWMRequest, opts ...grpc.CallOption) (*SetPWMResponse, error)
	// PWMFrequency gets the PWM frequency of the given pin of a board of the underlying robot.
	PWMFrequency(ctx context.Context, in *PWMFrequencyRequest, opts ...grpc.CallOption) (*PWMFrequencyResponse, error)
	// SetPWMFrequency sets the given pin of a board of the underlying robot to the given PWM frequency. 0 will use the board's default PWM frequency.
	SetPWMFrequency(ctx context.Context, in *SetPWMFrequencyRequest, opts ...grpc.CallOption) (*SetPWMFrequencyResponse, error)
	// ReadAnalogReader reads off the current value of an analog reader of a board of the underlying robot.
	ReadAnalogReader(ctx context.Context, in *ReadAnalogReaderRequest, opts ...grpc.CallOption) (*ReadAnalogReaderResponse, error)
	// GetDigitalInterruptValue returns the current value of the interrupt which is based on the type of interrupt.
	GetDigitalInterruptValue(ctx context.Context, in *GetDigitalInterruptValueRequest, opts ...grpc.CallOption) (*GetDigitalInterruptValueResponse, error)
}

type boardServiceClient struct {
	cc grpc.ClientConnInterface
}

func NewBoardServiceClient(cc grpc.ClientConnInterface) BoardServiceClient {
	return &boardServiceClient{cc}
}

func (c *boardServiceClient) Status(ctx context.Context, in *StatusRequest, opts ...grpc.CallOption) (*StatusResponse, error) {
	out := new(StatusResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/Status", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *boardServiceClient) SetGPIO(ctx context.Context, in *SetGPIORequest, opts ...grpc.CallOption) (*SetGPIOResponse, error) {
	out := new(SetGPIOResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/SetGPIO", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *boardServiceClient) GetGPIO(ctx context.Context, in *GetGPIORequest, opts ...grpc.CallOption) (*GetGPIOResponse, error) {
	out := new(GetGPIOResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/GetGPIO", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *boardServiceClient) PWM(ctx context.Context, in *PWMRequest, opts ...grpc.CallOption) (*PWMResponse, error) {
	out := new(PWMResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/PWM", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *boardServiceClient) SetPWM(ctx context.Context, in *SetPWMRequest, opts ...grpc.CallOption) (*SetPWMResponse, error) {
	out := new(SetPWMResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/SetPWM", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *boardServiceClient) PWMFrequency(ctx context.Context, in *PWMFrequencyRequest, opts ...grpc.CallOption) (*PWMFrequencyResponse, error) {
	out := new(PWMFrequencyResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/PWMFrequency", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *boardServiceClient) SetPWMFrequency(ctx context.Context, in *SetPWMFrequencyRequest, opts ...grpc.CallOption) (*SetPWMFrequencyResponse, error) {
	out := new(SetPWMFrequencyResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/SetPWMFrequency", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *boardServiceClient) ReadAnalogReader(ctx context.Context, in *ReadAnalogReaderRequest, opts ...grpc.CallOption) (*ReadAnalogReaderResponse, error) {
	out := new(ReadAnalogReaderResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/ReadAnalogReader", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *boardServiceClient) GetDigitalInterruptValue(ctx context.Context, in *GetDigitalInterruptValueRequest, opts ...grpc.CallOption) (*GetDigitalInterruptValueResponse, error) {
	out := new(GetDigitalInterruptValueResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.board.v1.BoardService/GetDigitalInterruptValue", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

// BoardServiceServer is the server API for BoardService service.
// All implementations must embed UnimplementedBoardServiceServer
// for forward compatibility
type BoardServiceServer interface {
	Status(context.Context, *StatusRequest) (*StatusResponse, error)
	SetGPIO(context.Context, *SetGPIORequest) (*SetGPIOResponse, error)
	// GetGPIO gets the high/low state of the given pin of a board of the underlying robot.
	GetGPIO(context.Context, *GetGPIORequest) (*GetGPIOResponse, error)
	// PWM gets the duty cycle of the given pin of a board of the underlying robot.
	PWM(context.Context, *PWMRequest) (*PWMResponse, error)
	// SetPWM sets the given pin of a board of the underlying robot to the given duty cycle.
	SetPWM(context.Context, *SetPWMRequest) (*SetPWMResponse, error)
	// PWMFrequency gets the PWM frequency of the given pin of a board of the underlying robot.
	PWMFrequency(context.Context, *PWMFrequencyRequest) (*PWMFrequencyResponse, error)
	// SetPWMFrequency sets the given pin of a board of the underlying robot to the given PWM frequency. 0 will use the board's default PWM frequency.
	SetPWMFrequency(context.Context, *SetPWMFrequencyRequest) (*SetPWMFrequencyResponse, error)
	// ReadAnalogReader reads off the current value of an analog reader of a board of the underlying robot.
	ReadAnalogReader(context.Context, *ReadAnalogReaderRequest) (*ReadAnalogReaderResponse, error)
	// GetDigitalInterruptValue returns the current value of the interrupt which is based on the type of interrupt.
	GetDigitalInterruptValue(context.Context, *GetDigitalInterruptValueRequest) (*GetDigitalInterruptValueResponse, error)
	mustEmbedUnimplementedBoardServiceServer()
}

// UnimplementedBoardServiceServer must be embedded to have forward compatible implementations.
type UnimplementedBoardServiceServer struct {
}

func (UnimplementedBoardServiceServer) Status(context.Context, *StatusRequest) (*StatusResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method Status not implemented")
}
func (UnimplementedBoardServiceServer) SetGPIO(context.Context, *SetGPIORequest) (*SetGPIOResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method SetGPIO not implemented")
}
func (UnimplementedBoardServiceServer) GetGPIO(context.Context, *GetGPIORequest) (*GetGPIOResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetGPIO not implemented")
}
func (UnimplementedBoardServiceServer) PWM(context.Context, *PWMRequest) (*PWMResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method PWM not implemented")
}
func (UnimplementedBoardServiceServer) SetPWM(context.Context, *SetPWMRequest) (*SetPWMResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method SetPWM not implemented")
}
func (UnimplementedBoardServiceServer) PWMFrequency(context.Context, *PWMFrequencyRequest) (*PWMFrequencyResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method PWMFrequency not implemented")
}
func (UnimplementedBoardServiceServer) SetPWMFrequency(context.Context, *SetPWMFrequencyRequest) (*SetPWMFrequencyResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method SetPWMFrequency not implemented")
}
func (UnimplementedBoardServiceServer) ReadAnalogReader(context.Context, *ReadAnalogReaderRequest) (*ReadAnalogReaderResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method ReadAnalogReader not implemented")
}
func (UnimplementedBoardServiceServer) GetDigitalInterruptValue(context.Context, *GetDigitalInterruptValueRequest) (*GetDigitalInterruptValueResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetDigitalInterruptValue not implemented")
}
func (UnimplementedBoardServiceServer) mustEmbedUnimplementedBoardServiceServer() {}

// UnsafeBoardServiceServer may be embedded to opt out of forward compatibility for this service.
// Use of this interface is not recommended, as added methods to BoardServiceServer will
// result in compilation errors.
type UnsafeBoardServiceServer interface {
	mustEmbedUnimplementedBoardServiceServer()
}

func RegisterBoardServiceServer(s grpc.ServiceRegistrar, srv BoardServiceServer) {
	s.RegisterService(&BoardService_ServiceDesc, srv)
}

func _BoardService_Status_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(StatusRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).Status(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/Status",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).Status(ctx, req.(*StatusRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BoardService_SetGPIO_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(SetGPIORequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).SetGPIO(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/SetGPIO",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).SetGPIO(ctx, req.(*SetGPIORequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BoardService_GetGPIO_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(GetGPIORequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).GetGPIO(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/GetGPIO",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).GetGPIO(ctx, req.(*GetGPIORequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BoardService_PWM_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(PWMRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).PWM(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/PWM",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).PWM(ctx, req.(*PWMRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BoardService_SetPWM_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(SetPWMRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).SetPWM(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/SetPWM",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).SetPWM(ctx, req.(*SetPWMRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BoardService_PWMFrequency_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(PWMFrequencyRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).PWMFrequency(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/PWMFrequency",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).PWMFrequency(ctx, req.(*PWMFrequencyRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BoardService_SetPWMFrequency_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(SetPWMFrequencyRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).SetPWMFrequency(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/SetPWMFrequency",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).SetPWMFrequency(ctx, req.(*SetPWMFrequencyRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BoardService_ReadAnalogReader_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(ReadAnalogReaderRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).ReadAnalogReader(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/ReadAnalogReader",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).ReadAnalogReader(ctx, req.(*ReadAnalogReaderRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BoardService_GetDigitalInterruptValue_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(GetDigitalInterruptValueRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BoardServiceServer).GetDigitalInterruptValue(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.board.v1.BoardService/GetDigitalInterruptValue",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BoardServiceServer).GetDigitalInterruptValue(ctx, req.(*GetDigitalInterruptValueRequest))
	}
	return interceptor(ctx, in, info, handler)
}

// BoardService_ServiceDesc is the grpc.ServiceDesc for BoardService service.
// It's only intended for direct use with grpc.RegisterService,
// and not to be introspected or modified (even as a copy)
var BoardService_ServiceDesc = grpc.ServiceDesc{
	ServiceName: "proto.api.component.board.v1.BoardService",
	HandlerType: (*BoardServiceServer)(nil),
	Methods: []grpc.MethodDesc{
		{
			MethodName: "Status",
			Handler:    _BoardService_Status_Handler,
		},
		{
			MethodName: "SetGPIO",
			Handler:    _BoardService_SetGPIO_Handler,
		},
		{
			MethodName: "GetGPIO",
			Handler:    _BoardService_GetGPIO_Handler,
		},
		{
			MethodName: "PWM",
			Handler:    _BoardService_PWM_Handler,
		},
		{
			MethodName: "SetPWM",
			Handler:    _BoardService_SetPWM_Handler,
		},
		{
			MethodName: "PWMFrequency",
			Handler:    _BoardService_PWMFrequency_Handler,
		},
		{
			MethodName: "SetPWMFrequency",
			Handler:    _BoardService_SetPWMFrequency_Handler,
		},
		{
			MethodName: "ReadAnalogReader",
			Handler:    _BoardService_ReadAnalogReader_Handler,
		},
		{
			MethodName: "GetDigitalInterruptValue",
			Handler:    _BoardService_GetDigitalInterruptValue_Handler,
		},
	},
	Streams:  []grpc.StreamDesc{},
	Metadata: "proto/api/component/board/v1/board.proto",
}