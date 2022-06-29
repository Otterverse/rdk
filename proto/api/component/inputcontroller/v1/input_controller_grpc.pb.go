// Code generated by protoc-gen-go-grpc. DO NOT EDIT.
// versions:
// - protoc-gen-go-grpc v1.2.0
// - protoc             (unknown)
// source: proto/api/component/inputcontroller/v1/input_controller.proto

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

// InputControllerServiceClient is the client API for InputControllerService service.
//
// For semantics around ctx use and closing/ending streaming RPCs, please refer to https://pkg.go.dev/google.golang.org/grpc/?tab=doc#ClientConn.NewStream.
type InputControllerServiceClient interface {
	// GetControls returns a list of GetControls provided by the Controller
	GetControls(ctx context.Context, in *GetControlsRequest, opts ...grpc.CallOption) (*GetControlsResponse, error)
	// GetEvents returns a list of events representing the last event on each control of a give Input Controller
	GetEvents(ctx context.Context, in *GetEventsRequest, opts ...grpc.CallOption) (*GetEventsResponse, error)
	// StreamEvents starts a stream of InputControllerEvents for the given controls (buttons/axes) on a robot's input controller
	StreamEvents(ctx context.Context, in *StreamEventsRequest, opts ...grpc.CallOption) (InputControllerService_StreamEventsClient, error)
	// TriggerEvent, where supported, injects an InputControllerEvent into an input controller to (virtually) generate events
	// like button presses or axis movements
	TriggerEvent(ctx context.Context, in *TriggerEventRequest, opts ...grpc.CallOption) (*TriggerEventResponse, error)
}

type inputControllerServiceClient struct {
	cc grpc.ClientConnInterface
}

func NewInputControllerServiceClient(cc grpc.ClientConnInterface) InputControllerServiceClient {
	return &inputControllerServiceClient{cc}
}

func (c *inputControllerServiceClient) GetControls(ctx context.Context, in *GetControlsRequest, opts ...grpc.CallOption) (*GetControlsResponse, error) {
	out := new(GetControlsResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.inputcontroller.v1.InputControllerService/GetControls", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *inputControllerServiceClient) GetEvents(ctx context.Context, in *GetEventsRequest, opts ...grpc.CallOption) (*GetEventsResponse, error) {
	out := new(GetEventsResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.inputcontroller.v1.InputControllerService/GetEvents", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *inputControllerServiceClient) StreamEvents(ctx context.Context, in *StreamEventsRequest, opts ...grpc.CallOption) (InputControllerService_StreamEventsClient, error) {
	stream, err := c.cc.NewStream(ctx, &InputControllerService_ServiceDesc.Streams[0], "/proto.api.component.inputcontroller.v1.InputControllerService/StreamEvents", opts...)
	if err != nil {
		return nil, err
	}
	x := &inputControllerServiceStreamEventsClient{stream}
	if err := x.ClientStream.SendMsg(in); err != nil {
		return nil, err
	}
	if err := x.ClientStream.CloseSend(); err != nil {
		return nil, err
	}
	return x, nil
}

type InputControllerService_StreamEventsClient interface {
	Recv() (*StreamEventsResponse, error)
	grpc.ClientStream
}

type inputControllerServiceStreamEventsClient struct {
	grpc.ClientStream
}

func (x *inputControllerServiceStreamEventsClient) Recv() (*StreamEventsResponse, error) {
	m := new(StreamEventsResponse)
	if err := x.ClientStream.RecvMsg(m); err != nil {
		return nil, err
	}
	return m, nil
}

func (c *inputControllerServiceClient) TriggerEvent(ctx context.Context, in *TriggerEventRequest, opts ...grpc.CallOption) (*TriggerEventResponse, error) {
	out := new(TriggerEventResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.inputcontroller.v1.InputControllerService/TriggerEvent", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

// InputControllerServiceServer is the server API for InputControllerService service.
// All implementations must embed UnimplementedInputControllerServiceServer
// for forward compatibility
type InputControllerServiceServer interface {
	// GetControls returns a list of GetControls provided by the Controller
	GetControls(context.Context, *GetControlsRequest) (*GetControlsResponse, error)
	// GetEvents returns a list of events representing the last event on each control of a give Input Controller
	GetEvents(context.Context, *GetEventsRequest) (*GetEventsResponse, error)
	// StreamEvents starts a stream of InputControllerEvents for the given controls (buttons/axes) on a robot's input controller
	StreamEvents(*StreamEventsRequest, InputControllerService_StreamEventsServer) error
	// TriggerEvent, where supported, injects an InputControllerEvent into an input controller to (virtually) generate events
	// like button presses or axis movements
	TriggerEvent(context.Context, *TriggerEventRequest) (*TriggerEventResponse, error)
	mustEmbedUnimplementedInputControllerServiceServer()
}

// UnimplementedInputControllerServiceServer must be embedded to have forward compatible implementations.
type UnimplementedInputControllerServiceServer struct {
}

func (UnimplementedInputControllerServiceServer) GetControls(context.Context, *GetControlsRequest) (*GetControlsResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetControls not implemented")
}
func (UnimplementedInputControllerServiceServer) GetEvents(context.Context, *GetEventsRequest) (*GetEventsResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetEvents not implemented")
}
func (UnimplementedInputControllerServiceServer) StreamEvents(*StreamEventsRequest, InputControllerService_StreamEventsServer) error {
	return status.Errorf(codes.Unimplemented, "method StreamEvents not implemented")
}
func (UnimplementedInputControllerServiceServer) TriggerEvent(context.Context, *TriggerEventRequest) (*TriggerEventResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method TriggerEvent not implemented")
}
func (UnimplementedInputControllerServiceServer) mustEmbedUnimplementedInputControllerServiceServer() {
}

// UnsafeInputControllerServiceServer may be embedded to opt out of forward compatibility for this service.
// Use of this interface is not recommended, as added methods to InputControllerServiceServer will
// result in compilation errors.
type UnsafeInputControllerServiceServer interface {
	mustEmbedUnimplementedInputControllerServiceServer()
}

func RegisterInputControllerServiceServer(s grpc.ServiceRegistrar, srv InputControllerServiceServer) {
	s.RegisterService(&InputControllerService_ServiceDesc, srv)
}

func _InputControllerService_GetControls_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(GetControlsRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(InputControllerServiceServer).GetControls(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.inputcontroller.v1.InputControllerService/GetControls",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(InputControllerServiceServer).GetControls(ctx, req.(*GetControlsRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _InputControllerService_GetEvents_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(GetEventsRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(InputControllerServiceServer).GetEvents(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.inputcontroller.v1.InputControllerService/GetEvents",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(InputControllerServiceServer).GetEvents(ctx, req.(*GetEventsRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _InputControllerService_StreamEvents_Handler(srv interface{}, stream grpc.ServerStream) error {
	m := new(StreamEventsRequest)
	if err := stream.RecvMsg(m); err != nil {
		return err
	}
	return srv.(InputControllerServiceServer).StreamEvents(m, &inputControllerServiceStreamEventsServer{stream})
}

type InputControllerService_StreamEventsServer interface {
	Send(*StreamEventsResponse) error
	grpc.ServerStream
}

type inputControllerServiceStreamEventsServer struct {
	grpc.ServerStream
}

func (x *inputControllerServiceStreamEventsServer) Send(m *StreamEventsResponse) error {
	return x.ServerStream.SendMsg(m)
}

func _InputControllerService_TriggerEvent_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(TriggerEventRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(InputControllerServiceServer).TriggerEvent(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.inputcontroller.v1.InputControllerService/TriggerEvent",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(InputControllerServiceServer).TriggerEvent(ctx, req.(*TriggerEventRequest))
	}
	return interceptor(ctx, in, info, handler)
}

// InputControllerService_ServiceDesc is the grpc.ServiceDesc for InputControllerService service.
// It's only intended for direct use with grpc.RegisterService,
// and not to be introspected or modified (even as a copy)
var InputControllerService_ServiceDesc = grpc.ServiceDesc{
	ServiceName: "proto.api.component.inputcontroller.v1.InputControllerService",
	HandlerType: (*InputControllerServiceServer)(nil),
	Methods: []grpc.MethodDesc{
		{
			MethodName: "GetControls",
			Handler:    _InputControllerService_GetControls_Handler,
		},
		{
			MethodName: "GetEvents",
			Handler:    _InputControllerService_GetEvents_Handler,
		},
		{
			MethodName: "TriggerEvent",
			Handler:    _InputControllerService_TriggerEvent_Handler,
		},
	},
	Streams: []grpc.StreamDesc{
		{
			StreamName:    "StreamEvents",
			Handler:       _InputControllerService_StreamEvents_Handler,
			ServerStreams: true,
		},
	},
	Metadata: "proto/api/component/inputcontroller/v1/input_controller.proto",
}
