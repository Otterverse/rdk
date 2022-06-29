// Code generated by protoc-gen-go-grpc. DO NOT EDIT.
// versions:
// - protoc-gen-go-grpc v1.2.0
// - protoc             (unknown)
// source: proto/api/service/slam/v1/slam.proto

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

// SLAMServiceClient is the client API for SLAMService service.
//
// For semantics around ctx use and closing/ending streaming RPCs, please refer to https://pkg.go.dev/google.golang.org/grpc/?tab=doc#ClientConn.NewStream.
type SLAMServiceClient interface {
	// GetPosition returns the current estimated position of the robot with
	// respect to the "origin" of the map.
	GetPosition(ctx context.Context, in *GetPositionRequest, opts ...grpc.CallOption) (*GetPositionResponse, error)
	// GetMap returns the latest map image or point cloud generated by the
	// SLAM library
	GetMap(ctx context.Context, in *GetMapRequest, opts ...grpc.CallOption) (*GetMapResponse, error)
}

type sLAMServiceClient struct {
	cc grpc.ClientConnInterface
}

func NewSLAMServiceClient(cc grpc.ClientConnInterface) SLAMServiceClient {
	return &sLAMServiceClient{cc}
}

func (c *sLAMServiceClient) GetPosition(ctx context.Context, in *GetPositionRequest, opts ...grpc.CallOption) (*GetPositionResponse, error) {
	out := new(GetPositionResponse)
	err := c.cc.Invoke(ctx, "/proto.api.service.slam.v1.SLAMService/GetPosition", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *sLAMServiceClient) GetMap(ctx context.Context, in *GetMapRequest, opts ...grpc.CallOption) (*GetMapResponse, error) {
	out := new(GetMapResponse)
	err := c.cc.Invoke(ctx, "/proto.api.service.slam.v1.SLAMService/GetMap", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

// SLAMServiceServer is the server API for SLAMService service.
// All implementations must embed UnimplementedSLAMServiceServer
// for forward compatibility
type SLAMServiceServer interface {
	// GetPosition returns the current estimated position of the robot with
	// respect to the "origin" of the map.
	GetPosition(context.Context, *GetPositionRequest) (*GetPositionResponse, error)
	// GetMap returns the latest map image or point cloud generated by the
	// SLAM library
	GetMap(context.Context, *GetMapRequest) (*GetMapResponse, error)
	mustEmbedUnimplementedSLAMServiceServer()
}

// UnimplementedSLAMServiceServer must be embedded to have forward compatible implementations.
type UnimplementedSLAMServiceServer struct {
}

func (UnimplementedSLAMServiceServer) GetPosition(context.Context, *GetPositionRequest) (*GetPositionResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetPosition not implemented")
}
func (UnimplementedSLAMServiceServer) GetMap(context.Context, *GetMapRequest) (*GetMapResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetMap not implemented")
}
func (UnimplementedSLAMServiceServer) mustEmbedUnimplementedSLAMServiceServer() {}

// UnsafeSLAMServiceServer may be embedded to opt out of forward compatibility for this service.
// Use of this interface is not recommended, as added methods to SLAMServiceServer will
// result in compilation errors.
type UnsafeSLAMServiceServer interface {
	mustEmbedUnimplementedSLAMServiceServer()
}

func RegisterSLAMServiceServer(s grpc.ServiceRegistrar, srv SLAMServiceServer) {
	s.RegisterService(&SLAMService_ServiceDesc, srv)
}

func _SLAMService_GetPosition_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(GetPositionRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(SLAMServiceServer).GetPosition(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.service.slam.v1.SLAMService/GetPosition",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(SLAMServiceServer).GetPosition(ctx, req.(*GetPositionRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _SLAMService_GetMap_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(GetMapRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(SLAMServiceServer).GetMap(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.service.slam.v1.SLAMService/GetMap",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(SLAMServiceServer).GetMap(ctx, req.(*GetMapRequest))
	}
	return interceptor(ctx, in, info, handler)
}

// SLAMService_ServiceDesc is the grpc.ServiceDesc for SLAMService service.
// It's only intended for direct use with grpc.RegisterService,
// and not to be introspected or modified (even as a copy)
var SLAMService_ServiceDesc = grpc.ServiceDesc{
	ServiceName: "proto.api.service.slam.v1.SLAMService",
	HandlerType: (*SLAMServiceServer)(nil),
	Methods: []grpc.MethodDesc{
		{
			MethodName: "GetPosition",
			Handler:    _SLAMService_GetPosition_Handler,
		},
		{
			MethodName: "GetMap",
			Handler:    _SLAMService_GetMap_Handler,
		},
	},
	Streams:  []grpc.StreamDesc{},
	Metadata: "proto/api/service/slam/v1/slam.proto",
}
