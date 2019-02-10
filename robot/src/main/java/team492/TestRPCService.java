package team492;

import jsonrpc.core.annotation.JsonRpcMethod;
import jsonrpc.core.annotation.JsonRpcService;

@JsonRpcService
public interface TestRPCService 
{
	@JsonRpcMethod("getMajiraInstance")
	public TestRPCClass getMajiraInstance();
}
