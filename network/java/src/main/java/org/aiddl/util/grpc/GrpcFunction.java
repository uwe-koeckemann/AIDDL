package org.aiddl.util.grpc;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.network.AiddlGrpc.AiddlBlockingStub;
import org.aiddl.network.AiddlOuterClass.FunctionCallRequest;
import org.aiddl.network.AiddlOuterClass.FunctionCallResult;

import io.grpc.Channel;
import io.grpc.ManagedChannelBuilder;
import io.grpc.StatusRuntimeException;

/**
 * Wrap function around grpc call.
 * 
 * @author Uwe Koeckemann
 *
 */
public class GrpcFunction implements Function {

	String ip;
	int port;
	SymbolicTerm function_uri;

	private final AiddlBlockingStub blockingStub;
	
	public GrpcFunction( String host, int port, SymbolicTerm function_uri ) {
		  this(ManagedChannelBuilder.forAddress(host, port).usePlaintext().build(), function_uri);
		}

	public GrpcFunction( Channel channel, SymbolicTerm function_uri ) {
		blockingStub = AiddlGrpc.newBlockingStub(channel);
		this.function_uri = function_uri;		
	}

	@Override
	public Term apply(Term args) {
		FunctionCallRequest call_args = FunctionCallRequest.newBuilder().
				setFunctionUri(this.function_uri.toString()).
				setArg(args.toString()).
				build();

		FunctionCallResult result;
		try {
			result = blockingStub.functionCall(call_args);
			Term r = Parser.ParseTerm(result.getResult());
			return r;
		} catch (StatusRuntimeException e) {
			System.err.println(e);
			System.exit(1);
		}
		return null;
	}
}


