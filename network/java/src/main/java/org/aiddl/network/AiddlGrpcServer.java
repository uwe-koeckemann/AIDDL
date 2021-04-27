package org.aiddl.network;

import java.io.IOException;
import java.util.concurrent.TimeUnit;
import java.util.logging.Logger;

import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.network.AiddlOuterClass.FunctionCallRequest;
import org.aiddl.network.AiddlOuterClass.FunctionCallResult;
import org.aiddl.network.AiddlOuterClass.FunctionCallResult.Builder;

import io.grpc.Server;
import io.grpc.ServerBuilder;
import io.grpc.stub.StreamObserver;

public class AiddlGrpcServer {
	private final Server server;

	public AiddlGrpcServer(int port, FunctionRegistry freg, boolean allow_remote_registry) throws IOException {
		if ( allow_remote_registry ) {
			freg.addFunction(Term.sym("org.aiddl.network.register-remote-function"), new GrpcFunctionLoader(freg));
		}
		
		server = ServerBuilder.forPort(port).addService(new AiddlService(freg))
				.build();
	}

	/** Start serving requests. */
	public void start() throws IOException {
		server.start();
//		logger.info("Server started, listening on " + port);
		Runtime.getRuntime().addShutdownHook(new Thread() {
			@Override
			public void run() {
				// Use stderr here since the logger may have been reset by its JVM shutdown hook.
				System.err.println("*** shutting down gRPC server since JVM is shutting down");
				try {
					AiddlGrpcServer.this.stop();
				} catch (InterruptedException e) {
					e.printStackTrace(System.err);
				}
				System.err.println("*** server shut down");
			}
		});
	}

	/** Stop serving requests and shutdown resources. */
	public void stop() throws InterruptedException {
		if (server != null) {
			server.shutdown().awaitTermination(30, TimeUnit.SECONDS);
		}
	}

	private static class AiddlService extends AiddlGrpc.AiddlImplBase {
		private FunctionRegistry freg;

		public AiddlService( FunctionRegistry  freg ) {
			this.freg = freg;
		}

		/**
		 *
		 * @param request the requested location for the feature.
		 * @param responseObserver the observer that will receive the feature at the requested point.
		 */
		@Override
		public void functionCall(FunctionCallRequest request, StreamObserver<FunctionCallResult> responseObserver) {
			responseObserver.onNext(runPlanner(request));
			responseObserver.onCompleted();
		}

		private FunctionCallResult runPlanner(FunctionCallRequest problem) {
			Term function_uri = Parser.ParseTerm(problem.getFunctionUri());
			Term function_arg = Parser.ParseTerm(problem.getArg());

			System.out.println("======= Request =======");
			System.out.println(function_uri);
			System.out.println(function_arg);
			
			Function f = this.freg.getFunction(function_uri);
			Term r = f.apply(function_arg);
			Builder s; 
			System.out.println(r);
			
			s = FunctionCallResult.newBuilder().setStatus(0);

			s.setResult(r.toString());

			return s.build();
		}
	}	
}
