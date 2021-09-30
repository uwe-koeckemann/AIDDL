package org.aiddl.network.test;

import java.io.IOException;

import org.aiddl.core.container.Container;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.network.AiddlGrpcServer;

public class RunServer {

	public static void main(String[] args) throws IOException, InterruptedException {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		AiddlGrpcServer server = new AiddlGrpcServer(8011, fReg, true);
		server.start();
		
		while ( true ) {
			Thread.sleep(500);
		}
	}

}
