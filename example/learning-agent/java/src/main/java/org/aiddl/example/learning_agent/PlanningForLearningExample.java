package org.aiddl.example.learning_agent;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.util.request.RequestHandler;
import org.aiddl.core.tools.Logger;

public class PlanningForLearningExample {
	public static void main( String[] args ) {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);

		Parser.parseFile("../aiddl/planning-for-learning.aiddl", db, fReg);
		System.out.println(db);

		RequestHandler server = new RequestHandler( DefaultFunctions.createDefaultRegistry(db) );
		CollectionTerm serviceConfigs = db.getEntry(Term.sym("service-configs")).getValue().asCollection();
		server.loadServices(serviceConfigs);
		
		Logger.addPrintStream(System.out);
				
		Entry main = db.getEntry(Term.sym("Main"));
		System.out.println(main);
		
		/**
		 * Uncomment to create a dot file from request
		 */
//		db.toggleNamespaces(false);
//		Request2Dot r2d = new Request2Dot();
//		r2d.compute(db.getEntry(Term.sym("Main")).getValue());
//		db.toggleNamespaces(true);

		server.satisfyRequest(main, db, db.getWorkingModule());
	}
}
