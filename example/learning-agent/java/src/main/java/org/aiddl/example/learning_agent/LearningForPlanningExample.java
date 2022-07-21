package org.aiddl.example.learning_agent;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.util.java.request.RequestHandler;
import org.aiddl.core.java.tools.Logger;

public class LearningForPlanningExample {
	public static void main( String[] args ) {

		Container db = new Container();
		Parser.parseFile("../aiddl/learning-for-planning.aiddl", db);
		System.out.println(db);

		RequestHandler server = new RequestHandler( DefaultFunctions.createDefaultRegistry(db) );
		CollectionTerm serviceConfigs = db.getEntry(Term.sym("service-configs")).getValue().asCollection();
		server.loadServices(serviceConfigs);
		
		Logger.addPrintStream(System.out);
		Entry main = db.getEntry(Term.sym("Main"));
		
		/**
		 * Uncomment to create a dot file from request.
		 */
//		db.toggleNamespaces(false);
//		Request2Dot r2d = new Request2Dot();
//		db.toggleNamespaces(true);
//		r2d.compute(main.getValue());
				
		server.satisfyRequest(main, db, db.getWorkingModule());
	}
}
