package org.aiddl.example.planning_and_goal_inference;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.util.java.request.Request2Dot;
import org.aiddl.util.java.request.RequestHandler;
import org.aiddl.core.java.tools.Logger;

public class RunExampleProlog {
	public static void main(String[] args) {
		Container db = new Container();
		FunctionRegistry freg = DefaultFunctions.createDefaultRegistry(db);
		Parser.parseFile("../aiddl/plan-and-reasoning.aiddl", db, freg);
		Logger.addPrintStream(System.out);
		
		RequestHandler server = new RequestHandler( DefaultFunctions.createDefaultRegistry(db) );
		CollectionTerm serviceConfigs = db.getEntry(Term.sym("service-configs")).getValue().asCollection();
		server.loadServices(serviceConfigs);

		Entry main = db.getEntry(Term.sym("main"));
		
		server.satisfyRequest(main, db, db.getWorkingModule());
		/**
		 * Uncomment to create a dot file from request.
		 */
//		db.toggleNamespaces(false);
//		Request2Dot r2d = new Request2Dot();
//		r2d.compute(db.getEntry(Term.sym("main")).getValue());
	}
}
