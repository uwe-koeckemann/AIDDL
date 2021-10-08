package org.aiddl.example.planning_and_goal_inference;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.util.request.Request2Dot;
import org.aiddl.util.request.RequestHandler;
import org.aiddl.core.tools.Logger;

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
