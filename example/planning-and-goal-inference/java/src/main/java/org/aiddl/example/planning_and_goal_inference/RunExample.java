package org.aiddl.example.planning_and_goal_inference;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.util.java.request.RequestHandler;
import org.aiddl.core.java.tools.Logger;

public class RunExample {
	public static void main(String[] args) {
		Container db = new Container();
		FunctionRegistry freg = DefaultFunctions.createDefaultRegistry(db);
		Parser.parseFile("../aiddl/plan-with-goal-graph.aiddl", db, freg);
		Logger.addPrintStream(System.out);

		RequestHandler server = new RequestHandler( DefaultFunctions.createDefaultRegistry(db) );
		CollectionTerm serviceConfigs = db.getEntry(Term.sym("service-configs")).getValue().asCollection();
		server.loadServices(serviceConfigs);

		Logger.addPrintStream(System.out);
		Entry main = db.getEntry(Term.sym("main"));
		server.satisfyRequest(main, db, db.getWorkingModule());
	}
}
