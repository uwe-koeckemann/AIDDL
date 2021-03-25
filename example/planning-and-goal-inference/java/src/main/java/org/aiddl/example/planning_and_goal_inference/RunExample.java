package org.aiddl.example.planning_and_goal_inference;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.service.RequestHandler;
import org.aiddl.core.tools.Logger;

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
