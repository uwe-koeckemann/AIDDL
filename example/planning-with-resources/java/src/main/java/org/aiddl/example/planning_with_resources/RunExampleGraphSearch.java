package org.aiddl.example.planning_with_resources;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.util.java.request.RequestHandler;
import org.aiddl.core.java.tools.Logger;
import org.aiddl.core.java.tools.Profiler;
import org.aiddl.core.java.tools.StopWatch;

public class RunExampleGraphSearch {
	public static void main( String[] args ) {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
				
		Term data_module;
		if ( args.length > 0 && args[0].equals("d1") ) {
			data_module = Parser.parseFile("../aiddl/domain-01.aiddl", db, fReg);	
		} else if ( args.length > 0 && args[0].equals("d2") ) {
			data_module = Parser.parseFile("../aiddl/domain-02.aiddl", db, fReg);	
		} else if ( args.length > 0 && args[0].equals("d3") ) {
			data_module = Parser.parseFile("../aiddl/domain-03.aiddl", db, fReg);		
		} else {
			data_module = Parser.parseFile("../aiddl/domain-01.aiddl", db, fReg);	
		}
		
		
		Term planner_module = Parser.parseFile("../aiddl/planner.aiddl", db, fReg);
		Term run_module = Term.sym("run-module");		
		db.addModule(run_module);
		
		RequestHandler server = new RequestHandler( fReg );
		
//		server.setVerbose(true);
				
		Term s0 = db.getEntry(data_module, Term.sym("s0")).getValue();
		Term g = db.getEntry(data_module, Term.sym("g")).getValue();
		Term O = db.getEntry(data_module, Term.sym("O")).getValue();		
		Term cap = db.getEntry(data_module, Term.sym("cap")).getValue();
		
		Entry main = db.getEntry(planner_module, Term.tuple(Term.sym("main"), s0, g, O, cap, run_module));
		
		/**
		 * Uncomment to create a dot file from request.
		 */
//		db.toggleNamespaces(false);
//		Request2Dot r2d = new Request2Dot();
//		r2d.compute( db.getEntry(planner_module, Term.tuple(Term.sym("plan"), Term.anonymousVar(), Term.anonymousVar(), Term.anonymousVar(), run_module)).getValue());
//		db.toggleNamespaces(true);
		
		
		fReg.loadContainerDefintions(db);
		server.satisfyRequest(main, db, run_module);

		System.out.println(Profiler.getString());
		System.out.println(StopWatch.allSums2Str());	
	}
}
