package org.aiddl.common.java.automata;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.util.java.request.RequestHandler;
import org.aiddl.core.java.tools.Logger;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestAutomata extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	

	public void testAdvanceState() {
		Container db = new Container();
		String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");
		Parser.parseFile(aiddlTestStr + "/automata/dfa-01.aiddl", db);
				
		Logger.addPrintStream(System.out);
		RequestHandler server = new RequestHandler(  db.getFunctionRegistry() );
		CollectionTerm serviceConfigs = db.getEntry(Term.sym("service-configs")).getValue().asCollection();
		server.loadServices(serviceConfigs);
		server.setEnforceTypeCheck(true);

		server.satisfyRequest(db.getEntry(Term.sym("main")), db, db.getWorkingModule());
			
		Term s_new = db.getEntry(Term.sym("s")).getValue();
		Term is_final = db.getEntry(Term.sym("is-final-state")).getValue();
		
		assertTrue( s_new.equals(Term.sym("s2")));
		assertTrue( is_final.getBooleanValue() );
	}
}
