package org.aiddl.core.java.unit_test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.Term;

/** Class for running AIDDL unit tests (#assert type entries)
 * @author Uwe Koeckemann
 *
 */
public class RunTests {

	/** Find all #assert type entries in container and evaluate them as unit tests.
	 * @param db container for testing
	 * @param eval arbiter containing required evaluators
	 * @param verbose verbose setting for evaluators
	 * @return RationalTerm representing the number of successful tests divided by total number of tests 
	 */
	public static Term run(Container db, Evaluator eval, boolean verbose ) {
		int num_tests = 0;
		int num_success = 0;
		
/*		RequestHandler rHandler = new RequestHandler(fReg);
//		rHandler.setVerbose(true);
		for ( Entry req : db.getMatchingEntries(Term.anonymousVar(), Term.sym("#assert-request"), Term.anonymousVar())) {
			Term request_term = req.getValue().get(0);
			Term exec_module = req.getValue().get(1).resolve(db);
			rHandler.satisfyRequest(request_term, db, exec_module);
		}*/
		
		boolean gotAllEvals = true;
						
		if ( gotAllEvals ) {
		
			Collection<Entry> tests = db.getMatchingEntries(null, Term.sym("#assert"), null);
			for ( Entry e : tests ) {				
				if ( e.getValue() instanceof ListTerm ) {
					int i = 1;
					for ( Term t : e.getValue().asList() ) {
						num_tests++;
						
						boolean pass = single_test( e.getName() + " " + i, t, eval, verbose );
						if ( pass ) {
							num_success++;
						}
						i++;
					}
				} else {
					num_tests++;
					
					boolean pass = single_test( e.getName().toString(), e.getValue(), eval, verbose );
					if ( pass ) {
						num_success++;
					}
				}
			}
			Term result = Term.rational(num_success, num_tests);
			if ( verbose ) {
				System.out.println("Successful: " + result);
			}
			
			return result;
		}
		return Term.sym("skipped");
	}
	
	public static boolean single_test( String label, Term test, Evaluator eval, boolean verbose ) {
		Term result = eval.apply(test);
		
		if ( result.equals(Term.bool(true))) {
			if ( verbose ) System.out.println(String.format("Test %s: ok %s", label, test));
			return true; //num_success++;
		} else {
			eval.setVerbose(2);
			result = eval.apply(test);
			System.err.println(String.format("Test %s: FAILURE %s", label, test));
			System.err.println("Result: " + result);
			eval.setVerbose(0);
			return false;
		}
	}
	
	/** Test a single file. 
	 * @param fName name of file to test
	 * @return <code>true</code> if all unit tests succeeded, <code>false</code> otherwise
	 */
	public static boolean testFile( String fName ) {
		List<String> fNames = new ArrayList<String>();
		fNames.add(fName);
		return testFiles(fNames);
	}
	
	/** Test a single file. 
	 * @param fName name of file to test
	 * @return <code>true</code> if all unit tests succeeded, <code>false</code> otherwise
	 */
	public static boolean testFile( String fName, Container db ) {
		List<String> fNames = new ArrayList<String>();
		fNames.add(fName);
		return testFiles(fNames, db);
	}
	
	/** Test a list of files. 
	 * @param fNames names of file to test
	 * @return <code>true</code> if all unit tests succeeded, <code>false</code> otherwise
	 */
	public static boolean testFiles( List<String> fNames ) {
		Container db = new Container( );
		for ( String fName : fNames )
			Parser.parseFile(fName, db);

		Term testResult = RunTests.run(db, db.evaluator(), true);
			
		return testResult.equals(Term.rational(1, 1));
	}
	
	/** Test a list of files. 
	 * @param fNames names of file to test
	 * @return <code>true</code> if all unit tests succeeded, <code>false</code> otherwise
	 */
	public static boolean testFiles( List<String> fNames, Container db ) {
		for ( String fName : fNames )
			Parser.parseFile(fName, db);

		/*FunctionRegistry fRegInternal = DefaultFunctions.createDefaultRegistry(db);
		for ( Term function_name : fReg.getRegisteredNames() ) {
			fRegInternal.addFunctionIfAbsent(function_name, fReg.getFunction(function_name));
		}*/
		
		Term testResult = RunTests.run(db, db.evaluator(), true);
			
		return testResult.equals(Term.rational(1, 1));
	}
	
}
