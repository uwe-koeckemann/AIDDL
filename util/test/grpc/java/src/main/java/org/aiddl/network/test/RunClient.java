package org.aiddl.network.test;

import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;

import org.aiddl.network.GrpcFunction;

public class RunClient {
	
	public static void main( String[] args ) {
	    GrpcFunction f = new GrpcFunction("localhost", 8011, Term.sym("org.aiddl.eval.numerical.add"));
	    
	    Term arg = Parser.ParseTerm("(1 2 3)");
	    
	    System.out.println(f.apply(arg));
	}	
}
