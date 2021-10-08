package org.aiddl.core.function;

import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;

public class Lambda implements Function {

	private Term f;
	private Term x;
	private Evaluator e;
	
	public Lambda( Term x, Term f, Evaluator e ) {
		this.x = x;		
		this.f = f;
		this.e = e;
	}

	@Override
	public Term apply(Term args) {
		Substitution s = this.x.match(args);		
		return e.apply(f.substitute(s));
	}
	
	@Override
	public String toString() {
		return "(org.aiddl.eval.lambda " + this.x + " " + this.f + ")";
	}
}
