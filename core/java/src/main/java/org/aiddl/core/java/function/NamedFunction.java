package org.aiddl.core.java.function;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;

public class NamedFunction implements Function {
	
	private Term name;	
	private Term f;
	private Term args = null;
	private Evaluator e;
	
	public NamedFunction( Term name, Term f, Evaluator e ) {
		this.name = name;
		this.f = f;
		this.e = e;
	}
	
	public NamedFunction( Term name, Term args, Term f, Evaluator e ) {
		this.name = name;
		this.f = f;
		this.e = e;
		this.args = args;
	}

	@Override
	public Term apply(Term args) {
		Substitution s;
		if ( this.args == null ) {
			s = new Substitution();
			s.add(Term.sym("#self"), args);
			s.add(Term.sym("#arg"), args);
		} else {
			s = this.args.match(args);
		}
		return e.apply(f.substitute(s));
	}
	
	@Override
	public String toString() {
		return f.toString();
	}
}
