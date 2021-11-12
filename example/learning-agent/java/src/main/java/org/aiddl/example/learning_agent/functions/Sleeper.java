package org.aiddl.example.learning_agent.functions;

import java.util.Map;

import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;


public class Sleeper implements ConfigurableFunction {

	int steps = 1;
	
	private String name = Sleeper.class.getSimpleName();
	boolean verbose = true;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}
		
	@Override
	public Term apply( Term ms ) {
		try {
			Thread.sleep(ms.getIntValue());
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}
		
		if ( verbose ) {
			Logger.msg(name, "================================================================================");
			Logger.msg(name, "= Landmark " + steps++);
			Logger.msg(name, "================================================================================");
		}
		
		return null;
	}
}
