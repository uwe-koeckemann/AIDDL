package org.aiddl.common.java.learning.supervised;

import java.util.Map;

import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.FunctionGenerator;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

/**
 * Learn a model, apply it to the input data, and return the result vector
 * 
 * @author Uwe Koeckemann
 *
 */
public class LearnAndApplyFunction implements ConfigurableFunction {
	
	Function learner = null;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		learner = settings.get(Term.sym("learner")).asFunRef().getFunction();

		learner = fReg.getFunction(settings.get(Term.sym("learner")).asFunRef().getFunRefTerm());


		if ( !(learner instanceof FunctionGenerator) ) {
			throw new IllegalArgumentException("Learner ("+learner.getClass().getName()+") must implement FunctionGenerator to provide learned model as a function.");
		}
	}
	
	@Override
	public Term apply(Term mlProblem) {
		learner.apply(mlProblem);
		Function model = ((FunctionGenerator)learner).generate();
		
		System.out.println("Weights: " + model);

		LockableList resultVector = new LockableList();
		
		for ( Term data_point : mlProblem.get(Term.sym("data")).asCollection() ) {
			resultVector.add(model.apply(data_point));
		}
		return Term.list(resultVector);
	}
}
