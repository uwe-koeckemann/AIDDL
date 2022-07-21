package org.aiddl.example.learning_agent.functions;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.java.learning.LearningTerm;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;

public class StateTransitionDataExtractor implements ConfigurableFunction {

	private String name = StateTransitionDataExtractor.class.getSimpleName();
	boolean verbose = true;
		
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term args ) {
		if ( verbose ) Logger.msg(this.getClass().getSimpleName(), "Extracting Data...");
		
		ListTerm attributes = (ListTerm) args.get(LearningTerm.Attributes);
		SetTerm sNextTerm = (SetTerm) args.get(Term.sym("next-state"));
		SetTerm sTerm = (SetTerm) args.get(PlanningTerm.State);
		Term actionTerm = args.get(Term.sym("action"));
		
		SetTerm currentData = (SetTerm) args.get(LearningTerm.Data);
		SetTerm updatedData = null;
		
		Set<Term> effects = new LinkedHashSet<>();
		for ( Term p : sNextTerm ) {
//			if ( sNextTerm.get(p.getKey()).equals(p.getValue())) {
				effects.add(p);
//			}
		}

//		Map<Term, Term> sTerm_sva = PlanningTerm.convert2svaMap(sTerm);
		List<Term> newDataPoint = new ArrayList<Term>();
		
		for ( int i = 0 ; i < attributes.size()-2 ; i++ ) {
			TupleTerm atom = (TupleTerm) attributes.get(i);
			newDataPoint.add(sTerm.get(atom));
		}
		newDataPoint.add(actionTerm);
		newDataPoint.add( Term.set(effects) );
		ListTerm newDataPointTerm = Term.list(newDataPoint);
		updatedData = currentData.add(newDataPointTerm);
		if ( verbose ) {
			if ( updatedData.size() != currentData.size() ) {
				Logger.msg(name, "New data point: " + newDataPoint);
			} else {
				Logger.msg(name, "Nothing new: " + newDataPoint);
			}
			
			Logger.incDepth();
			for ( Term d : updatedData ) {
				Logger.msg(name, d.toString());	
			}
			Logger.decDepth();					
			
			Logger.msg(name, String.format( "|Data| = %d", updatedData.size()) );
		}

		return updatedData;
	}
}
