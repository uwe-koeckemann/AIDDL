package org.aiddl.common.java.optimization.combinatorial.tsp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.java.math.graph.Graph;
import org.aiddl.common.java.math.graph.GraphTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.TermComparator;

public class TspExpandPath implements Function, InitializableFunction, ConfigurableFunction, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.optimization.combinatorial.tsp.expansion");
	
	
	SetTerm N;
	
	Term weights;
	FunctionRegistry fReg;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.fReg = fReg;
	}
	
	@Override
	public void initialize(Term G) {
		weights = G.get(GraphTerm.Weights);
		N =  G.get(GraphTerm.Nodes).asSet();
	}

	@Override
	public Term apply(Term args) {
		ListTerm path = args.asList();
		
//		NumericalTerm w = Term.integer(0);
		
		Set<Term> non_options = new HashSet<>();
		for ( Term e : path ) {
//			NumericalTerm w_step = weights.get(Term.set(e.get(1).getKey(), e.get(1).getValue()));
//			w = w.add(w_step);
			non_options.add(e.get(1).getKey());
			non_options.add(e.get(1).getValue());
		}
				
		List<Term> steps = new ArrayList<>();
		
//		if ( path.isEmpty() ) {
//			for ( Term next : N ) {
//				steps.add(Term.tuple(Term.integer(0), next));
//			}
//		}

		if ( path.size() == N.size()-1 ) {
			Term first = path.get(0).get(1).getKey();
			Term last = path.getLast().get(1).getValue();
			NumericalTerm w_final = Graph.getWeight(this.weights, last, first, fReg);
			steps.add(Term.tuple(w_final, Term.keyVal(last, first)));
		} else {
			
//			NumericalTerm acc_weight = path.get(path.size()-1).get(0).asNum();
			Term last;
			
			if ( !path.isEmpty() ) {
				last = path.get(path.size()-1).get(1).getValue();
			} else {
				last = N.iterator().next();
			}
			
			non_options.add(last);
			
			for ( Term next : N ) {
				if ( !non_options.contains(next) ) {
					NumericalTerm w_step = Graph.getWeight(this.weights, last, next, fReg);
					steps.add(Term.tuple(w_step, Term.keyVal(last , next)));
				}
			}
		}
		
		Collections.sort(steps, new TermComparator());		
		return Term.list(steps);
	}

	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
}
