package org.aiddl.common.optimization.combinatorial.tsp;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.math.graph.Graph;
import org.aiddl.common.math.graph.GraphTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class TspMinRemainder implements Function, InitializableFunction, ConfigurableFunction, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.optimization.combinatorial.tsp.heuristic");
	
	Term weights;
	SetTerm N;
	
	FunctionRegistry fReg;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.fReg = fReg; 
	}
	
	@Override
	public void initialize(Term args) {	
		N =  args.get(GraphTerm.Nodes).asSet();
		weights = args.get(GraphTerm.Weights);
	}

	@Override
	public Term apply(Term args) {
		Set<Term> N_has_in = new HashSet<>();
		Set<Term> N_has_out = new HashSet<>();
		
		for ( Term e : args.asCollection() ) {
			Term in = e.get(1).getKey();
			Term out = e.get(1).getValue();
			
			N_has_in.add(out);
			N_has_out.add(in);
		}
		
		NumericalTerm w_best = Term.integer(0);
		
		for ( Term n1 : N ) {
			if ( !N_has_out.contains(n1) ) {
				NumericalTerm w_best_n = Term.infPos();
				for ( Term n2 : N ) {
					if ( !n1.equals(n2) && !N_has_in.contains(n2) ) {
						NumericalTerm w = Graph.getWeight(weights, n1, n2, fReg); //weights.get(Term.set(n1, n2));
						if ( w.lessThan(w_best_n)) {
							w_best_n = w;
						}
					}
				}
				w_best = w_best.add(w_best_n);
			}
		}
		return w_best;
	}

	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
}
