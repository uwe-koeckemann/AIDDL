package org.aiddl.common.java.planning.state_variable.data;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.java.math.graph.GraphTerm;
import org.aiddl.common.java.planning.state_variable.heuristic.CausalGraphHeuristic;
import org.aiddl.common.java.planning.state_variable.Operator;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.LockableSet;

public class DomainTransitionGraphCreator implements Function {

	@Override
	public Term apply(Term O) {
		Map<Term, Set<Term>> domains = new HashMap<>();
		Set<Term> variables = new HashSet<>();
		for ( Term o_term : O.asCollection() ) {
			Operator o = new Operator(o_term.asTuple());
			for ( Term e : o.getEffects() ) {
				domains.putIfAbsent(e.getKey(), new HashSet<>());
				domains.get(e.getKey()).add(e.getValue());
				variables.add( e.getKey() );
			}
			for ( Term p : o.getPreconditions() ) {
				domains.putIfAbsent(p.getKey(), new HashSet<>());
				domains.get(p.getKey()).add(p.getValue());
				variables.add( p.getKey() );
			}
		}
		LockableSet DTGs = new LockableSet();
		
		for ( Term variable : variables ) {
			DTGs.add(Term.keyVal(variable, compute(O, variable, domains.get(variable))));
		}
		
		return Term.set(DTGs);
	}
	

	public Term compute(Term O, Term variable, Set<Term> domain ) {
		LockableSet V = new LockableSet();
		LockableList E = new LockableList();
		LockableSet L = new LockableSet();
		
		for ( Term o_term : O.asCollection() ) {
			Operator o = new Operator(o_term.asTuple());
			
			Term preVal = o.getPreconditions().getOrDefault(variable, CausalGraphHeuristic.Unknown);
			Term effVal = o.getEffects().get(variable); 

			Set<Term> from_values = null;

			from_values = new HashSet<>();
			from_values.add(preVal);
		
			for ( Term from_value : from_values ) {
				if ( effVal != null && !from_value.equals(effVal) ) {
					Term edge = Term.tuple(from_value, effVal);
//					LockableSet edge = new LockableSet();
//					edge.add(Term.keyVal(GraphTerm.Edge, ));
					
//					System.out.println(variable + " -> " + from_value);
					LockableSet conditions = new LockableSet();
					for ( Term p : o.getPreconditions() ) {
						if ( !(p.getKey().equals(variable) && p.getValue().equals(from_value)) ) {
							conditions.add(p);
						}
					}
//					System.out.println(Logger.prettyPrint(Term.set(conditions), 0));
//					Term e = Term.tuple(from_value, effVal); 
					E.add(edge);
//					edge.add(Term.keyVal(GraphTerm.Label, )); //o.getPreconditions().asCollection().remove(Term.keyVal(variable, from_value))));
					L.add(Term.keyVal(edge, Term.set(conditions)));
//					System.out.println(Term.set(edge));

					V.add(from_value);
					V.add(effVal);
				}
			}
		}
		return Term.tuple(
				Term.keyVal(GraphTerm.Nodes, Term.set(V)),
				Term.keyVal(GraphTerm.Edges, Term.set(E)),
				Term.keyVal(GraphTerm.Labels, Term.set(L))
				);
	}
}
