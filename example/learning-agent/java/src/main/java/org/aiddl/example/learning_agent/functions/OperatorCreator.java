package org.aiddl.example.learning_agent.functions;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;

public class OperatorCreator implements ConfigurableFunction {

	int target;
	String name = OperatorCreator.class.getSimpleName();
	boolean verbose = false;

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term tree ) {
		List<TupleTerm> O_new = extract(tree, Term.list(), null);
				
		O_new.sort(new CompareTuple());
		SetTerm O_set = Term.set(O_new);
		
		return O_set;
	}
	
	private class CompareTuple implements Comparator<TupleTerm> {
		@Override
		public int compare(TupleTerm o1, TupleTerm o2) {
			return o1.toString().compareTo(o2.toString());
		}
	}
	
	private boolean isCollectionOfSets( SetTerm S ) {
		for ( Term s : S ) {
			if ( !(s instanceof SetTerm) ) {
				return false;
			}
		}
		return true;
	}
	
	private List<TupleTerm> extract( Term tree, ListTerm preCurrent, Term actionName ) {
		List<TupleTerm> O = new ArrayList<TupleTerm>();
		if ( !(tree instanceof ListTerm) ) {
			SetTerm eff = (SetTerm) tree;
			
			if ( this.isCollectionOfSets(eff) ) { 
				SetTerm effects = eff;
				for ( Term eff_inside : effects ) {
					SetTerm actual_eff = (SetTerm) eff_inside;
					
					if ( actual_eff.size() > 0 ) {		
						Collection<Term> preCol = preCurrent.getCollectionCopy();
//						preCol.removeAll(actions.getCollection());
						SetTerm pre = Term.set(preCol);
						
//						Term name = Term.tuple(Term.sym("op"), Term.integer(eff.hashCode()));
//						Term name = Term.tuple(Term.sym("op_" + actual_eff.hashCode()), actual_eff);
						Term name = actionName;

						if ( actionName != null ) {

							Term namePair = Term.keyVal(PlanningTerm.Name, name);
							Term prePair = Term.keyVal(PlanningTerm.Preconditions, pre);
							Term effPair = Term.keyVal(PlanningTerm.Effects, actual_eff);
							
							O.add(Term.tuple(namePair, prePair, effPair));
						} else {
							System.err.println("NOT ADDING...");
						}
					}
				}
			} else {
				if ( eff.size() > 0 ) {
					Collection<Term> preCol = preCurrent.getCollectionCopy();
//					preCol.removeAll(actions.getCollection());
					SetTerm pre = Term.set(preCol);
									
//					Term name = Term.tuple(Term.sym("op"), eff);
//					Term name = Term.tuple(Term.sym("op"), Term.integer(eff.hashCode()));
					Term name = actionName;
					
					if ( actionName != null ) {
						Term namePair = Term.keyVal(PlanningTerm.Name, name);
						Term prePair = Term.keyVal(PlanningTerm.Preconditions, pre);
						Term effPair = Term.keyVal(PlanningTerm.Effects, eff);
											
						O.add(Term.tuple(namePair, prePair, effPair));
					} else {
						System.err.println("NOT ADDING...");
					}
				}
			}
		} else {
			ListTerm decisions = (ListTerm) tree;
			for ( Term decision : decisions ) {
				TupleTerm condition = (TupleTerm) decision.get(0);
				Term subTree = decision.get(1);
				Term preAtom  = condition.get(1);
				Term preValue = condition.get(2);
				
				ListTerm preNew;
				
				if ( preAtom.equals(Term.sym("Action"))) {
					actionName = preValue;
					preNew = preCurrent;
				} else {
					preNew = preCurrent.add(
							Term.keyVal(preAtom, 
										preValue));
				}
		
				O.addAll(extract(subTree, preNew, actionName));
			}
		}
		return O;
	}
}
