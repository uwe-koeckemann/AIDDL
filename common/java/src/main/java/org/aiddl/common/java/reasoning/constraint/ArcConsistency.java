package org.aiddl.common.java.reasoning.constraint;

import java.util.Map;

import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.LockableSet;

public class ArcConsistency implements Function, ConfigurableFunction {

	Function eval;
		
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}

	@Override
	public Term apply(Term args) {
		Term CSP = args.get(0);
		CollectionTerm a = args.get(1).asCollection();
		
		CollectionTerm X = CSP.get(0).asCollection();
		CollectionTerm D = CSP.get(1).asCollection();
		CollectionTerm C = CSP.get(2).asCollection();

//		int n_con_checks = 0;
		
		LockableList L = new LockableList();
		for ( Term variable : X ) {
			LockableSet D_new = new LockableSet();
			if ( !a.containsKey(variable) ) {
				for ( Term value : D.get(variable).asCollection() ) {
					boolean c_sat = true;
					for ( Term c : C ) {
						Substitution a_sub = new Substitution(a);
						a_sub.add(variable, value);
						
						Term c_sub = c.get(1).get(1).substitute(a_sub);
						
						if ( !this.eval.apply(c_sub).getBooleanValue() ) {
							c_sat = false;
							break;
						}
					}
//					n_con_checks ++;
					if ( c_sat ) {
						D_new.add(value);
					}
				}
			}
			L.add(Term.keyVal(variable, Term.set(D_new)));
		}
//		System.out.println("Con checks: " + n_con_checks);
		
		return Term.list(L);
	}
}
