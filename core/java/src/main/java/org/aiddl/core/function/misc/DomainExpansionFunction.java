package org.aiddl.core.function.misc;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.function.Uri;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.LazyFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.tools.ComboIterator;

/**
 * Evaluate a domain. Replaces domain expression by set containing all 
 * values in the domain.
 * 
 * @author Uwe Koeckemann
 */
public class DomainExpansionFunction implements LazyFunction, ConfigurableFunction {
	

	Function eval;
	
	private final static Term MinKey = Term.sym("min");
	private final static Term MaxKey = Term.sym("max");
	private final static Term IncKey = Term.sym("inc");
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public DomainExpansionFunction( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		if ( !(x instanceof CollectionTerm) ) {
			return x;
		}
		return Term.set( evalDomain(x.asCollection()) );
	}
	
	private Collection<Term> evalDomain( CollectionTerm C ) {
		Set<Term> D = new LinkedHashSet<>();
		
		if ( C.containsKey(MinKey)) {
			NumericalTerm min = (NumericalTerm) this.eval.apply(C.get(MinKey));
			NumericalTerm max = (NumericalTerm) this.eval.apply(C.get(MaxKey));
			NumericalTerm inc = (NumericalTerm) this.eval.apply(C.getOrDefault(IncKey, Term.integer(1)));
			for(  NumericalTerm i = min ; i.lessThanEq(max) ; i = i.add(inc) ) {
				D.add(i);
			}
		} else {
		
			for ( Term e : C ) {
				e = eval.apply(e);
				if ( e instanceof CollectionTerm ) {
					D.addAll(evalDomain((CollectionTerm)e));
				} else if ( e instanceof TupleTerm ) {
					List<List<Term>> combo_in = new ArrayList<>();
					for ( int i = 0 ; i < e.size() ; i++ ) {
						List<Term> sub_domain = new ArrayList<>();
						if ( e.get(i) instanceof CollectionTerm ) { 
							sub_domain.addAll(evalDomain(e.get(i).asCollection()));
						} else {
							sub_domain.add(e);
						}
						combo_in.add(sub_domain);
					}
					
					ComboIterator<Term> cBuilder = new ComboIterator<>(combo_in);
					for ( List<Term> combo : cBuilder ) {
						D.add(Term.tuple(combo));
					}
				} else {
					D.add(e);
				}
			}
		}
		return D;
	}
}
