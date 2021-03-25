package org.aiddl.common.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.ComboIterator;
import org.aiddl.core.tools.Logger;

public class Expand implements ConfigurableFunction {
	
	private String name = Expand.class.getSimpleName();
	private boolean verbose = false;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term args ) {
		CollectionTerm collection  = (CollectionTerm) args.get(Term.sym("container"));
		Term expansionTerm = args.get(Term.sym("term"));
		ListTerm domainMap = (ListTerm) args.get(Term.sym("choices"));
		List<Term> variables = new ArrayList<Term>();
		
		List<List<Term>> choices = new ArrayList<List<Term>>();
		for ( Term var_choice : domainMap ) {
			variables.add(var_choice.getKey());
			CollectionTerm domain = (CollectionTerm) var_choice.getValue();
			List<Term> domainList = new ArrayList<>();
			domain.addAllTo(domainList);
			choices.add( domainList );
		}
		
		if ( verbose ) {
			Logger.msg(name, "Container: " + collection);
			Logger.msg(name, "Term:      " + expansionTerm);
			Logger.msg(name, "Choices:   " + choices);
			Logger.incDepth();
		}
		
		ComboIterator<Term> combos = new ComboIterator<>(choices);
		for ( List<Term> combo : combos ) {
			Substitution s = new Substitution();
			for ( int i = 0 ; i < combo.size() ; i++ ) {
				s.add(variables.get(i), combo.get(i) );
			}
			if ( verbose ) Logger.msg(name, "Combo: " + s);
			collection = collection.add(expansionTerm.substitute(s));
		}
		
		if ( verbose ) {
			Logger.decDepth();
		}
			
		return collection;
	}
}
