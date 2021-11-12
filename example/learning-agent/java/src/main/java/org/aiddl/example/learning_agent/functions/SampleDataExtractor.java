package org.aiddl.example.learning_agent.functions;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.Logger;

public class SampleDataExtractor implements ConfigurableFunction {

	Random rng;
	
	String name = "ExtractDataFromSamples";
	int verbose = 0;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		rng = new Random();
	}

	Term var = Term.anonymousVar();
	private final static Term COLLECTED = Term.sym("collected"); 
	private final static Term DATA = Term.sym("data");
	private final static Term SAMPLE = Term.sym("sample");
	private final static Term EMPTY = Term.sym("empty");
	private final static Term MATCH = Term.keyVal( Term.tuple(COLLECTED, Term.tuple(DATA, Term.var("L"), Term.var("C"))), Term.sym("true"));
	private final static Term MATCH_SAMPLE = Term.keyVal( Term.tuple(SAMPLE, Term.anonymousVar()), Term.anonymousVar());
	private final static Term DATA_KEY = Term.tuple(Term.var("L"), Term.var("C"));
		
	@Override
	public Term apply( Term args ) {
		CollectionTerm state = (CollectionTerm) args.get(Term.sym("state"));
		CollectionTerm model = (CollectionTerm) args.get(Term.sym("model"));
		CollectionTerm data  = (CollectionTerm) args.get(Term.sym("data"));
		
		Set<Term> s_updated = new LinkedHashSet<>();
		
		List<Term> new_data = new ArrayList<>();
		for ( Term p : state ) {
			Term p_sv = p.getKey();
			Term p_a = p.getValue();
			Term sva = Term.keyVal(p_sv, p_a);
			Substitution s = MATCH.match(sva);
			
			if ( s != null ) {
				Term key = DATA_KEY.substitute(s);
				Term observed = model.get(key);
				new_data.add(Term.list(key.get(0), key.get(1), observed));
			} else if ( MATCH_SAMPLE.match(p) != null ) {
				s_updated.add(Term.keyVal(p_sv, EMPTY));
			} else {
				s_updated.add(p);
			}
		}
		
		if ( verbose >= 1 ) {
			Logger.msg(name, "Extracted " + new_data.size() + " samples.");
			if ( verbose >= 2 ) {
				for ( Term t : new_data ) {
					Logger.msg(name, "  " + t.toString() );
				}
			}
		}
			
		data.addAllTo(new_data);
//		new_data.addAll(data.getCollection());
		
		LockableList rList = new LockableList();
		rList.add(Term.keyVal(DataKey, Term.list(new_data)));
		rList.add(Term.keyVal(StateKey, Term.set(s_updated)));
		return Term.list( rList );
	}
	
	private static final Term DataKey = Term.sym("new-data");
	private static final Term StateKey = Term.sym("new-state");
}
