package org.aiddl.core.function.misc;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class GetMatchingEntriesFunction implements Function {

	Container C;
	
	public GetMatchingEntriesFunction( Container C ) {
		this.C = C;
	}
	
	@Override
	public Term apply(Term args) {
		Term modulePattern = args.get(0);
		Term typePattern = args.get(1);
		Term namePattern = args.get(2);
		
		Function f = null;
		if ( args.size() > 3 ) {
			f = args.get(3).asFunRef().getFunction();
		}
		
		LockableList S = new LockableList();
		for ( Entry e : C.getMatchingEntries(modulePattern, typePattern, namePattern) ) {
			if ( f == null || f.apply(e.getValue()).getBooleanValue() ) {
				S.add(e.asTuple());
			}
		}
		
		return Term.list(S);
	}

}
