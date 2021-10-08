package org.aiddl.common.learning.linear_regression;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class ExpansionFunction implements Function {

    @Override
    public Term apply(Term t) {
        return Term.tuple(Term.real(1.0), t.get(0)); //, t.get(1).asNum().mult(t.get(1).asNum()));
    }
    
}
