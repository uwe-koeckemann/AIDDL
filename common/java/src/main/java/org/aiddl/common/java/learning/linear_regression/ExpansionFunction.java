package org.aiddl.common.java.learning.linear_regression;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class ExpansionFunction implements Function {

    @Override
    public Term apply(Term t) {
        return Term.tuple(Term.real(1.0), t.get(0)); //, t.get(1).asNum().mult(t.get(1).asNum()));
    }
    
}
