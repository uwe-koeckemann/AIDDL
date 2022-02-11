package org.aiddl.common.java.optimization.combinatorial.tsp;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class TourCost implements Function {
    @Override
    public Term apply(Term x) {
        Term path = x.get(0).asList();
        Term weights = x.get(1).get(Term.sym("weights"));
        NumericalTerm cost = weights.get(Term.set(path.get(0), path.get(path.size() - 1))).asNum();

        for (int i = 1; i < path.size(); i++) {
            cost = cost.add(weights.get(Term.set(path.get(i - 1), path.get(i))).asNum());
        }
        return cost;
    }
}
