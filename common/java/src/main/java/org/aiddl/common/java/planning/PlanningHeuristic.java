package org.aiddl.common.java.planning;

import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.SetTerm;

/**
 * Interface for planning heuristics to compute the estimated distance
 * from a state s to a goal g.
 *
 * @author Uwe Koeckemann
 */
public interface PlanningHeuristic {
	/**
	 * Compute the heuristic value from s to g
	 * @param s state
	 * @param g goal
	 * @return estimated length of plan that leads from s to g
	 */
	public NumericalTerm compute ( SetTerm s, SetTerm g );
}
