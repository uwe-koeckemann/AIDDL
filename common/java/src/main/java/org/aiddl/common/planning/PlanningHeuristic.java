package org.aiddl.common.planning;

import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;

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
