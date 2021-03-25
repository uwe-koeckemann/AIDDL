package org.aiddl.common.search.adversarial;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

/**
 * Given a tuple of key-value pairs player:score, compute the relative score for 
 * one player as the distance to the maximum ignoring that player. 
 * @author Uwe Köckemann
 *
 */
public class RelativeScore implements Function {
	
	public Term apply( Term args ) {
		Term player = args.get(0);
		NumericalTerm playerScore = args.get(1).get(player).asNum(); 
		NumericalTerm maxOtherPlayerScore = Term.infNeg();
		
		for ( Term e : args.get(1).asList() ) {
			if ( !e.getKey().equals(player) ) {
				if ( e.getValue().asNum().greaterThan(maxOtherPlayerScore) ) {
					maxOtherPlayerScore = e.getValue().asNum();
					
				}
			}
		}
		
		return playerScore.sub(maxOtherPlayerScore);
	}

}
