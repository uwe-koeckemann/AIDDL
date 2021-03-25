package org.aiddl.common.tools;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class Sleeper implements Function {
	boolean verbose = true;
			
	@Override
	public Term apply( Term ms ) {
		try {
			Thread.sleep(ms.getIntValue());
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}		
		return null;
	}
}
