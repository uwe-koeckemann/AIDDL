package org.aiddl.common.java.tools;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

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
