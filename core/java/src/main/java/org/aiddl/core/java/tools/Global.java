package org.aiddl.core.java.tools;

public class Global {
	public static String workDir() {
		return System.getenv("AIDDL_WORK") + "/";
	}
}
