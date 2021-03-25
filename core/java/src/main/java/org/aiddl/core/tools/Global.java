package org.aiddl.core.tools;

public class Global {
	public static String workDir() {
		return System.getenv("AIDDL_WORK") + "/";
	}
}
