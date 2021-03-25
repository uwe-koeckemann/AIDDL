package org.aiddl.example.learning_agent;

public class Run {

	public static void main(String[] args) {
		if ( args[0].equals("l4p") ) {
			LearningForPlanningExample.main(args);
		} else if ( args[0].equals("p4l") ) {
			PlanningForLearningExample.main(args);
		} else {
			System.out.println("Need command line argument l4p or p4l to select which version to run. Received argument: " + args[0]);
		}
	}
}
