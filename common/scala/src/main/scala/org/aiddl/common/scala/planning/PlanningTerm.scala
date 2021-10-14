package org.aiddl.common.scala.planning

import org.aiddl.core.scala.representation.Sym

object PlanningTerm {
    val Operators = Sym("operators")
    val InitialState = Sym("initial-state")
    val State = Sym("state")
    val Goal = Sym("goal")
    
    val Plan = Sym("plan")
    val Id = Sym("id")
    val Name = Sym("name")
    val Preconditions = Sym("preconditions")
    val Effects = Sym("effects")
    
    val Domains = Sym("domains")
    val Signature  = Sym("signature")
    val Signatures = Sym("signatures")
    
    val Heuristic = Sym("heuristic")
    
    val CausalGraph = Sym("causal-graph")
    val DomainTransitionGraphs = Sym("domain-transition-graphs")
    val PlanningGraph = Sym("planning-graph")
    val RelaxedPlanningGraph = Sym("relaxed-planning-graph")
        
    val Task = Sym("task")	
    val SubTasks = Sym("sub-tasks")	
    val OpenTasks = Sym("open-tasks")
    val Methods = Sym("methods")
}