import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{ListTerm, Num, Substitution, Sym}
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.constraint.CspSolver
import org.aiddl.common.scala.planning.state_variable.ForwardSearchPlanIterator
import org.aiddl.common.scala.planning.state_variable.heuristic.FastForwardHeuristic
import org.aiddl.core.scala.util.logger.Logger

@main def solve: Unit =
  val db = new Container
  val parser = new Parser(db)
  val modCsp = parser.parseFile("../aiddl/n-queens.aiddl")
  val modPlan = parser.parseFile("../aiddl/planning.aiddl")

  val csp = db.getProcessedValueOrPanic(modCsp, Sym("csp"))
  val planningProblem = db.getProcessedValueOrPanic(modPlan, Sym("problem"))

  val cspSolver = new CspSolver
  cspSolver.init(csp)
  val a = cspSolver.search
  println(a)
  a match {
    case Some(a) => a.foreach(println)
    case None => println("No assignment found.")
  }

  println(cspSolver.search)
  println(cspSolver.search)

  println(Logger.prettyPrint(planningProblem, 0))

  val substitution = Substitution.from(ListTerm(a.get))

  val planningProblemSubstituted = planningProblem \ substitution

  println(Logger.prettyPrint(planningProblemSubstituted, 0))

  val h_ff = new FastForwardHeuristic
  val forwardPlanner = new ForwardSearchPlanIterator(List((h_ff, Num(1))))
  forwardPlanner.init(planningProblemSubstituted)

  val answer = forwardPlanner.search
  answer match {
    case Some(plan) => plan.foreach(println)
    case None => println("No plan found.")
  }
