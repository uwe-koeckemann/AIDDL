import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{ListTerm, Substitution, Sym}
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.constraint.CspSolver
import org.aiddl.common.scala.planning.state_variable.ForwardSearchPlanIterator
import org.aiddl.core.scala.tools.Logger

@main def solve: Unit =
  val db = new Container
  val modCsp = Parser.parseInto("../aiddl/n-queens.aiddl", db)
  val modPlan = Parser.parseInto("../aiddl/planning.aiddl", db)

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

  val forwardPlanner = new ForwardSearchPlanIterator
  forwardPlanner.init(planningProblemSubstituted)

  val answer = forwardPlanner.search
  answer match {
    case Some(plan) => plan.foreach(println)
    case None => println("No plan found.")
  }
