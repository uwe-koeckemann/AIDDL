import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Sym
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.constraint.CspSolver
import org.aiddl.common.scala.planning.state_variable.ForwardSearchPlanIterator
import org.aiddl.core.scala.representation.Substitution
import org.aiddl.core.scala.tools.Logger

@main def solve: Unit =
  val db = new Container
  Function.loadDefaultFunctions(db)

  val modCsp = Parser.parseInto("../aiddl/n-queens.aiddl", db)
  val modPlan = Parser.parseInto("../aiddl/planning.aiddl", db)

  val csp = db.eval(db.getEntry(modCsp, Sym("csp")).get.v)
  val planningProblem = db.eval(db.resolve(db.getEntry(modPlan, Sym("problem")).get.v))

  val cspSolver = new CspSolver
  println(csp)
  val a = cspSolver(csp)
  println(a)

  println(Logger.prettyPrint(planningProblem, 0))

  val substitution = Substitution.from(a.asCol)

  val planningProblemSubstituted = planningProblem \ substitution

  println(Logger.prettyPrint(planningProblemSubstituted, 0))

  val forwardPlanner = new ForwardSearchPlanIterator
  forwardPlanner.init(planningProblemSubstituted)

  val plan = forwardPlanner.search

  println(Logger.prettyPrint(plan, 0))