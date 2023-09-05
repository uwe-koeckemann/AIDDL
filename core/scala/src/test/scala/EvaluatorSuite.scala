package org.aiddl.core.scala

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function
import org.aiddl.core.scala.function.`type`.GenericTypeChecker
import org.aiddl.core.scala.function.{Configurable, DefaultFunctionUri, Function, Initializable}
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple, Var}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.test.TestFunction


class EvaluatorSuite extends AnyFunSuite {
  val c = new Container()
  val parser = new Parser(c)
  Function.loadDefaultFunctions(c)
  val eval = new Evaluator(c)

  val throwingFunctionList = List(
    DefaultFunctionUri.FILTER,
    DefaultFunctionUri.MAP,
    DefaultFunctionUri.REDUCE,
    DefaultFunctionUri.ADD,
    DefaultFunctionUri.SUB,
    DefaultFunctionUri.MULT,
    DefaultFunctionUri.DIV,
    DefaultFunctionUri.AND,
    DefaultFunctionUri.OR,
    DefaultFunctionUri.EXISTS,
    DefaultFunctionUri.FORALL,
    DefaultFunctionUri.COND,
    DefaultFunctionUri.IF,
    DefaultFunctionUri.ZIP,
    DefaultFunctionUri.MATCH,
    DefaultFunctionUri.DOMAIN,
    DefaultFunctionUri.LET,
    DefaultFunctionUri.LAMBDA,
    DefaultFunctionUri.TYPE
  )

  var badArgs: Map[Sym, Term] = Map.empty.withDefaultValue(Sym("bad-arg"))

  test("Evaluator on simple equation") {
    val eqn = parser.str("(org.aiddl.eval.numerical.add 1 2 3)")
    assert(Num(6) == eval.apply(eqn))
  }

  test("Functions throw exception on bad argument") {
    throwingFunctionList.foreach(
      uri => {
        val f = c.getFunctionOrPanic(uri)
        val badArg = badArgs(uri)
        assertThrows[IllegalArgumentException](f(badArg))
      }
    )
  }

  test("Match function does not find match and throws exception.") {
    val f = c.getFunctionOrPanic(DefaultFunctionUri.MATCH)
    assertThrows[IllegalArgumentException](f(Tuple(Sym("X"), ListTerm.empty)))
  }

  test("Domain generation on tuples") {
    val f = c.getFunctionOrPanic(DefaultFunctionUri.DOMAIN)
    val arg =
      SetTerm(
        Tuple(
          Sym("f"),
          ListTerm(Sym("a"), Sym("b")),
          ListTerm(Num(1), Num(2))))

    assert(f(arg).asCol.size == 4)
  }

  test("Lambda function to string test") {
    val f = c.getFunctionOrPanic(DefaultFunctionUri.LAMBDA)
    val lambdaFun = f(Tuple(Sym("x"), Tuple(Sym("+"), Sym("x"), Num(2))))
    assert(lambdaFun.asFunRef.f.toString == "(org.aiddl.eval.lambda x (+ x 2))")
  }

  test("Function loader with initialization and configuration") {
    val f = c.getFunctionOrPanic(DefaultFunctionUri.LOAD_FUNCTION)
    val function = f(Tuple(
      KeyVal(Sym("name"), Sym("my-factory")),
      KeyVal(Sym("module"), Sym("my-module")),
      KeyVal(Sym("init"), Sym("x")),
      KeyVal(Sym("config"), SetTerm.empty),
      KeyVal(Sym("class"), Sym("org.aiddl.core.scala.test.TestFunction"))
    )).asFunRef.f
    val result = function(Sym("NIL"))
    assert(result == Tuple(SetTerm.empty, Sym("x")))
  }

  test("Function factory with initialization and configuration") {
    val f = c.getFunctionOrPanic(DefaultFunctionUri.LOAD_FUNCTION_FACTORY)
    val factoryUri = f(Tuple(
      KeyVal(Sym("name"), Sym("my-factory")),
      KeyVal(Sym("module"), Sym("my-module")),
      KeyVal(Sym("class"), Sym("org.aiddl.core.scala.test.TestFunction"))
    )).asSym
    val factory = c.getFunctionOrPanic(factoryUri)

    val functionUri = factory(Tuple(
      Sym("new-fun-name"),
      KeyVal(Sym("init"), Sym("x")),
      KeyVal(Sym("config"), SetTerm.empty)
    )).asSym

    val function = c.getFunctionOrPanic(functionUri)
    val result = function(Sym("NIL"))

    assert(result == Tuple(SetTerm.empty, Sym("x")))
  }

  test("Generic type checker throws exception") {
    val tChecker = new GenericTypeChecker(Sym("uri"), Tuple(), Sym("x"), c.eval, c)
    assertThrows[IllegalArgumentException](tChecker(Sym("NIL")))
  }
}

