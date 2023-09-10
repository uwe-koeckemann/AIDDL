package org.aiddl.core.scala

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.function
import org.aiddl.core.scala.function.`type`.{GenericTypeChecker, TypeFunction}
import org.aiddl.core.scala.function.{Configurable, DefaultFunctionUri, Evaluator, Function, Initializable}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, EntRef, KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple, Var}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.test.TestFunction

import java.util.logging.Level


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

  test("Matrix type throws exception on row/column mismatch") {
    val matrix = parser.str("((1 2) (3 4))")
    val matrixType = parser.str("(org.aiddl.type.matrix m:2 n:2 col-types:(^numerical) row-types:(^numerical))")
    val tChecker = new TypeFunction(matrixType, c.eval)
    assertThrows[IllegalArgumentException](tChecker(matrix))
  }

  test("Type function throws exception because there is not matching type pattern") {
    val badType = parser.str("(non-existing-type m:2 n:2 col-types:(^numerical) row-types:(^numerical))")
    val tChecker = new TypeFunction(badType, c.eval)
    assertThrows[IllegalArgumentException](tChecker(Sym("x")))
  }

  test("Type function throws exception because there constraint does not evaluate to boolean") {
    val badType = parser.str("(org.aiddl.type.set-of ^org.aiddl.type.term.numerical constraint:{{1}:bad-value})")
    val tChecker = new TypeFunction(badType, c.eval)
    assertThrows[IllegalArgumentException](tChecker(SetTerm(Num(1))))
  }

  test("Type check requires evaluation of type definition") {
    val typeTerm = parser.str("(org.aiddl.eval.quote (org.aiddl.type.set-of ^org.aiddl.type.term.numerical))")
    val tChecker = new TypeFunction(typeTerm, c.eval)
    assert(tChecker(SetTerm(Num(1))) == Bool(true))
  }

  test("Type check matrix") {
    val typeTerm = parser.str("(org.aiddl.type.matrix m:2 n:2 col-types:(^org.aiddl.type.term.numerical ^org.aiddl.type.term.numerical) row-types:(^org.aiddl.type.term.numerical ^org.aiddl.type.term.numerical) cell-type:^org.aiddl.type.term.numerical)")
    val tChecker = new TypeFunction(typeTerm, c.eval)
    assert(tChecker(Tuple(Tuple(Num(1), Num(2)), Tuple(Num(3), Num(4)))) == Bool(true))
    assert(!tChecker(SetTerm.empty).asBool.v)
  }

  test("Type function throws exception because type dictionary has bad format") {
    val badType = parser.str("(org.aiddl.type.dictionary { not-a-key-val })")
    val tChecker = new TypeFunction(badType, c.eval)
    assertThrows[IllegalArgumentException](tChecker(SetTerm(Num(1))))
  }

  test("Testing evaluation with entry reference operators") {
    val thisModule = Sym("this-module")
    val otherModule = Sym("other-module")
    val alias = Sym("alias")
    c.addModuleAlias(thisModule, alias, otherModule)
    c.setEntry(otherModule, Entry(Sym("-"), Sym("pointer"), Sym("org.aiddl.eval.numerical.add")))
    c.setEntry(otherModule, Entry(Sym("-"), Tuple(Sym("pointer")), Sym("org.aiddl.eval.numerical.add")))
    c.setEntry(otherModule, Entry(Sym("-"), Tuple(Sym("pointer-2")), Sym("not.a.function")))
    c.setEntry(otherModule, Entry(Sym("-"), Tuple(Sym("pointer-3")), SetTerm.empty))

    val entRef1 = EntRef(thisModule, Sym("pointer"), alias)
    val entRef2 = EntRef(thisModule, Tuple(Sym("pointer")), alias)
    val entRef3 = EntRef(thisModule, Tuple(Sym("pointer-2")), alias)
    val entRef4 = EntRef(thisModule, Tuple(Sym("pointer-3")), alias)

    assert(c.eval(Tuple(entRef1, Num(2), Num(3))) == Num(5))
    assert(c.eval(Tuple(entRef2, Num(2), Num(3))) == Num(5))
    assert(c.eval(Tuple(entRef3, Num(2), Num(3))) == Tuple(Sym("not.a.function"), Num(2), Num(3)))
    assert(c.eval(Tuple(entRef4, Num(2), Num(3))) == Tuple(SetTerm.empty, Num(2), Num(3)))

  }
}

