import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.external.grpc.scala.converter.Converter
import org.aiddl.core.scala.representation._


class ConverterSuite extends AnyFunSuite {
  val c = new Container
  val converter = new Converter(c)
  val parser = new Parser(c)

  test("Test Sym conversion") {
    val term = Sym("x")
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test Var conversion") {
    val term = Var("x")
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test anonymous Var conversion") {
    val term = Var()
    assert(term.toString == converter.pb2aiddl(converter.aiddl2pb(term)).toString)
  }

  test("Test Integer conversion") {
    val term = Num(35)
    assert(term.toString == converter.pb2aiddl(converter.aiddl2pb(term)).toString)
  }

  test("Test Real conversion") {
    val term = Num(35.345)
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test String conversion") {
    val term = Str("This is a string.")
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test Boolean conversion") {
    val termFalse = Bool(false)
    assert(termFalse == converter.pb2aiddl(converter.aiddl2pb(termFalse)))
    val termTrue = Bool(true)
    assert(termTrue == converter.pb2aiddl(converter.aiddl2pb(termTrue)))
  }

  test("Test FunRef conversion") {
    c.addFunction(Sym("f"), x => x)
    val term = FunRef(Sym("f"), x => x)
    assert(term.toString == converter.pb2aiddl(converter.aiddl2pb(term)).toString)
  }

  test("Test KeyVal conversion") {
    val term = KeyVal(Sym("x"), Num(23))
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test InfPos conversion") {
    val term = InfPos()
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test InfNeg conversion") {
    val term = InfNeg()
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test NaN conversion") {
    val term = NaN()
    assert(converter.pb2aiddl(converter.aiddl2pb(term)).asNum.isNan)
  }

  test("Test List conversion") {
    val term = ListTerm(Num(1), Num(2), Num(3))
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test Set conversion") {
    val term = SetTerm(Num(1), Num(2), Num(3))
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test Tuple conversion") {
    val term = Tuple(Num(1), Num(2), Num(3))
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }

  test("Test EntRef conversion") {
    val term = EntRef(Sym("mod"), Tuple(Sym("a"), Num(1)), Sym("alias"))
    assert(term == converter.pb2aiddl(converter.aiddl2pb(term)))
  }
}