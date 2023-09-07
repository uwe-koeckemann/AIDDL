import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{EntRef, FunRef, KeyVal, Num, SetTerm, Substitution, Sym, Term, Tuple, Var}
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.parser.Parser


class ContainerSuite extends AnyFunSuite {
  test("Adding a module") {
    val C = new Container()
    val m = Sym("my-module")
    C.addModule(m)

    assert(C.getModuleNames == List(m))
  }

  test("Entry works as expected") {
    val entry = Entry(Sym("type"), Sym("name"), Var("x"))
    val substitution = new Substitution()
    substitution.add(Var("x"), Sym("value"))
    assert((entry \ substitution).value == Sym("value"))
    assert(entry.toString == "(type name ?x)")

  }

  test("Container should add Entry") {
    val C = new Container()
    val m = Sym("my-module")
    C.addModule(m)
    val e = Entry(Sym("type"), Sym("name"), Sym("value"))
    assert(C.getModuleEntries(m).isEmpty)
    C.setEntry(m, e)
    assert(C.getModuleEntries(m) == List(e))

    assert(C.getEntry(m, Sym("name")) == Some(e))

    C.deleteEntry(m, e)
    assert(C.getEntry(m, Sym("name")) == None)
  }

  test("Getting matching entries from a container") {
    val C = new Container()
    val m_a = Sym("mod.a")
    val m_b = Sym("mod.b")
    val m_c = Sym("mod.c")
    C.addModule(m_a)
    C.addModule(m_b)
    C.addModule(m_c)

    val e1 = Entry(Tuple(Sym("t"), Var("X")), Tuple(Sym("p"), Var("Y")), Sym("value"))
    val e2 = Entry(Tuple(Sym("t"), Sym("z")), Tuple(Sym("p"), Var("Y")), Sym("value"))
    val e3 = Entry(Tuple(Sym("t"), Var("y")), Tuple(Sym("p"), Sym("c")), Sym("value"))

    C.setEntry(m_a, e1)
    C.setEntry(m_b, e2)
    C.setEntry(m_c, e3)

    val matches = C.getMatchingEntries(Var("M"), Tuple(Var("T"), Sym("z")), Tuple(Sym("p"), Var("c")))

    assert(!matches.contains(e1))
    assert(matches.contains(e2))
  }

  test("Getting functions from a container") {
    val c = new Container()
    val uri = Sym("does-not-exist-yet")
    assert(c.getFunction(uri) == None)
    assert(c.getFunctionRef(uri) == None)

    object f extends Function {
      def apply(x: Term): Term = x
    }
    val fRef = FunRef(uri, f)

    assert(c.getFunctionOrDefault(uri, f) == f)
    assert(c.getFunctionRefOrDefault(uri, fRef) == fRef)

    c.addFunction(Sym("does-not-exist-yet"), f)
    assert(c.getFunction(Sym("does-not-exist-yet")) == Some(f))
    assert(c.getFunctionOrPanic(Sym("does-not-exist-yet")) == f)
    assert(c.getFunctionRef(uri) == Some(fRef))
    assert(c.getFunctionRefOrPanic(uri) == fRef)
  }

  test("Saving a module to a file") {
    val c1 = new Container()
    val uri = Sym("my-new-module")
    c1.addModule(uri)

    val typeRef = c1.getFunctionRefOrPanic(Sym("org.aiddl.type.term.numerical.real"))

    val e1 = Entry(typeRef, Sym("name"), Num(0.66))
    c1.setEntry(uri, e1)
    c1.saveModule(uri, "test-case.aiddl")

    assert(c1.typeCheckAllModules())

    val c2 = new Container()
    val parser = new Parser(c2)
    val uriC2 = parser.parseFile("test-case.aiddl")
    val e2 = c2.getEntry(uriC2, Sym("name"))
    assert(Some(e1) == e2)
    assert(c2.typeCheckAllModules())
  }

  test("Cannot resolve reference to non-existing term.") {
    val C = new Container()
    val m = Sym("my-module")
    C.addModule(m)
    val entryReference = EntRef(m, Sym("NOT_THERE"), Sym("self"))
    assertThrows[IllegalArgumentException] {
      C.resolveReference(entryReference)
    }
  }

  test("Cannot get function that does not exist.") {
    val C = new Container()
    assertThrows[IllegalArgumentException] {
      C.getFunctionOrPanic(Sym("NOT_THERE"))
    }
  }

  test("Cannot find alias that does not exist.") {
    val C = new Container()
    assertThrows[IllegalArgumentException] {
      C.findModuleAlias(Sym("MOD"), Sym("NOT_THERE"))
    }
  }

  test("Type checking non-existing module throws exception") {
    val container = new Container()
    assert(container.toString == "")
    assertThrows[IllegalArgumentException](container.typeCheckModule(Sym("non-existing-module")))
  }
}
