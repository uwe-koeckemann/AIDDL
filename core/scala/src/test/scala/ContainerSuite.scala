import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.{KeyVal, Num, SetTerm, Sym, Tuple, Var}
import org.aiddl.core.scala.container.Entry


class ContainerSuite extends AnyFunSuite {
  test("Adding a module") {
    val C = new Container()    
    val m = Sym("my-module")
    C.addModule(m)

    assert( C.getModuleNames == List(m) )
  }

  test("Container should add Entry") {
    val C = new Container()
    val m = Sym("my-module")
    C.addModule(m)
    val e = Entry(Sym("type"), Sym("name"), Sym("value"))

    C.setEntry(m, e)

    assert( C.getEntry(m, Sym("name")) == Some(e) )

    C.deleteEntry(m, e)
    assert( C.getEntry(m, Sym("name")) == None )
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

    assert( !matches.contains(e1) )
    assert( matches.contains(e2) )
  }
}
