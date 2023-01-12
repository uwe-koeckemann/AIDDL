package org.aiddl.core.scala.representation

import scala.annotation.targetName
import scala.collection.immutable.ArraySeq

private[representation] trait SymImpl { self: Sym =>
  private val id = Sym.id(self.name)
  override def toString: String = name
  override def asSym: Sym = this

  def split: ListTerm = ListTerm( ArraySeq.unsafeWrapArray(name.split("[.]")).map(Sym(_)))

  @targetName("concat")
  def +(s: Term):Sym = {
    s match {
      case Sym(tname) => Sym(name + "." + tname)
      case _ => ???
    }
  }

  override def equals(other: Any): Boolean = other match {
    case s : SymImpl => {
      s.id == this.id
    }
    case _ => false
  }
}

