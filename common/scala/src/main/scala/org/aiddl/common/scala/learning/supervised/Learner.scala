package org.aiddl.common.scala.learning.supervised

import org.aiddl.core.scala.function.Function

import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.container.Container

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.learning.Term._

trait Learner extends Function {
    def fit( x: ListTerm, y: ListTerm ): Term
    def predict( x: ListTerm ): ListTerm
    def predictOne( x: Term ): Term = predict(ListTerm(x)).head

    var parameters: CollectionTerm = SetTerm.empty

    def register( m: Sym, c: Container ) = {
        c.addFunction( m + Sym("fit"), t => this.fit(t(0), t(1)) )
        c.addFunction( m + Sym("predict"), x => this.predict(x) )
        c.addFunction( m + Sym("set-params"), x => {this.parameters = x.asCol; NIL})
    }

    def apply( args: Term ): Term = args match {
        case Tuple(Fit.sym, x, y) => this.fit(x, y)
        case Tuple(Predict.sym, x) => this.predict(x)
        case Tuple(SetParams.sym, params: CollectionTerm) => { this.parameters = params; NIL }
        case _ => ???
    }
}