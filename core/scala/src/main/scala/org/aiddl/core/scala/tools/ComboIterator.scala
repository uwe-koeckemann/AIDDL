package org.aiddl.core.scala.tools

import org.aiddl.core.scala.representation.Term
import scala.collection.mutable.ArraySeq

/**
 * Iterate over combinations of choices without storing them internally.
 * @param choices list of list of possible values
 * @tparam T type of the values that are combined
 */
class ComboIterator[T]( choices : Seq[Seq[T]] ) extends Iterator[Seq[T]] {

    private var nextReady = choices.forall( x => x.length > 0 )
    private val selection = ArraySeq.fill(choices.length)(0)
    
    override def hasNext: Boolean = nextReady

    override def next(): Seq[T] = {
        if ( nextReady ) {
            val r = selection.zipWithIndex.map( { case (c, i) => choices(i)(c) } ).toSeq
            nextReady = nextIdx(selection.length-1)
            r
        } else {
            throw new NoSuchElementException
        }
    } 

    private def nextIdx(n: Int): Boolean = {
        if ( n < 0 ) false 
        else {
            val idx =  if ( selection(n) < choices(n).length-1) selection(n) + 1 
                        else if (nextIdx(n-1)) 0
                        else -1
            if ( idx >= 0 ) { selection.update(n, idx); true } else { false }
        }   
    }
}
