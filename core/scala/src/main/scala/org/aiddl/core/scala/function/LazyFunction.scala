package org.aiddl.core.scala.function

/**
 * Trait used to mark a function as lazy. Arguments of lazy functions are skipped by the evaluator and left to be evaluated
 * only when needed inside the function.
 */
trait LazyFunction
