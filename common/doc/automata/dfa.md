# Deterministic Finite State Machine

A *Deterministic Finite State Machine (DFSM)* [1] is a model of states, events, and
transitions between states based on the current state and a given
event. Starting from an initial state, a DFSM accepts a sequence of events (also
called word) if the state reached when processing all events is a final state.

## Type Definition

States and events are represented as either symbols or first-order logic atoms.

    (#type State (type.union {^symbolic ^Atom@FL}))
    (#type Event (type.union {^symbolic ^Atom@FL}))
    
For convenience, we define a shortcut to sets of states and events.
    
    (#type States (type.set ^$State))
    (#type Events (type.set ^$Event))
    
A transition maps a tuple of a state and event to a state.
    
    (#type Transition  (type.kvp (type.sig [^$State ^$Event]):^$State))
    
A set of transitions is constrained to contain at most one successor to any state-event combination.

    (#type Transitions (type.set ^$Transition
                                  constraint:(lambda ?X
                                               (forall ?k:?v1 ?X
                                                 (not (exists ?k:?v2 ?X
                                                   (!= ?v1 ?v2)))))))

Finally, we define a *Deterministic Finite State Machine (DFSM)* as a tuple
containing a set of states, a set of events, a set of transitions, and initial
state, and a set of final states. The constraint asserts that all events and
states are part of their respective sets.

    (#type DFSM (type.sig [^$States ^$Events ^$Transitions ^$State ^$States]
      constraint:(lambda ?X 
        (match (?Q ?E ?D ?q0 ?F) ?X
          (and
            (forall (?s ?e) : ?s_next, ?D
              (and
                (in ?s ?Q)
                (in ?e ?E)
                (in ?s_next ?Q)))
            (forall ?s ?F (in ?s ?Q))
            (in ?q0 ?Q)
          )))))

The full AIDDL type definition can be found [here](../../aiddl/automata/discrete-finite-state-machine.aiddl).

## Example Input

The following entry contains an instance of a DFSM with two states `s1` and
`s2`, two events `a` and `b`, three transitions, initial state `s1` and a single
final state `s2`.

    (DFSM@A dfa
          (
            {s1 s2}
            {a b}
            {
              (s1 a) : s1
              (s1 b) : s2
              (s2 b) : s2
            }
            s1
            {s2}
          )
    )
        
The aiddl file containing this example can be found
[here](../../test/automata/dfa-01.aiddl).

## Implementation Overview

The Scala implementation of a DFSM can be found
[here](../../scala/src/main/scala/org/aiddl/common/scala/automata/DeterministicFiniteStateMachine.scala).

It loads a DFSM via the initializable trait.  Afterwards, the apply method
allows to traverse the state space by sending single events or sequences of
events, checking the current state and wether it is a final state, and resetting
the machine.

    override def apply(x: Term): Term = {
        x match {
            case Tuple(Sym("step"), e) => s = transitions.getOrPanic(Tuple(s, e)); s
            case Tuple(Sym("multi-step"), ListTerm(list)) => s = list.foldLeft(s)( (s, e) => transitions.getOrPanic(Tuple(s, e)) ); s
            case Sym("is-final-state") => Bool(finalStates.contains(s))
            case Sym("current-state") => s
            case Sym("reset") => s = initialState; s
            case _ => Sym("NIL")
        }
    }

## Try It Yourself

First, we create a container, parse the test file and load the entry.

    val c = new Container()
    val m = Parser.parseInto("../test/automata/dfa-01.aiddl", c)
    val p = c.resolve(c.getEntry(m, Sym("dfa")).get.v)
    
Next, we create an instance of our DFSM implementation and initialize it with
the entry shown above.

    val f_DFS = new DeterministicFiniteStateMachine
    f_DFS.init(p)
    
Now we can `step` with single events.

    val s1 = f_DFS(Tuple(Sym("step"), Sym("a")))
    assert(s1 == Sym("s1"))
    
We can use `multi-step` to move through a sequence of events.

    val s2 = f_DFS(Tuple(Sym("multi-step"), ListTerm(Sym("a"), Sym("a"), Sym("a"))))
    assert(s2 == Sym("s1"))
    
Finally, we perform another step that moves to `s2`, check again on the current
state and confirm that this is the final state.

    val s3 = f_DFS(Tuple(Sym("step"), Sym("b")))
    assert(s3 == Sym("s2"))
    assert(f_DFS(Sym("current-state")) == Sym("s2"))
    assert(f_DFS(Sym("is-final-state")) == Bool(true))
    
The test case containing this code can be found [here](../../scala/src/test/scala/automata/AutomataSuite.scala).

# References

[1] Sipser, Michael (2013). "Introduction to the Theory of Computation". Cengage Learning. 
