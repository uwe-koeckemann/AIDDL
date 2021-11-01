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

The full aiddl file can be found [here](../../aiddl/automata/discrete-finite-state-machine.aiddl).

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
        
The aiddl file with this entry can be found
[here](../../test/automata/dfa-01.aiddl).

## Implementation Overview

Quick overview of what and how we implemented it. Does not need code but may contain refrences to 

## Try It Yourself

Commented test cases

# References

[1] 
