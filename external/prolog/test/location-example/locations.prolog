:- discontiguous adjacent_table/2.

adjacent_table('loc4', 'loc5').
adjacent_table('loc1', 'loc5').

adjacent(L1, L2) :- adjacent_table(L1, L2).
adjacent(L1, L2) :- adjacent_table(L2, L1).


