has_subclass(place, room).
has_subclass(room, kitchen).
has_subclass(room, bedroom).
has_subclass(room, living-room).
has_subclass(place, garden).
has_subclass(garden, loc-0).
has_subclass(living-room, loc-1).
has_subclass(kitchen, loc-2).
has_subclass(bedroom, loc-3).

subclass_of(X, Y) :- has_subclass(X, Y).
subclass_of(X, Y) :- has_subclass(X, Z), subclass_of(Z, Y).
