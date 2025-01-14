#  AIDDL Common (Scala)

Collection of common types of AI models, algorithms, types, and abstractions.

# Versions

## 0.1.0 First Release

- First release including approaches for planning, learning, constraint
  processing and logical reasoning.

## 0.3.0

- Controllers, actors, sensors

## 0.4.0

- Graph2Dot extended with support for xlabel
- Tracing graph for GenericTreeSearch
- Sensor trait updated to support modes, remove the need for initial values and make the interface more clear
- ID3 algorithm now uses all attributes 

## 0.5.0

- Added additional propagators for constraint satisfaction problems

## 0.5.1

- Upped scala version to avoid vulnerability in transitive scala 2.x dependency

## 0.6.0

- Cleaning up generic graph search trait
  - Overridable edge cost function
  - Explicit heuristic trait
  - Adding heuristics directly
  - Works without heuristic

