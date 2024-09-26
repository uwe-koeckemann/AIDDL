# AIDDL gRPC Library

Various AIDDL abstractions made available via Protobuf and gRPC.

- Actor server: implements an actor
- Actor client: connect to a running actor server and use it as a regular actor

# Changelog

# 0.3.1

- Fixed `ActorClient.getStatus` method not calling stub
- Change NotImplemented to IllegalArgumentException when Protobuf term cannot be converted to AIDDL