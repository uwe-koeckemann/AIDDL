# A GRPC Library for AIDDL

Interfaces to connect the AIDDL framework to other components via Protobuf and gRPC. 

- Proxy container functionality
  - Server hosts a container
  - Client is a proxy for a container
  - Supported services:
    - Call function registered on server via its URI 
- Proxy Function
  - Single AIDDL function offered by a server
  - Client is a proxy of an AIDDL function f: Term -> Term
- Actor abstraction
  - Implement actor servers
  - Use actor servers via gRPC
- Sender abstraction
  - Send AIDDL messages to a server
- Receiver abstraction
  - Read queued up AIDDL messages form a receiver server
  - Query determines how messages are retrieved
  - Server collects messages and sends them to client when queried
  - Can be used to collect sensor data occasionally queried by a sensor abstraction

## Roadmap

### 0.1.0

- Actor client
- Actor server (abstract)
- Container proxy
  - Function call service
- Container proxy client
- Function proxy client
- Function proxy server (abstract)
- Receiver client
- Receiver server (abstract)
- Sender client
- Sender server (abstract)
