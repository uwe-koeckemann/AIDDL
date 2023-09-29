# AIDDL External ROS1 Library

This library is a bridge between the AIDDL framework and ROS1 realized via
gRPC. It provides classes that allow to write ROS nodes that focus on the
translation betweem AIDDL and ROS messages.

- TopicSenderServer: Send message from AIDDL to a ROS topic via 
- ActionlibActorServer: Send goals from an AIDDL gRPC actor client to this server that forwards to ROS actionlib goals

## Roadmap

### 0.1.0 

- Support publishing from AIDDL to ROS via TopicSenderServer
- Support for topic subscriptions allowing AIDDL Receiver to listen to ROS topics
- Support for connecting an AIDDL gRPC actor to ROS actionlib

### 0.2.0

- Support calling ROS services through function server
- Functions to capture redundant setup for ROS nodes

# Known Issues

- Converters go from ROS -> AIDDL -> AIDDL Protobuf and could skip intermediate
  step where possible
