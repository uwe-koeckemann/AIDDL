from __future__ import print_function

from concurrent import futures

import grpc
import rospy

from aiddl_core.parser import parser
from aiddl_external_grpc_python.actor import ActorServer
from aiddl_external_grpc_python.converter import Converter

import aiddl_external_grpc_python.generated.actor_pb2 as actor_pb2

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

class ActionlibActorServer(ActorServer):
    def __init__(self,
                 port,
                 topic,
                 action_lib_client,
                 f_is_supported,
                 f_extract_goal,
                 f_extract_fb=None):
        super(ActionlibActorServer, self).__init__(port)
        self.current_id = 0
        self.topic = topic
        self.client = action_lib_client
        self.status_history = {}
        self.f_is_supported = f_is_supported
        self.f_extract_goal = f_extract_goal
        self.f_extract_fb = f_extract_fb
        self.feedback = None
        
    def _feedback_handler(self, fb):
        print("Handling feedback...")
        if self.f_extract_fb is None:
            return None
        self.feedback = self.f_extract_fb(fb)
        
        
    def IsSupported(self, request, context):
        is_supported = self.f_is_supported(request)
        print('Is %s supported? %s' % (str(action), str(is_supported)))
        r = actor_pb2.Supported(is_supported=is_supported)
        print('Response:', r)
        return r
               
    def Dispatch(self, request, context):
        self.current_id += 1
        self.client.wait_for_server()
        goal = self.f_extract_goal(request)
        self.client.send_goal(goal, feedback_cb=self._feedback_handler)
        return self.currentGoalToStatus()

    def Status(self, request, context):
        #a_id = request.id
        #if a_id < self.current_id:
        #    return self.status_history[a_id]
        #else:
        return self.currentGoalToStatus()

    def Cancel(self, request, context):
        return actor_pb2.Status(
            id=self.next_id,
            status=2,
            feedback=aiddl_msg_pb2.AiddlStr(""),
            msg=""
        )

    def currentGoalToStatus(self):
        pb_status = None
        # if self.client.has_result():
        status = self.client.get_state()

        if status == GoalStatus.PENDING:
            pb_status = actor_pb2.PENDING
        elif status == GoalStatus.ACTIVE:
            pb_status = actor_pb2.ACTIVE
        elif status == GoalStatus.PREEMPTED:
            pb_status = actor_pb2.PREEMPTED
            #self.current_id += 1
        elif status == GoalStatus.SUCCEEDED:
            pb_status = actor_pb2.SUCCEEDED
            #self.current_id += 1
        elif status == GoalStatus.ABORTED:
            pb_status = actor_pb2.ERROR
            #self.current_id += 1
        elif status == GoalStatus.REJECTED:
            pb_status = actor_pb2.REJECTED
            #self.current_id += 1
        elif status == GoalStatus.PREEMPTING:
            pb_status = actor_pb2.PREEMPTING
        elif status == GoalStatus.RECALLING:
            pb_status = actor_pb2.RECALLING
        elif status == GoalStatus.RECALLED:
            pb_status = actor_pb2.RECALLED
            #self.current_id += 1
        else:
            pb_status = actor_pb2.ERROR
            print("Unknown status:", status)

        feedback = ""
        if self.feedback is not None:
            feedback = str(self.feedback)
            
        r = actor_pb2.Status(
            id=self.current_id,
            state=pb_status,
            feedback=aiddl_msg_pb2.AiddlStr(aiddl_str=feedback),
            msg=""
        )
        self.status_history[self.current_id] = r
        return r
