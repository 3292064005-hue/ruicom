#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Dict, Optional
import rospy
from std_msgs.msg import String
from .behavior_actuator_core import BehaviorActuatorCore
from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION, require_positive_float
from .lifecycle_protocol import decode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .msg import HealthState
from .time_core import NodeClock

class BehaviorActuatorNode:
    def __init__(self):
        rospy.init_node('behavior_actuator_node', anonymous=False)
        self.config=self._read_config(); self.clock=NodeClock(self.config['time_source_mode']); self.runtime=ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed']); self.core=BehaviorActuatorCore(); self._last_health_signature=(); self._last_health_emit_sec=0.0; self._last_state_signature=(); self._last_vendor_feedback_stamp=0.0
        self.state_pub=rospy.Publisher(self.config['actuator_state_topic'], String, queue_size=20, latch=True)
        self.output_pub=rospy.Publisher(self.config['actuator_output_topic'], String, queue_size=20)
        self.health_pub=rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub=rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.command_sub=rospy.Subscriber(self.config['actuator_command_topic'], String, self._command_cb, queue_size=20)
        self.feedback_sub=rospy.Subscriber(self.config['vendor_feedback_topic'], String, self._vendor_feedback_cb, queue_size=20)
        self.cancel_sub=rospy.Subscriber(self.config['cancel_topic'], String, self._cancel_cb, queue_size=20)
        self.control_sub=rospy.Subscriber(self.config['control_command_topic'], String, self._control_cb, queue_size=20) if self.config['lifecycle_managed'] else None
        self._publish_health('ok','node_ready',self._health_details())
    def _read_config(self)->dict:
        c={'actuator_command_topic': str(rospy.get_param('~actuator_command_topic','recon/platform/behavior_execution/actuator_command')).strip() or 'recon/platform/behavior_execution/actuator_command','actuator_state_topic': str(rospy.get_param('~actuator_state_topic','recon/platform/behavior_execution/actuator_state')).strip() or 'recon/platform/behavior_execution/actuator_state','actuator_output_topic': str(rospy.get_param('~actuator_output_topic','recon/platform/vendor/actuator_command')).strip() or 'recon/platform/vendor/actuator_command','vendor_feedback_topic': str(rospy.get_param('~vendor_feedback_topic','recon/platform/vendor/actuator_feedback')).strip() or 'recon/platform/vendor/actuator_feedback','cancel_topic': str(rospy.get_param('~cancel_topic','recon/platform/behavior_execution/cancel')).strip() or 'recon/platform/behavior_execution/cancel','health_topic': str(rospy.get_param('~health_topic','recon/health')).strip() or 'recon/health','health_typed_topic': str(rospy.get_param('~health_typed_topic','recon/health_typed')).strip() or 'recon/health_typed','health_frame_id': str(rospy.get_param('~health_frame_id','map')).strip() or 'map','publish_rate_hz': float(rospy.get_param('~publish_rate_hz',15.0) or 15.0),'health_heartbeat_hz': float(rospy.get_param('~health_heartbeat_hz',1.0) or 1.0),'time_source_mode': str(rospy.get_param('~time_source_mode','ros')).strip().lower() or 'ros','lifecycle_managed': bool(rospy.get_param('~lifecycle_managed',False)),'control_command_topic': str(rospy.get_param('~control_command_topic','recon/system_manager/command')).strip() or 'recon/system_manager/command','runtime_grade': str(rospy.get_param('~runtime_grade','integration')).strip().lower() or 'integration'}
        c['publish_rate_hz']=require_positive_float('publish_rate_hz',c['publish_rate_hz']); c['health_heartbeat_hz']=require_positive_float('health_heartbeat_hz',c['health_heartbeat_hz'])
        if c['time_source_mode'] not in ('ros','wall'): raise ConfigurationError('behavior_actuator_node time_source_mode must be ros or wall')
        return c
    @staticmethod
    def _connections(endpoint)->int:
        getter=getattr(endpoint,'get_num_connections',None)
        if getter is None: return 0
        try: return int(getter())
        except Exception: return 0
    def _publish_health(self,status,message,details=None):
        payload={'stamp': self.clock.now_business_sec(),'node':'behavior_actuator_node','status':str(status).strip(),'message':str(message).strip(),'schema_version':SCHEMA_VERSION,'details':dict(details or {})}
        self.health_pub.publish(String(data=JsonCodec.dumps(payload))); typed=HealthState(); typed.header.stamp=self.clock.now_ros_time(); typed.header.frame_id=self.config['health_frame_id']; typed.node=payload['node']; typed.status=payload['status']; typed.message=payload['message']; typed.schema_version=payload['schema_version']; typed.details_json=JsonCodec.dumps(payload['details']); self.health_typed_pub.publish(typed); self._last_health_emit_sec=float(payload['stamp']); self._last_health_signature=(payload['status'],payload['message'],JsonCodec.dumps(payload['details']))
    def _publish_state(self,payload:Dict[str,object]):
        sig=(payload.get('command_id',''),payload.get('state',''),JsonCodec.dumps(payload.get('details',{})))
        if sig==self._last_state_signature: return
        self.state_pub.publish(String(data=JsonCodec.dumps(payload))); self._last_state_signature=sig
    def _health_details(self)->Dict[str,object]:
        return {'runtime_grade': self.config['runtime_grade'],'actuator_command_topic_declared': bool(self.config['actuator_command_topic']),'actuator_state_topic_declared': bool(self.config['actuator_state_topic']),'actuator_output_topic_declared': bool(self.config['actuator_output_topic']),'vendor_feedback_topic_declared': bool(self.config['vendor_feedback_topic']),'actuator_output_bound': self._connections(self.output_pub)>0,'downstream_feedback_bound': self._connections(self.feedback_sub)>0,**self.runtime.snapshot(),'has_active_command': bool(self.core.active is not None)}
    def _coerce_payload(self,raw:str)->Dict[str,object]:
        payload=JsonCodec.loads(raw)
        if not isinstance(payload,dict): raise ConfigurationError('behavior actuator payload must be an object')
        return payload
    def _command_cb(self,msg:String)->None:
        try: payload=self._coerce_payload(msg.data)
        except Exception as exc: self._publish_health('warn','malformed_behavior_actuator_command',{**self._health_details(),'error':str(exc)}); return
        command_id=str(payload.get('command_id','')).strip()
        if self.runtime.lifecycle_managed and not self.runtime.processing_allowed:
            self._publish_state({'command_id':command_id,'action_type':str(payload.get('action_type','')).strip(),'task_type':str(payload.get('task_type','')).strip(),'state':'FAILED','stamp':self.clock.now_business_sec(),'details':{'reason':'runtime_not_active'},'source':'behavior_actuator'}); return
        try: state=self.core.accept(payload, now_sec=self.clock.now_business_sec())
        except Exception as exc:
            self._publish_state({'command_id':command_id,'action_type':str(payload.get('action_type','')).strip(),'task_type':str(payload.get('task_type','')).strip(),'state':'FAILED','stamp':self.clock.now_business_sec(),'details':{'reason':str(exc)},'source':'behavior_actuator'}); self._publish_health('warn','behavior_actuator_command_rejected',{**self._health_details(),'error':str(exc),'command_id':command_id}); return
        self.output_pub.publish(String(data=JsonCodec.dumps(payload))); self._publish_state(state.to_dict()); self._publish_health('ok','behavior_actuator_command_accepted',self._health_details())
    def _vendor_feedback_cb(self,msg:String)->None:
        self._last_vendor_feedback_stamp=self.clock.now_business_sec()
        try: payload=self._coerce_payload(msg.data)
        except Exception: return
        state=self.core.observe_vendor_feedback(payload, now_sec=self.clock.now_business_sec())
        if state is not None: self._publish_state(state.to_dict())
    def _cancel_cb(self,msg:String)->None:
        try: payload=self._coerce_payload(msg.data)
        except Exception: return
        state=self.core.cancel(now_sec=self.clock.now_business_sec(), reason=str(payload.get('reason','cancelled')).strip() or 'cancelled')
        if state is not None: self._publish_state(state.to_dict())
    def _control_cb(self,msg:String)->None:
        try: control=decode_lifecycle_control(msg.data)
        except Exception: return
        if control.target and control.target!='behavior_actuator_node': return
        result=self.runtime.apply(control.command)
        if result.changed and result.state in ('IDLE','PAUSED','SHUTDOWN'):
            state=self.core.cancel(now_sec=self.clock.now_business_sec(), reason='lifecycle_'+control.command)
            if state is not None: self._publish_state(state.to_dict())
        self._publish_health('ok' if result.accepted else 'warn', result.message, self._health_details())
    def spin(self)->None:
        rate=rospy.Rate(self.config['publish_rate_hz']); interval=1.0/self.config['health_heartbeat_hz']
        while not rospy.is_shutdown():
            now_sec=self.clock.now_business_sec(); heartbeat=self.core.heartbeat_state(now_sec=now_sec)
            if heartbeat is not None: self._publish_state(heartbeat.to_dict())
            terminal=self.core.maybe_timeout(now_sec=now_sec)
            if terminal is not None: self._publish_state(terminal.to_dict())
            details=self._health_details(); sig=('ok','behavior_actuator_ready',JsonCodec.dumps(details))
            if sig!=self._last_health_signature or (now_sec-self._last_health_emit_sec)>=interval: self._publish_health('ok','behavior_actuator_ready',details)
            rate.sleep()

def main()->int:
    node=BehaviorActuatorNode(); node.spin(); return 0
if __name__=='__main__':
    raise SystemExit(main())
