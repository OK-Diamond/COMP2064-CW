from enum import Enum

import rospy
from lodestone.msg import Log

log_id: int= 0
source_id: str= "None"
log_pub: rospy.Publisher

class LOG_TYPE(Enum):
    INFO=0
    WARN=1
    ERR =2
    NONE=5

def get_id() -> int:
    global log_id

    ret= log_id
    log_id+= 1

    return ret

def log(message: str, errno: int=0, l_id=None) -> None:
    l= Log()
    l.header.stamp= rospy.Time().now()
    l.message= message
    l.source= source_id
    l.id= get_id() if l_id is None else l_id
    l.errno= errno

    log_pub.publish(l)

def log_err(message: str, errno: int, l_id=None) -> None:
    log(message, -abs(errno), l_id)

def log_warn(message: str, errno: int, l_id=None) -> None:
    log(message, abs(errno), l_id)

def log_info(message: str, l_id=None) -> None:
    log(message, 0, l_id)

# This function MUST be called before any logging! and preferably only once!
def init_log_pub(source: str) -> None:
    global log_pub, source_id
    source_id= source
    LOG_TOPIC= rospy.get_param("LOG_TOPIC")
    log_pub= rospy.Publisher(LOG_TOPIC, Log, queue_size= 10)
