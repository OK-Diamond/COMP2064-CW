'''
Common code for other files
'''

from dataclasses import dataclass

@dataclass
class User:
    '''Dataclass for users'''
    name:str
    dob:str
    register_time:float

@dataclass
class MqttTopic:
    '''Dataclass for topics'''
    name:str
    topic:str

@dataclass
class Topics:
    '''MQTT Topics'''
    staff:MqttTopic
    user:MqttTopic
    pairing: MqttTopic
    usr: MqttTopic
