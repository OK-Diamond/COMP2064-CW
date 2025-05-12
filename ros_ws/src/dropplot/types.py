import errno
import json

from dropplot.logger import log_err

class Patient:
    def __init__(self, name, dob, register_time):
        self.name: str= name
        self.dob: str= dob
        self.register_time: float= register_time

    def get_name(self):
        return self.name

    def get_dob(self):
        return self.dob

    def get_register_time(self) -> float:
        return self.register_time

    def __str__(self):
        return f"PATIENT: {self.name} born {self.dob}"

class Pairing:
    def __init__(self, patient, gp):
        self.patient: Patient= patient
        self.gp: str= gp

class DataEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, Patient):
            return o.__dict__
        if isinstance(o, Pairing):
            return o.__dict__

        # could use super call here, but I'd rather the encoder on be used for patients
        log_err("Unable to encode non-patient and non-pairing data", errno.EINVAL)

def encode_data(data): # types based on DataEncoder
    return json.dumps(data, cls=DataEncoder)

def _decode_patient(data):
    if "name" in data and "dob" in data and "register_time" in data:
        return Patient(data["name"], data["dob"], data["register_time"])

    log_err("Unable to decode data into patient that does not contain both name and reason values", errno.EINVAL)
    return None

def decode_patient(data):
    return json.loads(data, object_hook=_decode_patient)

def _decode_pairing(data):
    if "gp" in data and "user" in data:
        patient= decode_patient(data["user"])
        if patient:
            return Pairing(patient, data["gp"])
    log_err(f"Unable to decode pairing data {data}", errno.EINVAL)

def decode_pairing(data):
    return json.loads(data, object_hook=_decode_pairing)