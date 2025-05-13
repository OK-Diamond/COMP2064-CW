"""Stores shared type class information along with methods for decode/encoding those types"""

import errno
import json

from dropplot.logger import log_err

class Patient:
    """Patient data for registered users

        Attributes
        ----------
            name: str
                This is the patients name
            dob: str
                The date of birth of the patient
            register_time: float
                The time at which the user sent the registration request as time in
                seconds since the Epoch
    """
    def __init__(self, name, dob, register_time):
        self.name: str= name
        self.dob: str= dob
        self.register_time: float= register_time

    def __str__(self):
        return f"PATIENT: {self.name} born {self.dob}"

class Pairing:
    """Pairing data for GP-Patient pairings

        Attributes
        ----------
            patient: Patient
                The patient that has been paired
            gp: str
                The GP that has been paired
    """
    def __init__(self, patient, gp):
        self.patient: Patient= patient
        self.gp: str= gp

class DataEncoder(json.JSONEncoder):
    """Encoder based on the JSONEncoder used for transforming class data for json dumps"""
    def default(self, o):
        if isinstance(o, Patient):
            return o.__dict__
        if isinstance(o, Pairing):
            return o.__dict__

        log_err("Unable to encode non-patient and non-pairing data", errno.EINVAL)
        return super().default(o)

def encode_data(data): # types based on DataEncoder
    """Encode class data into json format"""
    return json.dumps(data, cls=DataEncoder)

def _decode_patient(data: dict):
    """decoding dictionary patient data into Patient class"""
    if "name" in data and "dob" in data and "register_time" in data:
        return Patient(data["name"], data["dob"], data["register_time"])

    log_err("Unable to decode data into patient that does not contain both name and reason values",
            errno.EINVAL)
    return None

def decode_patient(data: str):
    """decoding json data into Patient class"""
    return json.loads(data, object_hook=_decode_patient)

def _decode_pairing(data: dict):
    """decoding dictionary pairing data into Pairing class"""
    if "gp" in data and "user" in data:
        patient= decode_patient(data["user"])
        if patient:
            return Pairing(patient, data["gp"])

    log_err(f"Unable to decode pairing data {data}", errno.EINVAL)
    return None

def decode_pairing(data: str):
    """decoding json data into Pairing class"""
    return json.loads(data, object_hook=_decode_pairing)
