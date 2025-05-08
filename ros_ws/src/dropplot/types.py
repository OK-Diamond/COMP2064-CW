import json

class Patient:
    def __init__(self, name, reason):
        self.name= name
        self.reason= reason

    def get_name(self):
        return self.name

    def get_reason(self):
        return self.reason

    def to_dict(self):
        return {
            "name": self.get_name(),
            "reason": self.get_reason()
        }

    @classmethod
    def from_dict(cls, data):
        return cls(data["name"], data["reason"])

    def __str__(self):
        return f"PATIENT: {self.name} for {self.reason}"

class PatientEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, Patient):
            return o.to_dict()
        print("Unable to encode non-patient data") # could use super call here, but I'd rather the encoder on be used for patients

def encode_patient(patient):
    return json.dumps(patient, cls=PatientEncoder)

def _decode_patient(data):
    if "name" in data and "reason" in data:
        return Patient(data["name"], data["reason"])

    print("Unable to decode data into patient that does not contain both name and reason values")
    return None

def decode_patient(data):
    return json.loads(data, object_hook=_decode_patient)
