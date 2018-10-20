#define BUFFER_LENGTH 32
#define DEBUG

bool _motorActive = false;
int _steeringSetpoint = 0;
char receivedMsg[BUFFER_LENGTH];

void setup() {
  Serial.begin(57600);
  while (!Serial) {
    ;
  }
}

void loop() {
  while (!Serial.available()) {
    ;
  }
  int numReceived = Serial.readBytes(receivedMsg, BUFFER_LENGTH);
  #ifdef DEBUG
  if (numReceived) {
    Serial.print("Received: ");
    Serial.println(receivedMsg);
  }
  #endif

  //Assuming message format "m: [0/1]" for motor off/on, "s: [-180,180]" for steering angle setpoint.
  for (int msgIter = 0; msgIter < BUFFER_LENGTH; msgIter++) {
    if (receivedMsg[msgIter] == 'm') {
      if (receivedMsg[msgIter+3] == '1') {
        SetMotor(1);
      }
      else if (receivedMsg[msgIter+3] == '0') {
        SetMotor(0);
      }
    }
    else if (receivedMsg[msgIter] == 's') {
      String receivedSteeringString = "";
      while (receivedMsg[msgIter+3] != '\0' && receivedMsg[msgIter+3] != ' ') {
        receivedSteeringString += receivedMsg[msgIter+3];
        msgIter += 1;
      }
      SetSteering(receivedSteeringString.toInt());
    }
  }
}

void SetMotor (bool newState) {
  #ifdef DEBUG
  Serial.print("Setting new motor state: ");
  Serial.println(newState);
  #endif
  _motorActive = newState;
  return;
}

void SetSteering (int newAngle) {
  if (newAngle < -180 || newAngle > 180) {
    #ifdef DEBUG
    Serial.println("Steering angle out of range");
    #endif
    return;
  }
  #ifdef DEBUG
  Serial.print("Setting new steering angle: ");
  Serial.println(newAngle);
  #endif
  _steeringSetpoint = newAngle;
  return;
}
 
