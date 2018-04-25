
int phase = 1;
char msg;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if( phase == 1)
  {
    // Capture Phase
    phase = missionPhase(1, '2', 1);
  }
  else if( phase == 2)
  {
    // Docking Phase
    phase = missionPhase(3, '4', 3);
  }
  else if( phase == 3)
  {
    // Refueling Phase
    phase = missionPhase(5, '6', 4);
  }
  else if( phase == 4)
  {
    // Return Phase
    phase = missionPhase(7, '8', 5);
  }
  else if( phase == 5)
  {
    // Mission Finished
    phase = 6;
  }

}

  // missionePhase(sendMsg, receiveMsg, missionPhase)
int missionPhase(int sent, char receive, int phase)
{
  Serial.println(sent);
  if(Serial.available() > 0)
  {
    msg = Serial.read();
    if(msg == receive)
        phase++;
  }
  
  return phase;
}

