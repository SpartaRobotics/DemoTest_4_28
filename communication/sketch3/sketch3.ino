
int phase = 1;
char msg;

void setup() {
  Serial3.begin(115200);
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
  delay(1000);
  //Serial3.println(sent);
  //Serial3.println(sent);
  Serial3.write("1");
  Serial3.println();
  if(Serial3.available() > 0)
  {
    msg = Serial3.read();
    if(msg == receive)
        phase++;
  }
}

