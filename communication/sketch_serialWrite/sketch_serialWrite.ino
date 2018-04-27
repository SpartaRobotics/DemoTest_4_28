
int phase = 1;
char msg;
char sent; 
char receive;

void setup() {
  Serial3.begin(115200);
}

void loop() {
  if( phase == 1)
  {
    // Capture Phase
    sent = '1';
    receive = '2';
    missionPhase();
  }
  else if( phase == 2)
  {
    // Docking Phase
    sent = '3';
    receive = '4';
    missionPhase();
  }
  else if( phase == 3)
  {
    // Refueling Phase
    sent = '5';
    receive = '6';
    missionPhase();
  }
  else if( phase == 4)
  {
    // Return Phase
    sent = '7';
    receive = '8';
    missionPhase();
  }
  else if( phase == 5)
  {
    // Mission Finished
    phase = 6;
  }

}

int missionPhase()
{

  Serial3.write(sent);
  Serial3.println();

  if(Serial3.available() > 0)
  {
    msg = Serial3.read();
    if(msg == receive)
    {
        phase++;
    }
  }
}

