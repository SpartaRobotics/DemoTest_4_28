void setup() {
  // put your setup code here, to run once:
  Serial3.begin(115200);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial3.write("1");
  Serial3.println();
  delay(1000);
}
