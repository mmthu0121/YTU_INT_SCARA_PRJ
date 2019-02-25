int state = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
      state = Serial.read()- '0'; //http://forum.arduino.cc/index.php?topic=288234.0 or http://forum.arduino.cc/index.php?topic=344013.0
     
      if  (state != 0){
      Serial.println(state);
      }
  }

}
