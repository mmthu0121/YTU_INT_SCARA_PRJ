int state = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
      state = Serial.read()- '0';
     
      if  (state != 0){
      Serial.println(state);
      }
  }

}
