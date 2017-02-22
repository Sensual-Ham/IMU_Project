// This code was written to test the arduino/matlab connection
void setup() {
  Serial.begin(9600);     
  Serial.print('a');      //handshake with matlab
  while (!Serial.available()); //wait for handshake to come back
  Serial.read();          //complete handshake
}

void loop() {
  float number = 1; //this will simulate data'
  int i;
  while(1){
    for (i = 0; i<6; i++){  //pretend there's six distinct values we're sending per timestep
      number+=0.001;
      Serial.print(number,4); //this is how each value should be printed. 4 is the number of post-decimal places, and can be adjusted to taste.
      Serial.print("\t");     //delimiter. Can be tab or space.
    }
    Serial.println(); //apparently the current script has newlines at the end of each timestep, so this simulates that.
  }
}
