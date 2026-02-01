// Define the setup function, which runs once at the start of the program
void setup() {
  // Initialize the serial communication at 9600 bits per second
  Serial.begin(9600);
  
  // Print a message to the serial monitor to indicate the program has started
  Serial.println("Arduino initialization code");
}

// Define the loop function, which runs repeatedly after the setup function
void loop() {
  // Add your main program code here
  Serial.println("Loop running...");
  
  // Delay for 1 second before running the loop again
  delay(1000);
}