/* 
 Test 1: Check if Arduino works fine by blinking the on-board LED:
 give permission (run on the terminal): sudo chmod 666 /dev/ttyACM0
*/

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on
  delay(500);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(500);                      // wait for a second
}
