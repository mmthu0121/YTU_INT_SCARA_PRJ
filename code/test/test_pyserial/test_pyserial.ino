//Serial Test Script

int count = 0;
int t = 0;
String readStr;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
    t++;
    if (t>5)
    {
      count++;
    if (count == 1)
    {
      Serial.println("Count = 10");
      count = 0;
      char c = Serial.read();
      readStr += c;
      if (readStr.length() >0)
      {
        Serial.println(readStr);
        if (readStr.length() >4)
        {
           readStr = 0; 
        }
      }
      
    }
    else
    {
      Serial.println("Too soon");
    }
    delay(1000);
    }
}
