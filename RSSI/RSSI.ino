

int loop_counter = 0;
byte c;

byte Holds = 0;
byte Frameloss_h = 0;
byte Frameloss_l = 0;
byte MainFades_h = 0;
byte MainFades_l = 0;
byte RemoteFades_h = 0;
byte RemoteFades_l = 0;

int Frameloss = 0;
int MainFades = 0;
int RemoteFades = 0;


int counter = 0;
void setup()
{
  Serial.begin(115200);
  pinMode(9, OUTPUT);   
  //analogWrite(9, 0); 
  analogWrite(9,1);
  delay(1000);
}


void loop()
{


    

  

  
  
while(Serial.available()>=16)
  {
       
    c = Serial.read();

    if(c==0x02)
    {
      counter++;
      Holds = Serial.read();
      Frameloss_h = Serial.read();
      Frameloss_l = Serial.read();
      MainFades_h = Serial.read();
      MainFades_l = Serial.read();
      Serial.read();
      Serial.read();
      RemoteFades_h = Serial.read();
      RemoteFades_l = Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();    
    
    
    
    Frameloss = (int)Frameloss_l + 256*(int)Frameloss_h;
    MainFades = (int)MainFades_l + 256*(int)MainFades_h;
   
    //RemoteFades = (int)RemoteFades_l + 256*(int)RemoteFades_l;
    
    /*
    Serial.print("Holds:");
    Serial.print(Holds,DEC);
    Serial.print("\t");
    Serial.print("Frameloss:");
    Serial.print(Frameloss,DEC);
    Serial.print("\t");
    Serial.print("MainFades:");
    Serial.print(MainFades,DEC);
    Serial.print("\t");
    Serial.print("RemoteFades:");
    Serial.print(RemoteFades_h,BIN);
    Serial.print(RemoteFades_l,BIN);
    Serial.println();
    */
    }
    
      
    
      
  
  }
  
 if(counter>10)
   analogWrite(9,175);
        
}
