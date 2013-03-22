void printdata(void)
{    
      Serial.print("!");

      Serial.print("GYRO,");
      Serial.print(AN[0]);  //(int)read_adc(0)
      Serial.print(",");
      Serial.print(AN[1]);
      Serial.print(",");
      Serial.print(AN[2]);  
      Serial.print(",ACC,");
      Serial.print(AN[3]);
      Serial.print (",");
      Serial.print(AN[4]);
      Serial.print (",");
      Serial.print(AN[5]);
      Serial.print(",MAG,");
      Serial.print(AN[6]);
      Serial.print (",");
      Serial.print(AN[7]);
      Serial.print (",");
      Serial.print(AN[8]);
      Serial.print (",");
      Serial.print(G_Dt,5);
      Serial.print (",");
      Serial.print(sonarRange);
      Serial.print (",");
      Serial.print(sonarNew);
      Serial.println();    
      sonarNew = 0;
}


