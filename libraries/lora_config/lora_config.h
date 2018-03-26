/* 
/* 
 *  LoRA_config.h : Common header file for  Lora Radio sketches that includes setups like
 *  frequency , Spreading facotr, TX Power,  plus some utility  methods for GPS encoding and decoding
 *  
*/	


#define LORA_TX_Power  14  // - TX power in dB, defaults to 17 laws usually 433E6 , 866E6 or 915E6
#define LORA_SpreadingFactor  7 // MUST MATCH Sender: ranges from 6-12, default 7 see API docs larger more range less data rate
#define FREQ 866E6  //MUST MATCH Sender: Define Lora Frequency depends on Regional laws usually 433E6 , 866E8 or 915E6
#define BEACON_INTERVAL 15*1000  //Define how often to sen out the lora signal 

#define MAX_GPS_STRING_SIZE  64 //Deines how big the GPS message is <64 bytes recommended

char  TxBuffer[MAX_GPS_STRING_SIZE+1]; //recommended 51 bytes is lora maximum at SFF 12 Transmit buffer 

/*  Defines GPS specific fields*/
char gps_sats[3]="00";
char DeviceID[6]="GDP01";
char gps_signal[2]="0";
char packet_count[3]="0";
char packet_message[16]="NO GPS";
char gps_time[10]="00:00:00";
char gps_date[10]="00-00-00";
char gps_lat[16]="00.000";
char gps_long[16]="00.000";
char gps_alt[16]="0.0";
char checksum[4]="---";
char gps_age[10]="00";

/* GPS Message related strings */
char gpsmessage[MAX_GPS_STRING_SIZE] = "GPS02|1|26|7|04:01:03|40.801503|-74.26623|127.95275" ;
char gpschecksum[]= "000";
char gpsbuf[MAX_GPS_STRING_SIZE];


/* getChecksum: returns a simple integer checksum given a string of characters
*/
uint8_t getCheckSum(char *stringg)
{
  int XOR = 0;  
  for (int i = 0; i < strlen(stringg); i++) 
  {
    XOR = XOR ^ stringg[i];
  }
  return XOR;
}

/*extractChecksum :looks for last pipe delimited field and extracts that as it checksum */
uint8_t extractChecksum(String msg)
{
String cksumm="";	
	
String originalmessage=(String)msg;
int pos=originalmessage.lastIndexOf("|"); //find the last seperator

cksumm=originalmessage.substring(pos+1,originalmessage.length() );  //get everything to right of delimiter

Serial.println("Original message "+String(msg) );
Serial.println("message checkesum "+String(cksumm) );

return cksumm.toInt();  //return the checksum as an integer
}


bool validMessage(String msg)
{
  char themessage[MAX_GPS_STRING_SIZE];
   uint8_t encodedCheckSum=extractChecksum(msg );  //extract the checksum from this message
   int pos=msg.lastIndexOf("|"); //find the last seperator
   String rawmessage=msg.substring(0,pos); //get just the message less the checksum
   strcpy(themessage,rawmessage.c_str() );
   uint8_t calculated_checksum=getCheckSum( themessage);  //now lets calculate our own checksum
	
	if (encodedCheckSum==calculated_checksum  )
		return true;
	else
		return false;
	
}

/* splits a  recieved GPS string into its consituent fields, return boolean true if value , false if not 
based on following pipe delimited format:
// Pipe delimted format
// DeviceID | GPS Signal Valid | MSG COUNT| SATS/MSG| Latitude | Longitude | Altitude | hecksum
returns true if valid message (ie. checksum ok) of successfully scanned parameters
*/
int decodeGPSString(char* Rxmsg)
{

int n = sscanf(Rxmsg,"%[^|]|%[^|]|%[^|]|%[^|]|%[^|]|%[^|]|%[^|]|%[^|]%s",DeviceID ,gps_signal ,packet_count ,packet_message ,gps_time,gps_lat,gps_long,gps_alt,checksum );

Serial.printf("--SSCANF RESULT ---------------------------------------------------------------- \n" );
Serial.printf("%s|%s|%s|%s|%s|%s|%s|%s|%s \n",DeviceID ,gps_signal ,packet_count ,packet_message ,gps_time,gps_lat,gps_long,gps_alt,checksum );
Serial.printf("--ORIGINAL MESSAGE ---------------------------------------------------------------- \n" );
Serial.println(Rxmsg);
Serial.printf("------------------------------------------------------------------------ \n" );

return validMessage( String(Rxmsg) );
}

/* Encodes global GPS variables into a pipe delimted formattted message TxBuffer string, returns message size */
int  encodeGPSString()
{

char gpsmessage[MAX_GPS_STRING_SIZE];
char gpschecksum[]= "000";

sprintf(gpsmessage,"%s|%s|%s|%s|%s|%s|%s|%s",DeviceID ,gps_signal ,packet_count ,packet_message ,gps_time,gps_lat,gps_long,gps_alt);

Serial.printf("--SPRINTF  RESULT ---------------------------------------------------------------- \n" );
Serial.println(gpsmessage);

itoa(getCheckSum(gpsmessage),gpschecksum,10);
snprintf(TxBuffer, sizeof TxBuffer, "%s|%s", gpsmessage, gpschecksum);
Serial.printf("--CHECKSUM ---------------------------------------------------------------- \n" );
Serial.print("Checksum: ");
Serial.println(TxBuffer);

return strlen(TxBuffer); //lets return the buffer with the encoded checksum value

}


/* Alternate checksum method */
uint8_t chksum8(char *buff, size_t len)
{
    unsigned int sum;       // nothing gained in using smaller types!
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);   // parenthesis not required!
    return (uint8_t)sum;
}
