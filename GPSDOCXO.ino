/*
  A GPS Disciplined Oven Controlled Crystal Oscillator
  Designed, Programmed and Assembled by Fischer Moseley in August 2016


  Notes:
  gpsdata is indexed starting at 0, so gpsdata[0]=0x40, gpsdata[BYTE_LENGTH-1]=0x0a, gpsdata[BYTE_LENGTH-2]=0x0d
  LCD datasheet can be found at:
  http://media.nkcelectronics.com/datasheet/LCM2004SD-NSW-BBW.pdf
*/
#include <SoftwareSerial.h> //for communicating with the GPS and the LCD
#include <Wire.h>//for I2C with the DAC
#include <EEPROM.h>//for storing/retrieving the previous DAC value

//LCD variables
SoftwareSerial lcdserial(4,3); //RX, TX define virtual serial port the LCD is on
byte set_contrast[] = {0xFE, 0x52, 0x25}; //set the contrast to 0x25 on a scale of 0x00 to 0x50
byte clear_screen[] = {0xFE, 0x51};//clear the LCD
byte cursor_home[] = {0xFE, 0x46};//set the cursor to 0,0
byte second_row[] = {0xFE, 0x45, 0x40};//set the cursor to column 1, line 2
byte third_row[] = {0xFE, 0x45, 0x14};//set the cursor to column 1, line 3
byte fourth_row[] = {0xFE, 0x45, 0x54};//set the cursor to column 1, line 4
byte lcd_menu=0;//the menu that the LCD is currently displaying

//GPS parsing constants
#define BYTE_LENGTH 54 //how long the response from the oncore is
#define MONTH_LOC 4 //where in the array the month element is
#define DAY_LOC 5 //where in the array the day element is
#define YEAR_LOC 6 //where in the array the year elements are, this is the first element, so there is also one at YEAR_LOC+1
#define HOUR_LOC 8 //where in the array the hour element is
#define MIN_LOC 9 //where in the array the minute element is
#define SEC_LOC 10 //where in the array the second element is
#define LAT_LOC 15 //where in the array the latitude element is, first of 4 elements
#define LONG_LOC 19 //where in the array the longitude element is, first of 4 elements
#define HEIGHT_LOC 23 //where in the array the GPS height element is, first of 4 elements
#define VSAT_LOC 39 //where in the array the visible satalite element is
#define TSAT_LOC 40 //where in the array the visible satalite element is  
#define STAT_LOC 41 //where in the array the reciever status is
#define CHKSUM_LOC 51 //where in the array the checksum is

//GPS parsing variables
SoftwareSerial gpsserial(10,11); //RX,TX, define the virtual serial port the oncore is on
byte gpsdata[BYTE_LENGTH] = {0}; //create an array to dump the serial data into for parsing
byte Hb[] = {0x40, 0x40, 0x48, 0x62, 0x01, 0x2B, 0x0D, 0x0A}; //bytes to request serial data from oncore
byte Cf[] = {0x40, 0x40, 0x43, 0x66, 0x25, 0x0D, 0x0A}; //set of bytes that resets the oncore
bool ant_bit1=false;//bits extracted from the antenna status byte
bool ant_bit2=false;

//hardware constants
#define ONBOARD_LED_PIN 13 //which pin the stat led is attached to
#define ONBOARD_LED_PIN_RED 6//which pin the red status LED is connected to
#define ONBOARD_LED_PIN_GREEN 7//which pin the green status LED is connected to
#define GPS_PIN_RED 9//which pin the red GPS LED is connected to
#define GPS_PIN_GREEN 12//which pin the green GPS LED is connected to
#define WARMUP_BYPASS_PIN A0//which pin the button for bypassing warmup is connected to
#define CYCLE_PIN A1//which pin the button for cycling the menu on the LCD is connected to
#define CLK_PIN 5//which pin the clock goes to
#define INT0_PIN 2//which pin the interrrupt goes to
#define DAC_ADDR 0x54//DAC I2C address
#define DAC_HB_ADDR 0x00//Address in EEPROM where the high byte of the DAC is
#define DAC_LB_ADDR 0x01//Address in EEPROM where the low byte of the DAC is

//modal constants
#define CSV_MODE 1//0 if in human readable data output, 1 to output data as CSV
bool ENFORCE_WARMUP=true;//true to not compute until the OCXO is warm, false to skip the check, not a constant so it can be changed by the user
#define BAUDRATE 115200//the baudrate of the connection from PC<->Arduino
#define USE_DEFAULT_DAC 0//nonzero if want to put in our own DAC value to begin with, zero to disable functionality
#define DEFAULT_DAC_VALUE 21900//the value to use and write in EEPROM if there isn't a DAC value or if we want to override it

//algorithim constants
#define MIN_SATS 5//minimum amount of satellites to be tracking before starting the counter
#define WARMUP_TIME 1800//time in seconds to wait before starting the counter, 1800=30min
#define LOW_CUTOFF 19257//lowest possible counter value that will be averaged
#define HIGH_CUTOFF 19271//highest possible counter value that will be averaged
#define MEAS_INT_TIME 2500//amount of time to average data for
#define LSB_PER_MHZ 1.2//how many LSBs to add/subract per milihertz of error in 10Mhz
#define DEAD_ZONE .1//least possible error to exist to warrant compensation, in millihertz
#define QT_INT_TIME 600//amount of time that QuickTune will average data for
#define MIN_MEAS_PERS 1//how many periods the GPSDO must go through before it changes from Measurement to QuickTune

//algorithim variables
bool ALG_IN_USE=false;//flag to track what algorithim is currently running, false for measure and discipline, true for single LSB adjustments (quicktune)
bool USER_CHANGED_ALG=false;//flag to track if the user has changed the algorithim, and if so, don't do the alg update based on done_per
bool osc_temp_flag=false;//flag to track if the crystal was ever warm during its runtime, included for millis() overflow protection
bool rel_cnt_ready=false;//flag to track if there's a new value in rel_cnt to check
bool have_first_sample=false;//flag to track if we've got the first sample
unsigned int rel_cnt=0;//what the current (relative) count is
unsigned int last_cnt=0;//what the last counter value was
int run_sum=0;//running sum of the errors
unsigned int samples=0;//amount of samples accumulated
unsigned int done_per=0;//amount of integration periods performed
unsigned int dac_value=0;//the value in the DAC's register
unsigned int abs_cnt=0;//the absolute counter value, or the delta between the current and last
float error_mhz=0;//the error, in millihertz, away from the ideal
float run_avg=0;//the running sum divided by samples
int rs_up_votes=0;//the amount of votes that want to increase the DAC, derived from the running sum
int rs_down_votes=0;//the amount of votes that want to decrease the DAC, derived from the running sum
int rs_no_change_votes=0;//the amount of votes that don't want to change the DAC, derived from the running sum
int cnt_up_votes=0;//the amount of votes that want to increase the DAC, derived from the absolute count
int cnt_down_votes=0;//the amount of votes that want to decrease the DAC, derived from the absolute count
int cnt_no_change_votes=0;//the amount of votes that don't want to change the DAC, derived from the absolute count

//DAC history variables
unsigned int last_dac=0;
unsigned int second_dac=0;

//uptime variables
byte up_sec=0;//amount of seconds the GPSDO has been running for
byte up_min=0;//amount of minutes the GPSDO has been running for
byte up_hour=0;//amount of hours the GPSDO has been running for
byte up_day=0;//amount of days the GPSDO has been running for
byte up_month=0;//amount of months the GPSDO has been running for

//location variables
float latitude=0;
float longitude=0;

void setup(){
  //serial setup
  Serial.begin(BAUDRATE);//initialize PC <-> Arduino serial port
  lcdserial.begin(9600);//initialize LCD <-> Arduino serial port
  lcdserial.write(set_contrast, sizeof(set_contrast));//set the contrast of the LCD
  lcdserial.write(clear_screen, sizeof(clear_screen));//clear the LCD
  lcdserial.write(cursor_home, sizeof(cursor_home));//set the cursor to 0,0
  gpsserial.begin(9600);//initialize oncore <-> Arduino serial port
  gpsserial.write(Hb, sizeof(Hb)); //poll the oncore once per second for status
  gpsserial.listen();

  //I/O setup
  digitalWrite(ONBOARD_LED_PIN, LOW); //turn the LEDs off in case they were on before
  pinMode(CLK_PIN, INPUT); //make interrupt pin an input
  pinMode(INT0_PIN, INPUT); //make clock input pin an input
  pinMode(ONBOARD_LED_PIN_RED, OUTPUT);//set LED pins as outputs
  pinMode(ONBOARD_LED_PIN_GREEN, OUTPUT);
  pinMode(GPS_PIN_RED, OUTPUT);
  pinMode(GPS_PIN_GREEN, OUTPUT);
  pinMode(WARMUP_BYPASS_PIN, INPUT);//set button input pins as inputs 
  pinMode(CYCLE_PIN, INPUT);
  digitalWrite(WARMUP_BYPASS_PIN, HIGH);//enable internal pullups
  digitalWrite(CYCLE_PIN, HIGH);
  attachInterrupt(0, pps, RISING);//attach interrupt to 1PPS pin

  //DAC setup
  dac_value=readfromeeprom();//set the dac value to what's in EEPROM
  if(dac_value==65535||dac_value==0||USE_DEFAULT_DAC){//if it doesn't exist or we want to overwrite
    dac_value=DEFAULT_DAC_VALUE;//use the default value
    writetoeeprom(dac_value);//and write that default to EEPROM for next time
  }
  setdac(dac_value);//set the dac
  delay(1);
  setdac(dac_value);

  //counter setup
  TCCR1A = 0b00000000; //configure the control registers
  TCCR1B = 0b01000111;
}

void pps(){ //the function that is run when 1PPS goes high
//note that this isn't all of what happens when the PPS pulse is recieved, just keeping the ISR short, hence the rel_cnt_ready flag
  rel_cnt=ICR1;
  rel_cnt_ready=true;
}

void runstats(){
	if(!have_first_sample){last_cnt=rel_cnt; have_first_sample=true;}//if there's no first count then take one, set the flag
	else{
		rel_cnt=last_cnt>rel_cnt?rel_cnt+65536:rel_cnt;
		abs_cnt=rel_cnt-last_cnt;
		if(gpsdata[TSAT_LOC]>=MIN_SATS && LOW_CUTOFF<=abs_cnt && abs_cnt<=HIGH_CUTOFF && (warmedup()||!ENFORCE_WARMUP)){
			//where the statistics generation takes place
			run_sum=run_sum+(abs_cnt-19264);//find new running sum where the running (error) sum
			samples++;
			run_avg=float(run_sum)/float(samples);//calculate the average error
			error_mhz=run_avg*2000;//calculate the error in mHz, compensate for the n/2 divider
			//here is where the actual algorithims take place, after all the measurement statistics have been done

			//perform the voting
			if(run_sum>0){rs_up_votes++;}
			if(run_sum==0){rs_no_change_votes++;}
			if(run_sum<0){rs_down_votes++;}

			if(abs_cnt>19264){cnt_up_votes++;}
			if(abs_cnt<19264){cnt_down_votes++;}
			if(abs_cnt==19264){cnt_no_change_votes++;}

			//update the algorithim in use
			if(!USER_CHANGED_ALG){
				if(done_per<=MIN_MEAS_PERS){ALG_IN_USE=false;}
				if(done_per>MIN_MEAS_PERS){ALG_IN_USE=true;}
			}

			if(!ALG_IN_USE&&samples>=MEAS_INT_TIME){
				second_dac=last_dac;
      			last_dac=dac_value;
        		dac_value=dac_value+(LSB_PER_MHZ*error_mhz);//calculate how much to adjust the dac by
        		writetoeeprom(dac_value);//write the new value to EEPROM
        		setdac(dac_value);//set the dac to the new value
        		delay(1);
        		setdac(dac_value);
        		rs_up_votes=0;//reset votes
        		rs_down_votes=0;
        		rs_no_change_votes=0;
        		cnt_up_votes=0;
        		cnt_down_votes=0;
        		cnt_no_change_votes=0;
        		run_sum=0;//reset the running sum
        		samples=0;//reset the samples
        		done_per++;//tell us that we've got an integration period under our belt & increment
			}

			if(ALG_IN_USE&&(samples>=QT_INT_TIME||rs_up_votes>225||rs_down_votes>225||rs_no_change_votes>225)){
				second_dac=last_dac;
				last_dac=dac_value;
				if(rs_up_votes>rs_down_votes&&rs_up_votes>rs_no_change_votes){dac_value++; dac_value++;}
				if(rs_down_votes>rs_up_votes&&rs_down_votes>rs_no_change_votes){dac_value--; dac_value--;}
				writetoeeprom(dac_value);
				setdac(dac_value);
				delay(1);
				setdac(dac_value);
				samples=0;
				run_sum=0;
				rs_up_votes=0;//reset the votes
				rs_down_votes=0;
				rs_no_change_votes=0;
				cnt_up_votes=0;
				cnt_down_votes=0;
				cnt_no_change_votes=0;
				done_per++;
			}
		}
		last_cnt=rel_cnt;
	}
}


int getchecksum(){//returns the checksum of the gpsdata data
  byte carryover=0x00; //define and initialize the working variable
  for(int i=0; i<=BYTE_LENGTH-4; i++){ //BYTE_LENGTH-4, so we don't take the checksum of the checksum, 0d 0a, and compensate for 0 based indexing
      carryover=carryover^gpsdata[i]; //XOR it
   }
   return carryover; //return the XOR'd value
}

int checkstring() {//compares the calculated checksum with the actual checksum, and validates the preamble and terminator
  if (gpsdata[CHKSUM_LOC] == getchecksum() && gpsdata[0] == 0x40 && gpsdata[1] == 0x40 && gpsdata[BYTE_LENGTH - 1] == 0x0A &&
   gpsdata[BYTE_LENGTH - 2] == 0x0D && 0<=gpsdata[MIN_LOC]<=60 && 0<=gpsdata[SEC_LOC]<=60 && 0<=gpsdata[HOUR_LOC]<=24
   && 0<=gpsdata[MONTH_LOC] <= 12 && 0 < gpsdata[DAY_LOC] <=31) {
    //make sure that the checksum is the same, the first and second bytes of the preamble are 40, 
    //and that the last and second to last bytes are 0d 0a, that the minute is between 0 and 60, 
    //the second is between 0 and 60, the hour between 0 and 24
    return true; //if they all line up, then it is true
  }
  else{ //but if not
    return false; //say it's false
  }
}

void addzeroes(int q, bool mode){//append zeroes to variables less than ten, mode=true for serial, false for LCD serial
  if(mode){//if we're in PC serial mode
    if(q<10){//if the variable is less than 10
      Serial.print('0');//print an extra 0
    }
    Serial.print(q);//print the number itself, regardless of whether or not it got the extra zero
  }
  else{//if we're in LCD serial mode
  	if(q<10){//if the variable is less than ten
  		lcdserial.print('0');//print an exta 0
  	}
  	lcdserial.print(q);//print the number itself, regardless of whether or not it got the extra zero
  }
}

void printstatus(){//print the sat info, time, data, and fix a broken gpsdata if necessary
  #if CSV_MODE==1 //if we're in CSV mode

  	//GPS Stanza
    Serial.print(String(gpsdata[VSAT_LOC])+','+String(gpsdata[TSAT_LOC])+',');
    Serial.print(String(gpsdata[MONTH_LOC])+','+gpsdata[DAY_LOC]+','+String((gpsdata[YEAR_LOC]*0x100)+gpsdata[YEAR_LOC+1])+',');//print month day year
    Serial.print(String(gpsdata[HOUR_LOC])+','+String(gpsdata[MIN_LOC])+','+String(gpsdata[SEC_LOC])+',');//print hour minute second
    Serial.print(String(latitude,4)+','+String(longitude,4)+',');//print lat long

    //read the antenna status bits
    ant_bit1=bitRead(gpsdata[STAT_LOC+1],1);
    ant_bit2=bitRead(gpsdata[STAT_LOC+1],2);
    
    //antenna status is given as the combination of bits in the byte gpsdata[STAT_LOC+1]
    if(!ant_bit1&&!ant_bit2){Serial.print("0");}//Good
    if(!ant_bit1&&ant_bit2){Serial.print("1");}//Overcurrent
    if(ant_bit1&&!ant_bit2){Serial.print("2");}//Undercurrent
    if(ant_bit1&&ant_bit2){Serial.print("3");}//No Bias Voltage

    //Algorithim Stanza
    Serial.print(','+String(ALG_IN_USE)+','+String(abs_cnt)+','+(String(error_mhz,5))+','+String(samples)+','+String(done_per)+
    ','+String(dac_value)+','+String(run_sum)+','+String(rs_up_votes)+','+String(rs_down_votes)+','+String(rs_no_change_votes)+',');

    //Status Stanza
    Serial.println(String(ENFORCE_WARMUP)+','+String(warmedup())+','+String(up_month)+','
    +String(up_day)+','+String(up_hour)+','+String(up_min)+','+String(up_sec)+','+String(cnt_up_votes)+','
    +String(cnt_down_votes)+','+String(cnt_no_change_votes));
    if(!checkstring()){resetserial();}//reset the serial if it's corrupted
  #endif

  #if CSV_MODE==0//if we're not in CSV mode
    if(!checkstring()){Serial.print("FAIL, CLEARING... ");resetserial();}

    //GPS Time, Date, Location Data:
    Serial.print(" v: "+String(gpsdata[VSAT_LOC])+" t: "+String(gpsdata[TSAT_LOC])+' '+String(gpsdata[MONTH_LOC])+'/'+String(gpsdata[DAY_LOC])+'/');
    Serial.print(String((gpsdata[YEAR_LOC]*0x100)+gpsdata[YEAR_LOC+1])+' ');
    addzeroes(gpsdata[HOUR_LOC],true);//print hour
    Serial.print(':');
    addzeroes(gpsdata[MIN_LOC],true);//print minute
    Serial.print(':');
    addzeroes(gpsdata[SEC_LOC],true);//print second
    Serial.print(" UTC LOC: ");//let us know we're in UTC
    Serial.print(String(latitude,4)+", "+String(longitude,4)+' ');//print lat/long
    //Antenna bias status message:
    ant_bit1=bitRead(gpsdata[STAT_LOC+1],1);
    ant_bit2=bitRead(gpsdata[STAT_LOC+1],2);

    //Antenna status is defined by the combination of bits in the byte
    if(ant_bit1&&ant_bit2){Serial.print("[NO ANT BIAS VOLTAGE] ");}
    if(!ant_bit1&&ant_bit2){Serial.print("[ANT OVERCURRENT] ");}
    if(ant_bit1&&!ant_bit2){Serial.print("[ANT UNDERCURRENT] ");}
    //if(!ant_bit1&&!ant_bit2){Serial.print("[ANT OK] ");}

    //Error messages:
    if(ENFORCE_WARMUP && !warmedup()){//if it's cold and we're enforcing hot, print such
      Serial.print("OSC COLD: ");
      int remaining_minutes=(int)((WARMUP_TIME-(millis()/1000))/60);//calc minutes remaining in the warmup by dividing seconds by 60 and truncating
      Serial.print(String(remaining_minutes)+':');//print remaining minutes with a colon seperator
      addzeroes((WARMUP_TIME-(millis()/1000))-(remaining_minutes*60), true); //calc remaining seconds by subtracting how many seconds are 
                                                                                      //represented in remaining_minutes
      Serial.print(' ');
    }
    if(gpsdata[TSAT_LOC]<MIN_SATS){//if there aren't enough sats, print such
      Serial.print("[NOT ENOUGH SATS] ");
    }
    if(abs_cnt>HIGH_CUTOFF){//if the count value is a high outlier, print such
      Serial.print("[TOO HIGH] ");
    }
    if(abs_cnt<LOW_CUTOFF){//if the count value is a low outlier, print such 
      Serial.print("[TOO LOW] ");
    }

    //Algorithim Data:
    Serial.println("cnt: "+String(abs_cnt)+" error_mHz: "+String(error_mhz,3)      +" samples: "+String(samples)+
	" done_per: " +String(done_per)+ " DAC: " +String(dac_value)+"run_sum: "+String(run_sum)+"up: "+String(rs_up_votes)+" down: "+
	String(rs_down_votes)+" dn: "+String(rs_no_change_votes));
    #endif
}

void resetserial(){//sets all gpsdata elements to 0, clears readbuffer
  while(gpsserial.available()>0){//if there are any bytes
    byte oblivion=gpsserial.read();//read them into oblivion
  }
  for(byte i; i<BYTE_LENGTH; i++){//write zeroes to all array elements
    gpsdata[i]=0;
  }
}

void updatestatusleds(){//updates the status LEDs with new data
  if(gpsdata[TSAT_LOC]>0){//if we're tracking at least one sat
    digitalWrite(ONBOARD_LED_PIN, HIGH); //turn the onboard LED on
  }
  if(gpsdata[TSAT_LOC]==0){//but if we're not tracking any sats
    digitalWrite(ONBOARD_LED_PIN, LOW);//turn the onboard LED off
    digitalWrite(GPS_PIN_RED, HIGH);//make the front panel GPS led red
    digitalWrite(GPS_PIN_GREEN, LOW);
  }
  if(gpsdata[TSAT_LOC]>=MIN_SATS){//if there's enough sats to begin counting
    digitalWrite(GPS_PIN_GREEN, HIGH);//make the front panel GPS led green
    digitalWrite(GPS_PIN_RED, LOW);
  }
  if(gpsdata[TSAT_LOC]>0 && gpsdata[TSAT_LOC]<MIN_SATS){//if we're tracking at least one sat, but not enough to begin counting
    digitalWrite(GPS_PIN_RED, HIGH);//make the front panel GPS led yellow
    digitalWrite(GPS_PIN_GREEN, HIGH);
  }
  if((ENFORCE_WARMUP&&!warmedup())||gpsdata[TSAT_LOC]<MIN_SATS){//if we're not counting
    digitalWrite(ONBOARD_LED_PIN_RED, HIGH);//make the status LED red
    digitalWrite(ONBOARD_LED_PIN_GREEN, LOW);
  }
  if(!(ENFORCE_WARMUP&&!warmedup())&&gpsdata[TSAT_LOC]>=MIN_SATS){//if we're counting
    digitalWrite(ONBOARD_LED_PIN_RED, LOW);//make the status LED green
    digitalWrite(ONBOARD_LED_PIN_GREEN, HIGH);
  }
}

void setdac(unsigned int val){//set the DAC's output 
  byte msbyte;//define high byte
  byte lsbyte;//define low byte

  //shift it over so it can be written in two parts
  msbyte = byte((val & 0xFF00)>>8); //first eight bits
  lsbyte = byte(val & 0x00FF); //last eight bits
  Wire.beginTransmission(DAC_ADDR); //start the data transfer with the address specified
  Wire.write(0x01);  //command to load code into the register and output to r2r circuit (CODE_LOAD) in datasheet I think
  Wire.write(msbyte);  // MS Byte
  Wire.write(lsbyte);  // LS Byte
  Wire.endTransmission();
  Serial.print("[DAC WRITTEN] ");
}

void writetoeeprom(unsigned int val){//write an unsigned to EEPROM
  EEPROM.write(DAC_LB_ADDR, lowByte(val));//write the low byte of the DAC value
  EEPROM.write(DAC_HB_ADDR, highByte(val));//write the high byte of the DAC value
}

int readfromeeprom(){//read an unsigned int from EEPROM
  unsigned int combined=EEPROM.read(DAC_HB_ADDR)<<8;
  combined|=EEPROM.read(DAC_LB_ADDR);
  return combined;
}

void readgpsstatus(){//grabs data from the GPS on the serial port, calculates lat/long
  for(byte i=0; i<BYTE_LENGTH; i++){gpsdata[i] = gpsserial.read();} //read as many bytes as are defined into readbuffer
   latitude=(gpsdata[LAT_LOC+3]+(gpsdata[LAT_LOC+2]*0x100)+(gpsdata[LAT_LOC+1]*0x10000)+(gpsdata[LAT_LOC]*0x1000000))/3600000.0;
   longitude=(gpsdata[LONG_LOC+3]+(gpsdata[LONG_LOC+2]*0x100)+(gpsdata[LONG_LOC+1]*0x10000)+(gpsdata[LONG_LOC]*0x1000000))/3600000.0;
}

int warmedup(){//returns if the oscillator is warmed up, includes millis() overflow protection
  if((millis()/1000)>WARMUP_TIME||osc_temp_flag){//if the system has been on for long enough at one point
    osc_temp_flag=true;//set the flag to true, so this function latches on
    return true;//return true
  }
  else{//but if not
    return false;
  }
}

void updateuptime(){//updates local timekeeping
  //notes: 31 days to a month
  //using local timekeeping guards against leap seconds
  //if a leap second were to happen while gettting time from GPS, uptime would be one second higher than the actual amount of time it ran for 
  up_sec++;
  if(up_sec>=60){
    up_min++;
    up_sec=0;
  }
  if(up_min>=60){
    up_hour++;
    up_min=0;
  }
  if(up_hour>=24){
    up_day++;
    up_hour=0;
  }
  if(up_day>=31){
    up_month++;
    up_day=0;
  }
}

void updateLCD(){//update the LCD
	lcdserial.write(clear_screen, sizeof(clear_screen));//clear the LCD
	lcdserial.write(cursor_home, sizeof(cursor_home));//home the cursor
	/*
  	LCD Menu definitions 
  	0=GPS menu, visible sats, tracked sats, time, date, latitude, longitude, altitude, antenna status
  	1=Algorithim menu, runtime, done integration periods, error, time until next dac recalculation, time left on warmup
  	*/
  	switch(lcd_menu){
    	case 0://GPS menu
    		//Line 1: Date and Time
      		lcdserial.print(String(gpsdata[MONTH_LOC])+'/'+String(gpsdata[DAY_LOC])+'/'+String((gpsdata[YEAR_LOC]*0x100)+gpsdata[YEAR_LOC+1])+"  ");//print date
      		if(gpsdata[MONTH_LOC]<10){lcdserial.print(' ');}//if the month is less than ten add an extra space so that the time is right-justified
      		if(gpsdata[DAY_LOC]<10){lcdserial.print(' ');}//if the day is less than ten add an extra space so that the time is right-justified
      		if(gpsdata[HOUR_LOC]<10){lcdserial.print(' ');}//if the hour is less than ten add an extra space so that the time is right-justified
      		lcdserial.print(gpsdata[HOUR_LOC]);//print the hour
      		lcdserial.print(':');//colon seperator
      		addzeroes(gpsdata[MIN_LOC],false);//add zeroes to the minutes
      		lcdserial.print(':');//colon seperator
      		addzeroes(gpsdata[SEC_LOC],false);//add zeroes to the seconds
      		//Line 2: Location Coordinates
      		lcdserial.write(second_row, sizeof(second_row));//set the cursor to the second row
      		lcdserial.print("LOC "+String(latitude,3)+", "+String(longitude,3));//print the current location
      		lcdserial.write(third_row, sizeof(third_row));//set the cursor to the third row
      		//Line 3: Visible and Tracked Satellites
      		lcdserial.print(String(gpsdata[VSAT_LOC])+" Visible ");//print visible satellites
      		lcdserial.print(String(gpsdata[TSAT_LOC])+" Tracked");//print tracked satellites
      		lcdserial.write(fourth_row, sizeof(fourth_row));//set the cursor to the fourth row
      		//Line 4: Antenna Bias Message, defined by a combination of bits
    		ant_bit1=bitRead(gpsdata[STAT_LOC+1],1);//get the first bit from the antenna status byte
   			ant_bit2=bitRead(gpsdata[STAT_LOC+1],2);//get the second bit from the antenna status byte
   			if(!ant_bit1&&!ant_bit2){lcdserial.print("ANT GOOD");}
   			if(ant_bit1&&ant_bit2){lcdserial.print("NO ANT VOLTAGE");}
   			if(!ant_bit1&&ant_bit2){lcdserial.print("ANT OVERCURRENT");}
   			if(ant_bit1&&!ant_bit2){lcdserial.print("ANT UNDERCURRENT");}
      		break;
    	case 1://Algorithim Menu
    		//Line 1: Error Estimate and EFC LSBs
      		lcdserial.print(String(error_mhz,2)+" mHz V: ");
      		lcdserial.print(String(rs_up_votes)+':'+String(rs_down_votes)+':'+String(rs_no_change_votes));
      		lcdserial.write(second_row, sizeof(second_row));
      		//Line 2: Time left on Warmup or Time to next adjustment
      		if(!warmedup()&&ENFORCE_WARMUP){
      			int rem_min=(int)((WARMUP_TIME-(millis()/1000))/60);
      			lcdserial.print("Ready in: "+String(rem_min)+':');
      			addzeroes((WARMUP_TIME-(millis()/1000))-(rem_min*60),false);
      		}
      		else{
      			int time_left=0;
      			if(ALG_IN_USE){time_left=(QT_INT_TIME-(samples-1));}
      			else{time_left=(MEAS_INT_TIME-(samples-1));}
      			int r_m=time_left/60;
      			lcdserial.print("DAC Adj in: "+String(r_m)+':');
      			addzeroes(time_left-(r_m*60),false);
      		}
      		//Line 4: Uptime
      		lcdserial.print("Uptime: ");
      		if(up_month){
        		lcdserial.print(String(up_month)+':');
        		addzeroes(up_day,false);
        		lcdserial.print(':');
        		addzeroes(up_hour,false);
        		lcdserial.print('h');
      		}
      		if(up_day&&!up_month){
        		lcdserial.print(String(up_day)+':');
        		addzeroes(up_hour,false);
        		lcdserial.print(':');
        		addzeroes(up_min,false);
        		lcdserial.print('m');
      		}
      		else{
        		lcdserial.print(String(up_hour)+':');
        		addzeroes(up_min,false);
        		lcdserial.print(':');
        		addzeroes(up_sec, false);
        		lcdserial.print('s');
      		}
      		break;
    	default:
      	lcdserial.print("HE'S DEAD, JIM!"+String(lcd_menu));
    	break;
  }
}

void loop(){
  if(rel_cnt_ready){//if there's a 1PPS signal
  	delay(1);//delay a bit so that the signal can get to the ICP pin?
    runstats();
    rel_cnt_ready=false;
  }

  //PPS signal happens before the GPS status message, which lets this work and so it's not a second behind
  if(gpsserial.available()>=BYTE_LENGTH) { //if there is new data from the GPS
  	//all the once per second processing happens in here
  	updateuptime();//add a second to the runtime of the GPSDO
    readgpsstatus();//read them into gpsdata
    updatestatusleds();//update the status LEDs, 
    printstatus();//print the satellite status, date, time, algorithim data over serial
    updateLCD();//update the LCD with the new everything
    if(Serial.available()>0&&Serial.available()<8){//if there's something stuck in the readbuffer clear it
    	for(int i=Serial.available(); i>0; i--){char q=Serial.read();}
    }

  }

  if(Serial.available()>=8){//if there are any commands from the PC
  	String q=Serial.readString();
  	if(q=="RSTGPS\r\n"|| q=="RSTGPS\r"||q=="RSTGPS\n"){//send the reset command to the GPS, supports either CR or LF or both
  		gpsserial.write(Cf, sizeof(Cf));
  		Serial.print("[GPS RESET] ");
  	}

  	else if(q=="RSTALG\r\n"||q=="RSTALG\r"||q=="RSTALG\n"){//reset all the algorithim variables
  		error_mhz=0;
  		samples=0;
  		run_sum=0;
  		run_avg=0;
  		last_cnt=0;
  		abs_cnt=0;
  		rel_cnt=0;
  		done_per=0;
  		rs_up_votes=0;
  		rs_down_votes=0;
  		rs_no_change_votes=0;
  		have_first_sample=false;
  	}

  	else if(q=="CHGALG\r\n"||q=="CHGALG\r"||q=="CHGALG\n"){//switch the algorithim
  		USER_CHANGED_ALG=true;//note that the user has overridden the alg in use
  		ALG_IN_USE=!ALG_IN_USE;//invert the algorithim in use
  		Serial.print("[ALGORITHIM CHANGED] ");//tell us that we changed the algorithim
  	}

  	else if(q=="STRDAC\r\n"||q=="STRDAC\r"||q=="STRDAC\n"){//store the current dac value in EERPROM
  		writetoeeprom(dac_value);
  		Serial.print("[CURRENT DAC VALUE WRITTEN TO EEPROM] ");
  	}

  	else if(q=="GETDAC\r\n"||q=="GETDAC\r"||q=="GETDAC\n"){//print to the computer what the current value of the DAC in EEPROM is
  		Serial.print("[CURRENT DAC VALUE IN EEPROM: " +String(readfromeeprom())+"] ");
  	}

  	else if(q=="DEFDAC\r\n"||q=="DEFDAC\r"||q=="DEFDAC\n"){//set the dac to whatever is in EEPROM
  		setdac(readfromeeprom());
  	}

  	else if(q.charAt(0)=='D'){//write a new value to the DAC
  		q.setCharAt(0,'0');
  		second_dac=last_dac;
      	last_dac=dac_value;
  		dac_value=q.toInt();
  		setdac(dac_value);
  		delay(1);
  		setdac(dac_value);
  	}

  	else{//if the command wasn't recognized say so
  		Serial.print("[INVALID COMMAND] ");
  	}
  }

  if(lcd_menu!=digitalRead(CYCLE_PIN)){//if the switch was flipped, change the LCD menu and update it now
  	lcd_menu=digitalRead(CYCLE_PIN);
  	updateLCD();
  }
  if(ENFORCE_WARMUP!=digitalRead(WARMUP_BYPASS_PIN)){//if he switch was flipped, change the WARMUP_BYPASS flag and update the LCD now
  	ENFORCE_WARMUP=digitalRead(WARMUP_BYPASS_PIN);
  	updateLCD();
  }
}
