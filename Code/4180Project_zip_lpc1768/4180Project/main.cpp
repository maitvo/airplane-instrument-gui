#include "mbed.h"
#include "uLCD_4DGL.h"
#include "LSM9DS1.h"
#include "MPL3115A2.h"
#include "GPS.h"
#include "math.h"
#include "SDFileSystem.h"

int DISP_MODE = 2;
#define M_PI 3.14159265

Ticker velocityTimer;

Serial pc(USBTX, USBRX);
uLCD_4DGL uLCD(p13,p14,p11);
LSM9DS1 lol(p9, p10, 0xD6, 0x3C);
I2C i2c(p9, p10);       // sda, scl
MPL3115A2 sensor(&i2c, &pc);
DigitalOut myled(LED1);     // Sanity check to make sure the program is working.
AnalogIn altSetPot(p20);
DigitalIn screenMode(p21);
DigitalIn rawMode(p22);
DigitalIn flightDir(p23);
Altitude a;
//Temperature t;
GPS gps(p28, p27);
char direction[] = "NE";
float dataOld[4] = {0.0, 0.0, 0.0, 0.0};
float dataNew[4] = {0.0, 0.0, 0.0, 0.0};
int satellites = 0;
float velocity = 0.0;
int R = 6371000;
int count = 0;
SDFileSystem sd(p5, p6, p7, p8, "sd");

double pitch, roll, heading; //positive: pitch up, roll right
double oldPitch, oldRoll;
int speed, setAlt, FDProp, oldFDProp;
       
void initAlti() {
    sensor.init();
    // Offsets for Dacula, GA form example
    sensor.setOffsetAltitude(83);
    sensor.setOffsetTemperature(20);
    sensor.setModeActive();
}

void initDisplay(int mode) {
    uLCD.baudrate(3000000);
    uLCD.printf("Flight Instruments HUD\n");
    uLCD.printf("Booting...");
    if (mode == 3) {
        uLCD.media_init();
        uLCD.set_sector_address(0,0);
        uLCD.set_font(MEDIAFONT);
        uLCD.text_mode(OPAQUE);
        uLCD.color(GREEN);
        uLCD.text_width(2);
        uLCD.text_height(2);
    }
}

void initIMU() {
    lol.begin();
    if (!lol.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    lol.calibrate(1);
    lol.calibrateMag(0);
}

void getAtt() {
    while(!lol.magAvailable(X_AXIS));
    lol.readMag();
    while(!lol.accelAvailable());
    lol.readAccel();
    float ax = lol.calcAccel(lol.ax);
    float ay = lol.calcAccel(lol.ay);
    float az = lol.calcAccel(lol.az);
    float mx = lol.calcMag(lol.mx);
    float my = lol.calcMag(lol.my);
    float mz = lol.calcMag(lol.mz);
    roll = atan2(ay, az);
    pitch = atan2(-ax, sqrt(ay * ay + az * az));
// touchy trig stuff to use arctan to get compass heading (scale is 0..360)
    mx = -mx;
    if (my == 0.0)
        heading = (mx < 0.0) ? 180.0 : 0.0;
    else
        heading = atan2(mx, my)*360.0/(2.0*M_PI);
    if(heading>180.0) heading = heading - 360.0;
    else if(heading<-180.0) heading = 360.0 + heading;
    else if(heading<0.0) heading = 360.0  + heading;
    // Convert everything from radians to degrees:
    //heading *= 180.0 / PI;
    pitch *= 180.0 / M_PI;
    roll  *= 180.0 / M_PI;
}

void initGPS() {
}

void getAlti() {
    sensor.readAltitude(&a);
}

void getData() { //raw test data
    pitch = 15; //positive: pitch up
    roll = 12; //positive: roll right
    speed = 100;
    heading = 270;
}

float getGyroPitch() {    
    return(lol.gx*M_PI/180);
}

float getGyroYaw() {
    return(lol.gy*M_PI/180);
}

float getGyroRoll() {
    return(lol.gz*M_PI/180);
}

void setGPSdata() {
    if(gps.sample()) {
        dataNew[0] = gps.longitude;
        dataNew[1] = gps.latitude;
        dataNew[2] = gps.alt;
        dataNew[3] = gps.time;
        direction[0] = gps.ns;
        direction[1] = gps.ew;
        satellites = gps.num_sat;
    }
}

void calculateVelocity() {
    if (count == 1) {
        count++;
        for (int index = 0; index < 4; index++) {
            dataOld[index] = dataNew[index];
        }
    } else {
        float deltaLong = R * (dataNew[0] - dataOld[0]) * (M_PI/180.0);
        float deltaLat = R * (dataNew[1] - dataOld[1]) * (M_PI/180.0);
        float distance = sqrt(deltaLong*deltaLong + deltaLat*deltaLat);
        velocity = distance / 3.0 * 1.94384; //convert from m/s to knot
        for (int index = 0; index < 4; index++) {
            dataOld[index] = dataNew[index];
        }
    }
    if (DISP_MODE != 2 && velocity <= 3) {velocity = 0;} //trim slight jitter when not moving
}

void dispData(int mode) {//1:normal, 2:raw, 3:HUD
    if (mode == 1) {
    //Normal mode
    //Draw horizon pitch: 2 px per degree
    uLCD.background_color(BLUE);
    uLCD.cls(); //draw sky
    uLCD.text_mode(TRANSPARENT);
    uLCD.color(WHITE);
    int rightHoriz = 64*tan(roll*0.0174533) - 2*pitch;
    int leftHoriz = rightHoriz + 2*2*pitch;
    for (int line = 0; line <= 127; line++) {
        uLCD.line(0,64+leftHoriz+line, 127,64-rightHoriz+line, 0xCD7F32); //draw ground
        //note: not efficient. Always draws 128 lines. Can improve depending on roll (just higher side to end)
    }
    //wings
    if (!flightDir) {
        uLCD.line(54,64+2*pitch-FDProp,74,64+2*pitch-FDProp,0xFF00FF);
        uLCD.line(54,65+2*pitch-FDProp,74,65+2*pitch-FDProp,0xFF00FF);
        uLCD.text_height(1);
        uLCD.text_width(1);
        uLCD.locate(10,13);
        uLCD.printf("%d",setAlt);
    }
    uLCD.line(54,63,74,63,WHITE);
    uLCD.line(54,64,74,64,WHITE);
    uLCD.text_width(2);
    uLCD.text_height(2);
    //speed
    uLCD.locate(0, 7);
    if (satellites < 4) {uLCD.color(RED);uLCD.printf("XXX");uLCD.color(WHITE);}
    else {uLCD.printf("%3.0f",velocity);}
    //alt
    uLCD.locate(5, 7);
    uLCD.printf("%s",a.print()); //"%s",a.print() for actual altitude, "%d",alt for sim
    //heading
    uLCD.locate(3,0);
    uLCD.printf("%.0f ",heading);
    } else if (mode == 2) {
    //Raw data mode:
    uLCD.locate(0,0);
    uLCD.printf("Raw Data:         ");
    uLCD.printf("Pitch: %2.1f\n",pitch);
    uLCD.printf("Roll: %2.1f\n",roll);
    uLCD.printf("Alt: %s\n",a.print());//alt);
    uLCD.printf("Speed: %f\n",velocity);
    uLCD.printf("Heading: %3.4f  \n",heading);
    uLCD.printf("Lat: %f\n",dataNew[1]);
    uLCD.printf("Long: %f\n",dataNew[0]);
    uLCD.printf("Sats: %d\n",satellites);
    uLCD.printf("AltSet: %d\n",setAlt);
    uLCD.printf("FDProp: %d \n",FDProp);
    wait(1);
    } else { //HUD display mode: everything is inverted
    uLCD.background_color(BLACK);
    //uLCD.cls(); //draw sky
    int rightHoriz = 64*tan(oldRoll*0.0174533) - 2*oldPitch;
    int leftHoriz = rightHoriz + 2*2*oldPitch;
    uLCD.line(0,63-leftHoriz, 127,63+rightHoriz, BLACK); //erase previous data
    uLCD.line(0,64-leftHoriz, 127,64+rightHoriz, BLACK);
    
    rightHoriz = 64*tan(roll*0.0174533) - 2*pitch;
    leftHoriz = rightHoriz + 2*2*pitch;
    uLCD.line(0,63-leftHoriz, 127,63+rightHoriz, GREEN); //draw 2 || lines for very thick
    uLCD.line(0,64-leftHoriz, 127,64+rightHoriz, GREEN);
    if (!flightDir) {
        uLCD.line(54,64-2*oldPitch+oldFDProp,74,64-2*oldPitch+oldFDProp,BLACK);
        uLCD.line(54,65-2*oldPitch+oldFDProp,74,65-2*oldPitch+oldFDProp,BLACK);
        uLCD.line(54,64-2*pitch+FDProp,74,64-2*pitch+FDProp,0xFF00FF);
        uLCD.line(54,65-2*pitch+FDProp,74,65-2*pitch+FDProp,0xFF00FF);
        uLCD.text_width(1);
        uLCD.text_height(1);
        uLCD.locate(6,2);
        uLCD.printf("%d",setAlt);
        uLCD.text_width(2);
        uLCD.text_height(2);
    }
    oldPitch = pitch;
    oldRoll = roll;
    oldFDProp = FDProp;
    //boresight
    uLCD.line(55,63,58,63,GREEN);// -
    uLCD.line(58,63,61,60,GREEN);// \
    uLCD.line(61,60,64,63,GREEN);// /
    uLCD.line(64,63,67,60,GREEN);// \
    uLCD.line(67,60,70,63,GREEN);// /
    uLCD.line(70,63,74,63,GREEN);// -
    uLCD.line(55,64,58,64,GREEN);
    uLCD.line(58,64,61,61,GREEN);
    uLCD.line(61,61,64,64,GREEN);
    uLCD.line(64,64,67,61,GREEN);
    uLCD.line(67,61,70,64,GREEN);
    uLCD.line(70,64,74,64,GREEN);
    uLCD.locate(0,0);
    uLCD.locate(0,0);
    if (satellites < 4) {uLCD.color(RED);uLCD.printf("XXX");uLCD.color(GREEN);}
    else {uLCD.printf("%3.0f",velocity);}
    uLCD.locate(3,0);
    uLCD.printf("%s ",a.print());//%d",alt);
    uLCD.locate(2,5);
    uLCD.printf("%3.0f ",heading);
    
    }
}

void switchScreen() {
    if ((DISP_MODE == 1 || DISP_MODE == 3) && !rawMode) { //switch to raw mode
        uLCD.background_color(BLACK);
        uLCD.cls();
        uLCD.text_width(1);
        uLCD.text_height(1);
        uLCD.color(GREEN);
        uLCD.set_font(FONT_7X8);
        DISP_MODE = 2;
    } else if ((DISP_MODE == 1 || DISP_MODE == 2) && !screenMode && rawMode) { //switch to HUD mode
        uLCD.background_color(BLACK);
        uLCD.cls();
        uLCD.media_init();
        uLCD.set_sector_address(0,0);
        uLCD.set_font(MEDIAFONT);
        uLCD.text_mode(OPAQUE);
        uLCD.color(GREEN);
        uLCD.text_width(2);
        uLCD.text_height(2);
        DISP_MODE = 3;
    } else if ((DISP_MODE == 2 || DISP_MODE == 3) && screenMode && rawMode) { //switch to normal mode
        uLCD.background_color(BLUE);
        uLCD.cls();
        uLCD.set_font(FONT_7X8);
        DISP_MODE = 1;
    }
        
}
    
int main() {
    //Init
    initDisplay(DISP_MODE);
    initIMU();
    initAlti();
    velocityTimer.attach(&calculateVelocity, 3.0);
    FILE *fp = fopen("/sd/mydir/flightData.csv", "w");
    fprintf(fp, "time,lat,long,gpsAlt,speed,baroAlt,pitch,roll,heading\n");
    fclose(fp);
    screenMode.mode(PullUp);
    rawMode.mode(PullUp);
    flightDir.mode(PullUp);
    //getData();
    while(1) {
        switchScreen();
        getAlti();
        getAtt();
        setGPSdata();
        setAlt = altSetPot * 8000;
        setAlt = setAlt <= 1000 ? 1000 : setAlt;
        setAlt = setAlt % 100 >= 50 ? setAlt + 100 - setAlt % 100 : setAlt - setAlt % 100;
        if (!flightDir) {
            FDProp = setAlt - atoi(a.print());
            FDProp = FDProp > 1000 ? 1000 : FDProp;
            FDProp = FDProp < -1000 ? -1000 : FDProp;
            FDProp = FDProp / 34; //number of pixels for FD wings
        }            
        dispData(DISP_MODE);
        myled = !myled;
        if (DISP_MODE == 1) {wait(.5);} //give time to refresh and read display
        speed++;
        FILE *fp = fopen("/sd/mydir/flightData.csv", "a");
        fprintf(fp, "%f,%f,%f,%f,%f,%s,%f,%f,%f\n",dataNew[3],dataNew[1],dataNew[0],dataNew[2],velocity,a.print(),pitch,roll,heading);
        fclose(fp);
    }
}