//This program includes the first version of the CanSAT Project, it includes multiple sensors and the ability to store and send the data
#include "mbed.h"
#include "Adafruit_SSD1306.h"
#include "DS1307.h"
#include "SHTx/sht15.hpp"
#include "L3GD20.h"
#include "LSM303DLHC.h"
#include "BMP180.h"
#include "MS5803.h"
#include "SDFileSystem.h"
#include "FastJpegCamera.h"
#include<stdio.h>
#include<string.h> 
#include<iostream>
#include<fstream>
#include<errno.h>

//Puertos
DigitalOut LEDs[4] = {    DigitalOut(LED4), DigitalOut(LED3), DigitalOut(LED2), DigitalOut(LED1)};
I2C myI2C(p9,p10);
Adafruit_SSD1306_I2c o(myI2C,D13,0x7A,64,128);
//Serial pc(USBTX,USBRX);
Serial xbee(p28,p27);
RtcDs1307 gRtc ( myI2C );
DateTime dt;
SHTx::SHT15 sensor(p22, p21);
L3GD20 gyro(p9, p10);
LSM303DLHC compass(p9, p10);
BMP180 bmp180(&myI2C);
AnalogIn uv(p20);
AnalogIn Vol(p19);
MS5803     press_sensor( p9, p10, 0x76);
SDFileSystem sd(p5, p6, p7, p8, "sd");
Timer timer;
InterruptIn geiger(p29);


//variables
float ax, ay, az;
float mx, my, mz;
float gx, gy, gz;
double yaw,pitch,roll;
int press;
float temp;
float uv_rad=0;
using namespace std;
int m=0;
int loop_count;
float amperaje;
float voltaje;
char file[32];
char send_buffer[15000];
char final[20000];
int cont=0;
#define     M_PI   3.14159265358979323846
#define archivof "/sd/fotos/pic%05d.jpg"
#define archivod "/sd/datos/dato%05d.txt"

void contador()
{
    cont++;
}

int main() {
    wait(3);
    
    o.clearDisplay();
    o.setTextCursor(0,0);
    //pc.printf("CANSAT v.2 UPAEP\n\r");
    
    o.printf("CANSAT v.2 UPAEP\n\r");
    o.display();
    geiger.mode(PullUp);
    geiger.rise(&contador);

    wait(1);
    LEDs[0]=!LEDs[0];
    
    mkdir("/sd/fotos", 0777);     
    mkdir("/sd/datos", 0777);
    sensor.setOTPReload(false);
    sensor.setResolution(true);
    timer.start();
    xbee.baud(115200);
    FastJpegCamera fast_jpeg_camera(p13, p14, xbee);
          o.clearDisplay();
    o.setTextCursor(0,0);
    o.printf("Tomara la foto, la foto debe tardar aproximadamente 4 segundos, si tarda es que esta mal  la camara");
    o.display();
  
     while(1) 
    {
    wait(2);
    xbee.baud(115200);
           loop_count += 1;
       //h=0; 
       //t=1;
       //pc.printf("%f\n\r",timer.read());
       timer.reset();
       fast_jpeg_camera.shoot(loop_count);
       FILE *picture;

       int size, read_size, packet_index;
       char send_buffer[10240];
       packet_index = 1;
       char file[32];
       sprintf(file, archivof, loop_count); 
       picture = fopen(file, "r");
       //printf("Getting Picture Size\n");   
    
    
        fseek(picture, 0, SEEK_END);
        size = ftell(picture);
        fseek(picture, 0, SEEK_SET);
        //printf("Total Picture size: %i\n",size);
  //      xbee.printf("Total Picture size: %i\n",size);
        
          while(!feof(picture)) {
   //while(packet_index = 1){
      //Read from the file into our send buffer
      read_size = fread(send_buffer, 1, sizeof(send_buffer)-1, picture);

      //Send data through our socket 

      packet_index++;  
      
     }
     
     fclose(picture);
     o.clearDisplay();
     o.setTextCursor(0,0);    
     o.printf("Foto %d\n\r",loop_count );
     o.display();
        o.printf("Datos \n\r");
        fclose(picture);
        LEDs[1]=!LEDs[1];
        bmp180.init();
        compass.read(&ax, &ay, &az, &mx, &my, &mz);
        gyro.read(&gx, &gy, &gz);
        roll=atan2(ay, az);
        
        pitch=atan((-ax)/(ay*sin(roll)+az*cos(roll)));
        yaw=atan2(mz*sin(roll)-my*cos(roll),mx*cos(pitch)+my*sin(pitch)*sin(roll)+mz*sin(pitch)*cos(roll));
        press_sensor.Barometer_MS5803();
        bmp180.startTemperature();
        wait_ms(5);     // Wait for conversion to complete
        if(bmp180.getTemperature(&temp) != 0) {o.printf("Error getting temperature\n");}
        bmp180.startPressure(BMP180::ULTRA_LOW_POWER);
        wait_ms(10);    // Wait for conversion to complete
        if(bmp180.getPressure(&press) != 0) {o.printf("Error getting pressure\n");}
        xbee.baud(9600);    
        uv_rad=((uv.read()*3.3)-1)/0.125;
        if(uv_rad<0)uv_rad=0;        
        sensor.update();
        dt = gRtc.now();
    
        voltaje=(Vol.read()*3.3)/(.2423);
  
    
        o.setTextCursor(0,0);
        o.clearDisplay();

        char fil[32];
        sprintf(fil, archivod, loop_count); 
        FILE *fp = fopen(fil, "w");
   
    
        o.printf("%u/%u/%02u %2u:%02u:%02u\n\r",dt.month(),dt.day(),dt.year(),(dt.hour()),dt.minute(),dt.second());
        o.printf("Temperatura: %3.2f C\r\n", sensor.getTemperature());
        o.printf("Humedad: %3.2f %%\r\n", sensor.getHumidity());
        o.printf("P=%d Pa\n\rT=%.2fC\n\r", press, temp);
        o.printf("r:%.1fp:%.1fy:%.1f\n\r",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
        o.printf("UV=%.3fmW/cm^2\n\r",uv_rad);
        o.printf("P=%.0fPa T=%.2fC\r\n", press_sensor.MS5803_Pressure()*100,press_sensor.MS5803_Temperature());
        o.printf("Vol:%f V\n\r",voltaje);
        o.printf("Geiger: %d en tiempo:%f\n\r",cont,timer.read());

        fprintf(fp,"%u/%u/%02u %2u:%02u:%02u\n\r",dt.month(),dt.day(),dt.year(),(dt.hour()),dt.minute(),dt.second());
        fprintf(fp,"Temperatura: %3.2f C\r\n", sensor.getTemperature());
        fprintf(fp,"Humedad: %3.2f %%\r\n", sensor.getHumidity());
        fprintf(fp,"P=%d Pa\n\rT=%.2fC\n\r", press, temp);
        fprintf(fp,"r:%.1fp:%.1fy:%.1f\n\r",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
        fprintf(fp,"UV=%.3fmW/cm^2\n\r",uv_rad);
        fprintf(fp,"P=%.0fPa T=%.2fC\r\n", press_sensor.MS5803_Pressure()*100,press_sensor.MS5803_Temperature());
        fprintf(fp,"Vol:%f V\n\r",voltaje);
        fprintf(fp,"Geiger: %d en tiempo:%f\n\r",cont,timer.read());
        
        cont=0; 
        timer.reset();
        fclose(fp);
        m++;
    
        o.fillCircle(125,61,2,1);
        o.drawCircle(118,61,2,1);
        o.fillCircle(111,61,2,1);
        o.drawCircle(104,61,2,1);
        o.fillCircle(97,61,2,1);
        o.display();
        
        //Envio
        
        xbee.printf("INI%2u,%2u,%2u,",(dt.hour()),dt.minute(),dt.second());
        xbee.printf("%.3f,",uv_rad);
        xbee.printf("%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,2.2f,%3.1f,%3.1f,%3.1f,",ax,ay,az,gx,gy,gz,yaw,pitch,roll);
        xbee.printf("%2.2f,%7d,",temp,press);  
        xbee.printf("%2.2f,%3.1f,", sensor.getTemperature(),sensor.getHumidity());
        xbee.printf("%7.0f,%2.2f,", press_sensor.MS5803_Pressure()*100,press_sensor.MS5803_Temperature());
        xbee.printf("%1.1f,",voltaje);
        xbee.printf("%5u,",cont);
        xbee.printf("%7i,",size);
        m=0;
        while(m<read_size)
      {
          xbee.printf("%02hx",send_buffer[m]);
          m++;
          
        }
        xbee.printf("FIN");
        m=0;
        
        strcpy(send_buffer, "");
        
    }
   
}
