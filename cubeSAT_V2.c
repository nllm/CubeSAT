//CubeSAT Version 2 - integrates a gyroscope, can be used with a LabVIEW interface

//Librerías
#include "mbed.h"
#include "BMP180.h"
#include "Si7021.h"
#include "TSL2561.h"
#include "MPU9250.h"
#include "Arial12x12.h"
#include "Arial24x23.h"
#include "Arial28x28.h"
#include "font_big.h"
#include "ST7735_TFT.h"

//Declaración de pines de la mbed
//I2C myI2C(p9,p10);
Serial pc(USBTX,USBRX);
Serial xbee(p28, p27);
BMP180 bmp180(p9,p10);
Si7021 sensor(p9,p10);
TSL2561 tsl2561(p9,p10);
I2C DFG(p9, p10);
MPU9250 mpu9250;
ST7735_TFT TFT(p5, p6, p7, p12, p11, p15,"TFT"); // mosi, miso, sclk, cs, rs, reset 

//Declaración de variables
int presion;
float temperatura;
int sensor_addr = 41 << 1;
float sum = 0;
uint32_t sumCount = 0;
char buffer[14];
Timer t;
char flag, g;
int i=0;
int pin[8];

//Inicio del programa principal
int main(){
    xbee.baud(115200);
    //Pantalla
    TFT.set_orientation(1);
    TFT.claim(stdout);      // send stdout to the TFT display 
    TFT.background(Black);    // set background to black
    TFT.foreground(Green);    // set chars to white
    TFT.cls();
    TFT.set_font((unsigned char*) Arial12x12);  // select the font
    
    if(g!='1'){
            flag=xbee.getc();
            g='1';
            xbee.printf("2");
            g=xbee.getc();}
    if(g=='2'){
            for(i=0;i<7;i++){
            xbee.printf("1");
            flag=xbee.getc();
            if(flag=='1'){
                pin[i]=1;
                }
    else{
            pin[i]=0;
    }}
            
    //Giroscopio
    if(pin[6]==1){
    t.start();         
    uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    wait(1);
    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    wait(2);
    mpu9250.initMPU9250();
    wait(1); 
    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity|
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    }}
         
    while(1){       
        //Adquisición de datos
        
            //Presión
            bmp180.init();
            bmp180.startPressure(BMP180::ULTRA_LOW_POWER);
            wait_ms(10);
            bmp180.getPressure(&presion);
            float altitud= 44330.0*(1-pow(presion*.01/1020.0,(1/5.225)));
            
            //Humedad
            sensor.measure();
            float temperatura=sensor.get_temperature()/1000.0;
            float humedad=sensor.get_humidity()/1000.0;
            
            //Luz
            tsl2561.begin();
            tsl2561.setGain(TSL2561_GAIN_0X);
            tsl2561.setTiming(TSL2561_INTEGRATIONTIME_402MS);  
            uint16_t x,y,z;  
            x = tsl2561.getLuminosity(TSL2561_VISIBLE);     
            y = tsl2561.getLuminosity(TSL2561_FULLSPECTRUM);
            z = tsl2561.getLuminosity(TSL2561_INFRARED);
            
            //RGB
            char id_regval[1] = {146};
            char data[1] = {0};
            DFG.write(sensor_addr,id_regval,1, true);
            DFG.read(sensor_addr,data,1,false);
            char timing_register[2] = {129,0};
            DFG.write(sensor_addr,timing_register,2,false);
            char control_register[2] = {143,0};
            DFG.write(sensor_addr,control_register,2,false);
            char enable_register[2] = {128,3};
            DFG.write(sensor_addr,enable_register,2,false);
            char clear_reg[1] = {148};
            char clear_data[2] = {0,0};
            DFG.write(sensor_addr,clear_reg,1, true);
            DFG.read(sensor_addr,clear_data,2, false);
            int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
            char red_reg[1] = {150};
            char red_data[2] = {0,0};
            DFG.write(sensor_addr,red_reg,1, true);
            DFG.read(sensor_addr,red_data,2, false);
            int red_value = ((int)red_data[1] << 8) | red_data[0];
            char green_reg[1] = {152};
            char green_data[2] = {0,0};
            DFG.write(sensor_addr,green_reg,1, true);
            DFG.read(sensor_addr,green_data,2, false);
            int green_value = ((int)green_data[1] << 8) | green_data[0];
            char blue_reg[1] = {154};
            char blue_data[2] = {0,0};
            DFG.write(sensor_addr,blue_reg,1, true);
            DFG.read(sensor_addr,blue_data,2, false);
            int blue_value = ((int)blue_data[1] << 8) | blue_data[0];
            
            //Giroscopio
            if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
            ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
            ay = (float)accelCount[1]*aRes - accelBias[1];   
            az = (float)accelCount[2]*aRes - accelBias[2];  
            mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
            gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
            gy = (float)gyroCount[1]*gRes - gyroBias[1];  
            gz = (float)gyroCount[2]*gRes - gyroBias[2];   
            mpu9250.readMagData(magCount);}
            Now = t.read_us();
            deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
            lastUpdate = Now;
            sum += deltat;
            sumCount++;
            mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
            delt_t = t.read_ms() - count;
            if (delt_t > 500) { // update LCD once per half-second independent of read rate
            yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
            pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
            roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            pitch *= 180.0f / PI;
            yaw   *= 180.0f / PI; 
            yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
            roll  *= 180.0f / PI;
            count = t.read_ms(); 
            sum = 0;
            sumCount = 0; }
            
        //Impresión de datos
        g=xbee.getc();
        //Monitor serial
            //Presión
            if(pin[3]==1){
            pc.printf("P:%d Pa\n\rA:%.2f m\n\r", presion, altitud);}
            //Humedad
            if(pin[2]==1){
            pc.printf("T:%0.2f C\n\rH:%0.2f\n\r",temperatura, humedad);}
            //Luz
            if(pin[5]==1){
            pc.printf("Visible:%d\n\rFull:%d\n\rIR:%d\n\r",x,y,z);}
            //RGB
            if(pin[4]==1){
            pc.printf("Clear (%d), Red (%d), Green (%d), Blue (%d)\n\r", clear_value, red_value, green_value, blue_value);}
            //Giroscopio
            if(pin[6]==1){
            pc.printf("%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%3.1f,%3.1f,%3.1f\r\n",ax,ay,az,gx,gy,gz,yaw,pitch,roll);}
            pc.printf("\r\n");
        
        //Pantalla
            //Presión
            if(pin[3]==1){
            printf("P:%d Pa\n\rA:%.2f m\n\r", presion, altitud);}
            //Humedad
            if(pin[2]==1){
            printf("T:%0.2f C\n\rH:%0.2f\n\r",temperatura, humedad);}
            //Luz
            if(pin[5]==1){
            printf("Visible:%d\n\rFull:%d\n\rIR:%d\n\r",x,y,z);}
            //RGB
            if(pin[4]==1){
            printf("Clear (%d), Red (%d), Green (%d), Blue (%d)\n\r", clear_value, red_value, green_value, blue_value);}
            //Giroscopio
            if(pin[6]==1){
            printf("%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%3.1f,%3.1f,%3.1f\r\n",ax,ay,az,gx,gy,gz,yaw,pitch,roll);}
            printf("\r\n");
         
        //Xbee y bluetooth
            xbee.printf("# ");
            //Presión
            if(pin[3]==1){
            xbee.printf("%d %.2f ", presion, altitud);}
            //Humedad
            if(pin[2]==1){
            xbee.printf("%0.2f %0.2f ",temperatura, humedad);}
            //Luz
            if(pin[5]==1){
            xbee.printf("%d %d %d ",x,y,z);}
            //RGB
            if(pin[4]==1){
            xbee.printf("%d %d %d %d ", clear_value, red_value, green_value, blue_value);}
            //Giroscopio
            if(pin[6]==1){
            xbee.printf("%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %3.1f %3.1f %3.1f ",ax,ay,az,gx,gy,gz,yaw,pitch,roll);}
            xbee.printf("&");
            wait(1);
        
}}
