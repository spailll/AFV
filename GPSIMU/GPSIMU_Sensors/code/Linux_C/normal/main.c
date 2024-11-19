#include "serial.h"
#include "wit_c_sdk.h"
#include "REG.h"
#include <stdint.h>
#include <stdio.h>
#include <unistd.h> // For usleep

#define ACC_UPDATE 0x0001
#define GYRO_UPDATE 0x0002
#define ANGLE_UPDATE 0x0004
#define MAG_UPDATE 0x0008
#define TEMP_UPDATE 0x0010
#define LonL_UPDATE 0x49
#define LonH_UPDATE 0x4a
#define LatL_UPDATE 0x4b
#define LatH_UPDATE 0x4c
#define READ_UPDATE 0x8000

static int fd, s_iCurBaud = 9600;
static volatile uint16_t s_cDataUpdate = 0;

const int c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

static void AutoScanSensor(char *dev);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Please input dev name\n");
        return 0;
    }

    if ((fd = serial_open(argv[1], 9600)) < 0)
    {
        printf("Open %s fail\n", argv[1]);
        return 0;
    }

    float fAcc[3], fGyro[3], fAngle[3], temp;
    int32_t lon, lat;
    int lon_degrees, lat_degrees;
    float lon_minutes, lat_minutes;
    int i;
    char cBuff[1];

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitRegisterCallBack(SensorDataUpdata);

    AutoScanSensor(argv[1]);

    // Print CSV headers
    // printf("AccX,AccY,AccZ,GyroX,GyroY,GyroZ,AngleX,AngleY,AngleZ,MagX,MagY,MagZ,Temp,Longitude,Latitude\n");

    while (1)
    {
        while (serial_read_data(fd, cBuff, 1))
        {
            WitSerialDataIn(cBuff[0]);
        }
        Delayms(500);

        if (s_cDataUpdate)
        {
            if ((s_cDataUpdate & ACC_UPDATE) && (s_cDataUpdate & GYRO_UPDATE) &&
                (s_cDataUpdate & ANGLE_UPDATE) && (s_cDataUpdate & MAG_UPDATE) &&
                (s_cDataUpdate & TEMP_UPDATE) && (s_cDataUpdate & LonL_UPDATE) &&
                (s_cDataUpdate & LonH_UPDATE) && (s_cDataUpdate & LatL_UPDATE) && (s_cDataUpdate & LatH_UPDATE))
            {

                // Process accelerometer data
                for (i = 0; i < 3; i++)
                {
                    fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
                    fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
                    fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
                }

                // Process magnetometer data
                int16_t mag[3];
                for (i = 0; i < 3; i++)
                {
                    mag[i] = sReg[HX + i];
                }

                // Process temperature
                temp = sReg[TEMP] / 100.0f;

                // Process longitude and latitude
                lon = ((int32_t)(uint16_t)sReg[LonH] << 16) | (uint16_t)sReg[LonL];
                lat = ((int32_t)(uint16_t)sReg[LatH] << 16) | (uint16_t)sReg[LatL];

                lon_degrees = lon / 10000000;
                lon_minutes = (lon % 10000000) / 100000.0f;

                lat_degrees = lat / 10000000;
                lat_minutes = (lat % 10000000) / 100000.0f;

                // Print data in CSV format
                printf("%.3f,%.3f,%.3f,", fAcc[0], fAcc[1], fAcc[2]);       // Acceleration
                printf("%.3f,%.3f,%.3f,", fGyro[0], fGyro[1], fGyro[2]);    // Gyroscope
                printf("%.3f,%.3f,%.3f,", fAngle[0], fAngle[1], fAngle[2]); // Angle
                printf("%d,%d,%d,", mag[0], mag[1], mag[2]);                // Magnetometer
                printf("%.2f,", temp);                                      // Temperature
                printf("%.5f, %.5f\n", lon_degrees, lat_degrees);           // GPS.

                // Clear all update flags after printing
                s_cDataUpdate = 0;
            }
        }
    }

    serial_close(fd);
    return 0;
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for (i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
        case AZ:
            s_cDataUpdate |= ACC_UPDATE;
            break;
        case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
        case HZ:
            s_cDataUpdate |= MAG_UPDATE;
            break;
        case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
        case TEMP:
            s_cDataUpdate |= TEMP_UPDATE;
            break;
        case LonL:
            s_cDataUpdate |= LonL;
            break;
        case LonH:
            s_cDataUpdate |= LonH;
            break;
        case LatL:
            s_cDataUpdate |= LatL;
            break;
        case LatH:
            s_cDataUpdate |= LatH;
            break;
        default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
    usleep(ucMs * 1000);
}

static void AutoScanSensor(char *dev)
{
    int i, iRetry;
    char cBuff[1];

    for (i = 1; i < 10; i++)
    {
        serial_close(fd);
        s_iCurBaud = c_uiBaud[i];
        fd = serial_open(dev, c_uiBaud[i]);

        iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            Delayms(200);
            while (serial_read_data(fd, cBuff, 1))
            {
                WitSerialDataIn(cBuff[0]);
            }
            if (s_cDataUpdate != 0)
            {
                return;
            }
            iRetry--;
        } while (iRetry);
    }
    printf("Cannot find sensor\n");
    printf("Please check your connection\n");
}
