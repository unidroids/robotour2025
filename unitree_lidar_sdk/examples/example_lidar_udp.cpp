/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/


#include "example.h"

int main(int argc, char *argv[])
{

    // Získání aktuálního času
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);

    // Vytvoření názvu souboru
    char filename[128];
    strftime(filename, sizeof(filename), "/data/logs/lidar/cloud_%Y-%m-%d_%H-%M-%S.log", t);

    // Přesměrování stdout do souboru
    FILE* fout = freopen(filename, "w", stdout);    

    // Initialize
    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    std::string lidar_ip = "192.168.10.62";
    std::string local_ip = "192.168.10.2";

    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;

    if (lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip))
    {
        printf("Unilidar initialization failed! Exit here!\n");
        exit(-1);
    }
    else
    {
        printf("Unilidar initialization succeed!\n");
    }

    lreader->stopLidarRotation();
    sleep(1);

    // std::string versionFirmware;
    // while (!lreader->getVersionOfLidarFirmware(versionFirmware))
    // {
    //     lreader->runParse();
    // }

    // Set lidar work mode
    uint32_t workMode = 4; //no IMU
    //uint32_t workMode = 0;
    std::cout << "set Lidar work mode to: " << workMode << std::endl;
    lreader->setLidarWorkMode(workMode);
    sleep(1);
    
    // Reset Lidar
    lreader->resetLidar();
    sleep(1);

    // Process
    exampleProcess(lreader);

    lreader->stopLidarRotation();
    

    workMode = 16; //no AutoStart
    std::cout << "set Lidar work mode to: " << workMode << std::endl;
    lreader->setLidarWorkMode(workMode);
    sleep(1);

    std::cout << "end" << std::endl;

    fclose(fout);

    return 0;
}