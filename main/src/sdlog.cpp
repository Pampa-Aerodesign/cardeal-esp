#include "sdlog.hpp"
#include <stdio.h>

void logWriteHeader(FILE** file){
  fprintf(file, "PacketID,Timestamp,Baro,Temp\n");
}

void logWrite(FILE** file, DataPacket* sdpacket){
  fprintf(file, "%d,", sdpacket->packetid);
  fprintf(file, "%d,", sdpacket->timestamp);
  fprintf(file, "%d,", sdpacket->baro);
  fprintf(file, "%d\n", sdpacket->temp);
}
