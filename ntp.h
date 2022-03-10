#ifndef ntp_h
#define ntp_h
#include "Arduino.h"

#include <WiFiNINA.h>
#include <WiFiUdp.h>


const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

class ntp
{
    public:
        ntp(unsigned long localPort);
        void destroy();
        void updateTime();
        void updateTimeFromIP(IPAddress& address);
        byte getHours();
        byte getMinutes();
        byte getSeconds();
        unsigned long getSecsSince1970();
        unsigned long getSecsSince19000();
    private:
        byte hours;
        byte minutes;
        byte seconds;
        unsigned long secsSince1970;
        unsigned long secsSince19000;
        void sendNTPpacket();
        void sendNTPpacketToIP(IPAddress& address);
        void parseResult();
        WiFiUDP Udp;    // A UDP instance to let us send and receive packets over UDP
        unsigned int m_localPort = 2390;      // local port to listen for UDP packets
        IPAddress *timeServer; // time.nist.gov NTP server           (129, 6, 15, 28);
        byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
        
};
#endif 
