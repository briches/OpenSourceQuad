#include "packet.h"
#include <stdio.h>

packet_t new_empty_packet_t()
{
	packet_t Newpacket_t;
	for (int i = 0; i < PACKET_SIZE; i++)
	{
		Newpacket_t.Buffer[i] = 0x00;
	}
	return Newpacket_t;
}

void printf_packet_t(packet_t MyPacket)
{
	for (int i = 0; i < PACKET_SIZE; i++)
	{
		printf("%c", MyPacket.Buffer[i]);
	}
	printf("%c", '\n');
}

packet_t set_packet_t(packet_t MyPacket, const char *string)
{
	int i = 0;
	while (string[i] != '\0' && i < PACKET_SIZE)
	{
		MyPacket.Buffer[i] = string[i];
		i++;
	}
	while (i < PACKET_SIZE)
	{
		MyPacket.Buffer[i] = 0x00;
		i++;
	}
	return MyPacket;
}
