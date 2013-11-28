#pragma once

/*
		Open Source Quad: packet.h
	Author:			Brandon Yue
	Last Updated:	November 2013

	Depends:
		packet.h	Defines a packet data structure of 5 bytes.
					Designed so that the preprocessor definition
					PACKET_SIZE is scalable.

	Description:
		This file enables baisic queue functionality using a Linked List.
*/

#define PACKET_SIZE 5

typedef struct packet_t
{
	char Buffer[PACKET_SIZE];
} packet_t;

packet_t new_empty_packet_t();
void printf_packet_t(packet_t MyPacket);
packet_t set_packet_t(packet_t MyPacket, const char *string);