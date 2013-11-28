#pragma once
#include "packet.h"
#include "stdafx.h"

/*
		Open Source Quad: OSQ_Queue.cpp
	Author:			Brandon Yue
	Last Updated:	November 2013

	Depends:
		packet.h	Defines a packet data structure of 5 bytes.
					Designed so that the preprocessor definition
					PACKET_SIZE is scalable.

	Description:
		This file enables baisic queue functionality using a Linked List.
*/

typedef struct osq_node_t
{
	packet_t Data;
	osq_node_t *Next;
} osq_node_t;

class OSQ_Queue
{
public:
	OSQ_Queue();
	~OSQ_Queue();
	bool push(packet_t Incoming);
	packet_t pull();
	void cleanup();
	void printqueue();

private:
	osq_node_t *Head;
	osq_node_t *Tail;
	unsigned int length;
};

void test_OSQ_Queue();
