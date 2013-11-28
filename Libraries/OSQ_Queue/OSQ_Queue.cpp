#include "OSQ_Queue.h"

OSQ_Queue::OSQ_Queue()
{
	Head = NULL;
	Tail = NULL;
	length = 0;
}

OSQ_Queue::~OSQ_Queue()
{
	cleanup();
}

bool OSQ_Queue::push(packet_t Incoming)
{
	osq_node_t *NewNode = (osq_node_t *) calloc(1, sizeof(osq_node_t));
	if (!NewNode)
	{
		return false;
	}

	NewNode -> Data = Incoming;
	NewNode -> Next = NULL;

	if (length == 0)
	{
		Head = NewNode;
		Tail = NewNode;
	}
	else
	{
		Tail -> Next = NewNode;
		Tail = NewNode;
	}

	length ++;
	return true;
}

packet_t OSQ_Queue::pull()
{
	if (length == 0)
	{
		// If there's nothing in the queue
		// Return an empty packet_t.
		packet_t Emptypacket_t;
		for (int i = 0; i < PACKET_SIZE; i++)
		{
			Emptypacket_t.Buffer[i] = 0x00;
		}
		return Emptypacket_t;
	}
	else
	{
		// Otherwise return the next packet_t
		packet_t Result = Head -> Data;
		osq_node_t *temp = Head;
		if (length == 1)
		{
			// If there's only one element
			// Set the head and tail to NULL.
			Head = NULL;
			Tail = NULL;
		}
		else
		{
			// If there's more than one element
			// Set the head to the next element.
			Head = Head -> Next;
		}
		// Always cleanup.
		free(temp);
		length --;
		return Result;
	}
}

void OSQ_Queue::cleanup()
{
	while (length > 0)
	{
		pull();
	}
}

void OSQ_Queue::printqueue()
{
	osq_node_t *AtNode = Head;
	while (AtNode)
	{
		printf_packet_t(AtNode -> Data);
		AtNode = AtNode -> Next;
	}
	printf("-----\n");
}

void test_OSQ_Queue()
{
	printf("Starting OSQ Queue test!\n");
	OSQ_Queue MyQueue;
	packet_t MyPacket = { {} };

	MyPacket = set_packet_t(MyPacket, "Hello");
	MyQueue.push(MyPacket);
	
	MyPacket = set_packet_t(MyPacket, "world");
	MyQueue.push(MyPacket);
		
	MyPacket = set_packet_t(MyPacket, "qwert");
	MyQueue.push(MyPacket);
		
	MyPacket = set_packet_t(MyPacket, "yuiop");
	MyQueue.push(MyPacket);

	MyQueue.printqueue();

	MyQueue.pull();
	MyQueue.pull();
	MyQueue.pull();

	MyQueue.printqueue();

	MyPacket = set_packet_t(MyPacket, "Hello");
	MyQueue.push(MyPacket);

	MyPacket = set_packet_t(MyPacket, "again");
	MyQueue.push(MyPacket);
	
	MyPacket = set_packet_t(MyPacket, "fucko");
	MyQueue.push(MyPacket);

	MyQueue.printqueue();

	MyQueue.cleanup();

	MyQueue.printqueue();

	printf("End of test!\n");
}

