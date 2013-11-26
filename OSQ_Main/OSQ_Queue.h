/*=====================================================================
	OSQ_Queue library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Andrew Coulthard
	Date		: November 2013
	License		: GNU Public License

	This header provides a MAXIMUM_SIZE byte circular queue to store incoming 
	data. The boolean returns from the put/get functions can be used to 
	determine if data was stored successfully.

	Copyright (C) 2013  Andrew Coulthard

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	-----------------------------------------------------------------------------*/

#ifndef OSQ_QUEUE_H_INCLUDED
#define OSQ_QUEUE_H_INCLUDED

#define MINIMUM_SIZE 1

typedef struct Queue_t
{
		boolean err;
	
		uint8_t  size,
                 head,
                 tail,
				 maxsize;
        
        uint8_t * memory;
        
		Queue_t(uint8_t max, uint8_t min); // Constructor

		~Queue_t(); // Destructor

		boolean qPutData(uint8_t data);
		boolean qGetData(uint8_t &data);

};


/* Constructor, initializes queue memory array and initial values for
queue variables. Will set err == 1 if memory is unable to be allocated.
*/
Queue_t :: Queue_t(uint8_t max) 
{

	size = 0;
	head = 0;
	tail = 0;
	err = 0;
	maxsize = max;

	memory = (uint8_t *)malloc(sizeof(uint8_t)*maxsize);
	if(!memory) {
		err = 1;
	}
		
	
};

/* Destructor, frees queue memory array.
*/
Queue_t :: ~Queue_t()
{
	free(memory);
};

/* Function takes byte input of data, stores to queue array if space
is available. Will return true/false based on memory availability.
*/
boolean qPutData(uint8_t data) 
{

	if (size >= maxsize) 
	{
		return false;
	}
	else
	{
		*(memory + head) = data;
		head = (head + 1) % maxsize
		size++;
		return true;
	}

};
/* Function takes input as location of byte of data, pulls data from
the queue if it is available. Will return true/false based on memory
availability.
*/
boolean qGetData(uint8_t &data) 
{

	if (size < MINIMUM_SIZE)
	{
		return false;
	}
	else
	{
		data = *(memory + tail);
		tail = (tail + 1) % maxsize
		size--;
		return true;
	}

};

#endif // OSQ_QUEUE_H_INCLUDED
