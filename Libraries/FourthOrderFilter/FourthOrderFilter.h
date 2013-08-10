/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _AQ_FOURTH_ORDER_FILTER_H_
#define _AQ_FOURTH_ORDER_FILTER_H_

////////////////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

#include <GlobalDefined.h>

#define XAXIS   (0)
#define YAXIS	(1)
#define ZAXIS	(2)



float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters);

void setupFourthOrder(void);

////////////////////////////////////////////////////////////////////////////////

#endif
