#pragma once
#include "stdafx.h"

/*--------|---------|---------|---------|---------|---------|---------|---------
Callbacks for controlling QC movement. These are connected / disconnected to the
key controls by qc_callback_control_scheme.
|---------|---------|---------|---------|---------|---------|---------|-------*/
void qc_callback_control_scheme(
					GtkWidget *widget,
					gpointer   data		);
void moveForeward(	GtkWidget *widget,
					gpointer   data		);
void moveBack(		GtkWidget *widget,
					gpointer   data		);
void strafeLeft(	GtkWidget *widget,
					gpointer   data		);
void strafeRight(	GtkWidget *widget,
					gpointer   data		);
void rotateLeft(	GtkWidget *widget,
					gpointer   data		);
void rotateRight(	GtkWidget *widget,
					gpointer   data		);
void altAscend(		GtkWidget *widget,
					gpointer   data		);
void altDescend(	GtkWidget *widget,
					gpointer   data		);
void qc_release(	GtkWidget *widget,
					gpointer   data		);