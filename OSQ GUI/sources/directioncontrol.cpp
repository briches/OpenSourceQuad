#include "directioncontrol.h"
#include "stdafx.h"

void qc_callback_control_scheme(
					GtkWidget *widget,
					gpointer   data		)
{	
	char *myString = gtk_combo_box_get_active_text(GTK_COMBO_BOX (widget));
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
	if (myString[3] == 'S')
	{
		g_print("\t%s was selected.\n", myString);
		/*	Default control scheme.
		 *	UP		W	|	Foreward
		 *	DOWN	S	|	Back
		 *	LEFT	A	|	Strafe Left
		 *	RIGHT	D	|	Strafe Right
		 *	INSERT	Q	|	Rotate Left
		 *	HOME	E	|	Rotate Right
		 *	PGUP	X	|	Ascend
		 *	PGDN	Z	|	Descend
		 */
	}
	else if (myString[3] == 'R')
	{
		g_print("\t%s was selected.\n", myString);
		/*	Alternate control scheme.
		 *	UP		W	|	Foreward
		 *	DOWN	S	|	Back
		 *	LEFT	A	|	* Rotate Left
		 *	RIGHT	D	|	* Rotate Right
		 *	INSERT	Q	|	* Strafe Left
		 *	HOME	E	|	* Strafe Right
		 *	PGUP	X	|	Ascend
		 *	PGDN	Z	|	Descend
		 */
	}
	else if (myString[3] == 'o')
	{
		g_print("\t%s was selected.\n", myString);
		/*	FPS control scheme.
		 *	UP		W	|	Foreward
		 *	DOWN	S	|	Back
		 *	LEFT	A	|	Strafe Left
		 *	RIGHT	D	|	Strafe Right
		 *	INSERT	Q	|	Rotate Left
		 *	HOME	E	|	Rotate Right
		 *	PGUP	SPC	|	Ascend
		 *	PGDN	SHFT|	Descend
		 */
	}
}

void moveForeward(	GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void moveBack(		GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void strafeLeft(	GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void strafeRight(	GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void rotateLeft(	GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void rotateRight(	GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void altAscend(		GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void altDescend(	GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void qc_release(	GtkWidget *widget,
					gpointer   data		)
{
	g_print("CALLBACK: %s was released.\n", (char *) data);
}
