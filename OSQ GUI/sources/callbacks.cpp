#include "callbacks.h"

void callback(		GtkWidget *widget,
					gpointer  data		)
{
	//Generic callback
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void qc_callback_0(	GtkWidget *widget,
					gpointer   data		)
{
	//BUTTON 0
	//Emergency shutdown procedure
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

/*------------------------------------------------------------------------------
	ROW 1																		*/
void qc_callback_1_1(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 1 - 1
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
	if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (widget)))
	{
		//PUSHED
		//Arm the system
		PlaySound(MAKEINTRESOURCE(IDR_WAVE1), GetModuleHandle(NULL), SND_RESOURCE | SND_ASYNC);
	}
	else
	{
		//RELEASED
		//Disarm the system
		PlaySound(MAKEINTRESOURCE(IDR_WAVE2), GetModuleHandle(NULL), SND_RESOURCE | SND_ASYNC);
	}
}

void qc_callback_1_2(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 1 - 2
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
	if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (widget)))
	{
		//PUSHED
	}
	else
	{
		//RELEASED
	}
}

void qc_callback_1_3(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 1 - 3
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
	if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (widget)))
	{
		//PUSHED
	}
	else
	{
		//RELEASED
	}
}

void qc_callback_1_4(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 1 - 4
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
	if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (widget)))
	{
		//PUSHED
	}
	else
	{
		//RELEASED
	}
}

void qc_callback_1_5(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 1 - 5
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
	if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (widget)))
	{
		//PUSHED
	}
	else
	{
		//RELEASED
	}
}

void qc_callback_1_6(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 1 - 6
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
	if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (widget)))
	{
		//PUSHED
	}
	else
	{
		//RELEASED
	}
}

/*------------------------------------------------------------------------------
	ROW 2																		*/
void qc_callback_2_1(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 2 - 1
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void qc_callback_2_2(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 2 - 2
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void qc_callback_2_3(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 2 - 3
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void qc_callback_2_4(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 2 - 4
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void qc_callback_2_5(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 2 - 5
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}

void qc_callback_2_6(	GtkWidget *widget,
						gpointer   data		)
{
	//BUTTON 2 - 6
	g_print("CALLBACK: %s was pressed.\n", (char *) data);
}