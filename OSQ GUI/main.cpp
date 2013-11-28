#include "sources/stdafx.h"

gint delete_event(	GtkWidget	*widget,
					GdkEvent	*event,
					gpointer	data	)
{
	gtk_main_quit();
	return false;
}

int main(int argc, char *argv[])
{
	/*	Initialize the GTK Widgets	*/
	GtkWidget *mainWindow; GtkWidget *button; 
	GtkWidget *boxV; GtkWidget *boxH; GtkWidget *boxSub0;
	GtkWidget *controlsTable;
	GtkWidget *comboBox;
	//GtkWidget *bankValues; GtkWidget *xTilt; GtkWidget yTilt;
	GtkWidget *separator; GtkWidget *arrow;

	/*	GTK init	*/
	gtk_init(&argc, &argv);

	/*	Create the widgets	*/
	/*	->	Main Window	*/
		mainWindow	= gtk_window_new(GTK_WINDOW_TOPLEVEL);
		g_signal_connect(	G_OBJECT (mainWindow),
							"delete_event",
							G_CALLBACK (delete_event),
							NULL	);
		gtk_window_set_title			(GTK_WINDOW		(mainWindow), "QCGUI: Base Station");
		gtk_container_set_border_width	(GTK_CONTAINER	(mainWindow), 5);
		//gtk_window_set_resizable		(GTK_WINDOW		(mainWindow), false);

	/*	->	boxV	*/
		boxV = gtk_vbox_new(false, 5);
		gtk_container_add(GTK_CONTAINER (mainWindow), boxV);

	/*	->	Row 1:	*/
	/*		->	Button 1: EMERGENCY	*/
			button = gtk_button_new_with_label("EMERGENCY");
			g_signal_connect(	G_OBJECT (button),
								"clicked",
								G_CALLBACK (qc_callback_0),
								(gpointer) "Button 0");
			gtk_box_pack_start(GTK_BOX (boxV), GTK_WIDGET (button), true, true, 0);
			gtk_widget_show(button);

		separator = gtk_hseparator_new();
		gtk_box_pack_start(GTK_BOX (boxV), GTK_WIDGET (separator), true, true, 0);
		gtk_widget_show(separator);

	/*	->	Row 2	*/
		boxH = gtk_hbox_new(true, 0);
	/*		->	boxH	*/
			gtk_box_pack_start(GTK_BOX (boxV), GTK_WIDGET (boxH), true, true, 0);
	/*			->	Button 1: ARM	*/
				button = gtk_toggle_button_new_with_label("\nARM/DISARM\n");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_1_1),
									(gpointer) "Button 1 - 1");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 2:	*/
				button = gtk_toggle_button_new_with_label("LAND");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_1_2),
									(gpointer) "Button 1 - 2");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 3:	*/
				button = gtk_toggle_button_new_with_label("STREAM");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_1_3),
									(gpointer) "Button 1 - 3");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 4:	*/
				button = gtk_toggle_button_new_with_label("ALARMS");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_1_4),
									(gpointer) "Button 1 - 4");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 5:	*/
				button = gtk_toggle_button_new_with_label("CTRL POLICY");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_1_5),
									(gpointer) "Button 1 - 5");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 6:	*/
				button = gtk_toggle_button_new_with_label("FREE");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_1_6),
									(gpointer) "Button 1 - 6");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
		gtk_widget_show(boxH);
		
		separator = gtk_hseparator_new();
		gtk_box_pack_start(GTK_BOX (boxV), GTK_WIDGET (separator), true, true, 0);
		gtk_widget_show(separator);

	/*	->	Row 3	*/
		boxH = gtk_hbox_new(true, 0);
	/*		->	BoxH	*/
			gtk_box_pack_start(GTK_BOX (boxV), GTK_WIDGET (boxH), true, true, 0);
	/*			->	Button 1	*/
				button = gtk_button_new_with_label("\nSTART\n");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_2_1),
									(gpointer) "Button 2 - 1");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 2:	*/
				button = gtk_button_new_with_label("KILL");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_2_2),
									(gpointer) "Button 2 - 2");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 3:	*/
				button = gtk_button_new_with_label("CALIBRATE");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_2_3),
									(gpointer) "Button 2 - 3");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 4:	*/
				button = gtk_button_new_with_label("RST ATT");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_2_4),
									(gpointer) "Button 2 - 4");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 5:	*/
				button = gtk_button_new_with_label("TEST");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_2_5),
									(gpointer) "Button 2 - 5");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
	/*			->	Button 6:	*/
				button = gtk_button_new_with_label("MINIMUM");
				g_signal_connect(	G_OBJECT (button),
									"clicked",
									G_CALLBACK (qc_callback_2_6),
									(gpointer) "Button 2 - 6");
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (button), true, true, 0);
				gtk_widget_show(button);
		gtk_widget_show(boxH);

		separator = gtk_hseparator_new();
		gtk_box_pack_start(GTK_BOX (boxV), GTK_WIDGET (separator), true, true, 0);
		gtk_widget_show(separator);

	/*	->	Row 4	*/
		boxH = gtk_hbox_new(false, 0);
	/*		->	BoxH	*/
			gtk_box_pack_start(GTK_BOX (boxV), GTK_WIDGET (boxH), true, true, 0);
			boxSub0 = gtk_vbox_new(false, 0);
	/*			->	boxSub0	*/
				gtk_box_pack_start(GTK_BOX (boxH), GTK_WIDGET (boxSub0), true, true, 0);
	/*				->	Control Scheme Combo	*/
					comboBox = gtk_combo_box_new_text();
						gtk_box_pack_start(GTK_BOX (boxSub0), comboBox, false, false, 0);
						gtk_combo_box_append_text(GTK_COMBO_BOX (comboBox), "AD Strafe (default)");
						gtk_combo_box_append_text(GTK_COMBO_BOX (comboBox), "AD Rotate");
						gtk_combo_box_append_text(GTK_COMBO_BOX (comboBox), "Shooter Style");
						gtk_combo_box_set_active(GTK_COMBO_BOX (comboBox), 0);
						g_signal_connect(	G_OBJECT (comboBox),
											"changed",
											G_CALLBACK (qc_callback_control_scheme),
											(gpointer) "Control Scheme Combo Box");
					gtk_widget_show(comboBox);
	/*				->	Controls Table	*/
					controlsTable = gtk_table_new(3, 5, false);
					gtk_box_pack_start(GTK_BOX (boxSub0), controlsTable, true, true, 0);
	/*					->	UP	*/
						button = gtk_button_new();
						g_signal_connect(	G_OBJECT (button),
											"pressed",
											G_CALLBACK (moveForeward),
											(gpointer) "UP");
						g_signal_connect(	G_OBJECT (button),
											"released",
											G_CALLBACK (qc_release),
											(gpointer) "UP");
						arrow  = gtk_arrow_new(GTK_ARROW_UP, GTK_SHADOW_OUT);
						gtk_container_add(GTK_CONTAINER (button), arrow);
						gtk_table_attach(GTK_TABLE (controlsTable), GTK_WIDGET (button), 2, 3, 0, 1, GTK_SHRINK, GTK_SHRINK, 0, 0);
						gtk_widget_show(button);
						gtk_widget_show(arrow);
	/*					->	LEFT	*/
						button = gtk_button_new();
						g_signal_connect(	G_OBJECT (button),
											"pressed",
											G_CALLBACK (strafeLeft),
											(gpointer) "LEFT");
						g_signal_connect(	G_OBJECT (button),
											"released",
											G_CALLBACK (qc_release),
											(gpointer) "LEFT");
						arrow  = gtk_arrow_new(GTK_ARROW_LEFT, GTK_SHADOW_OUT);
						gtk_container_add(GTK_CONTAINER (button), arrow);
						gtk_table_attach(GTK_TABLE (controlsTable), GTK_WIDGET (button), 1, 2, 1, 2, GTK_SHRINK, GTK_SHRINK, 0, 0);
						gtk_widget_show(button);
						gtk_widget_show(arrow);
	/*					->	RIGHT	*/
						button = gtk_button_new();
						g_signal_connect(	G_OBJECT (button),
											"pressed",
											G_CALLBACK (strafeRight),
											(gpointer) "RIGHT");
						g_signal_connect(	G_OBJECT (button),
											"released",
											G_CALLBACK (qc_release),
											(gpointer) "RIGHT");
						arrow  = gtk_arrow_new(GTK_ARROW_RIGHT, GTK_SHADOW_OUT);
						gtk_container_add(GTK_CONTAINER (button), arrow);
						gtk_table_attach(GTK_TABLE (controlsTable), GTK_WIDGET (button), 3, 4, 1, 2, GTK_SHRINK, GTK_SHRINK, 0, 0);
						gtk_widget_show(button);
						gtk_widget_show(arrow);
	/*					->	DOWN	*/
						button = gtk_button_new();
						g_signal_connect(	G_OBJECT (button),
											"pressed",
											G_CALLBACK (moveBack),
											(gpointer) "DOWN");
						g_signal_connect(	G_OBJECT (button),
											"released",
											G_CALLBACK (qc_release),
											(gpointer) "DOWN");
						arrow  = gtk_arrow_new(GTK_ARROW_DOWN, GTK_SHADOW_OUT);
						gtk_container_add(GTK_CONTAINER (button), arrow);
						gtk_table_attach(GTK_TABLE (controlsTable), GTK_WIDGET (button), 2, 3, 2, 3, GTK_SHRINK, GTK_SHRINK, 0, 0);
						gtk_widget_show(button);
						gtk_widget_show(arrow);
	/*					->	ROTATE LEFT	*/
						button = gtk_button_new_with_label("RO L");
						g_signal_connect(	G_OBJECT (button),
											"pressed",
											G_CALLBACK (rotateLeft),
											(gpointer) "ROTATE LEFT");
						g_signal_connect(	G_OBJECT (button),
											"released",
											G_CALLBACK (qc_release),
											(gpointer) "ROTATE LEFT");
						arrow  = gtk_arrow_new(GTK_ARROW_DOWN, GTK_SHADOW_OUT);
						gtk_table_attach(GTK_TABLE (controlsTable), GTK_WIDGET (button), 0, 1, 0, 1, GTK_SHRINK, GTK_SHRINK, 0, 0);
						gtk_widget_show(button);
	/*					->	ROTATE RIGHT	*/
						button = gtk_button_new_with_label("RO R");
						g_signal_connect(	G_OBJECT (button),
											"pressed",
											G_CALLBACK (rotateRight),
											(gpointer) "ROTATE RIGHT");
						g_signal_connect(	G_OBJECT (button),
											"released",
											G_CALLBACK (qc_release),
											(gpointer) "ROTATE RIGHT");
						arrow  = gtk_arrow_new(GTK_ARROW_DOWN, GTK_SHADOW_OUT);
						gtk_table_attach(GTK_TABLE (controlsTable), GTK_WIDGET (button), 4, 5, 0, 1, GTK_SHRINK, GTK_SHRINK, 0, 0);
						gtk_widget_show(button);
	/*					->	ASCEND	*/
						button = gtk_button_new_with_label("ASC");
						g_signal_connect(	G_OBJECT (button),
											"pressed",
											G_CALLBACK (altAscend),
											(gpointer) "ASCEND");
						g_signal_connect(	G_OBJECT (button),
											"released",
											G_CALLBACK (qc_release),
											(gpointer) "ASCEND");
						arrow  = gtk_arrow_new(GTK_ARROW_DOWN, GTK_SHADOW_OUT);
						gtk_table_attach(GTK_TABLE (controlsTable), GTK_WIDGET (button), 0, 1, 2, 3, GTK_SHRINK, GTK_SHRINK, 0, 0);
						gtk_widget_show(button);
	/*					->	DESCEND	*/
						button = gtk_button_new_with_label("DES");
						g_signal_connect(	G_OBJECT (button),
											"pressed",
											G_CALLBACK (altDescend),
											(gpointer) "DESCEND");
						g_signal_connect(	G_OBJECT (button),
											"released",
											G_CALLBACK (qc_release),
											(gpointer) "DESCEND");
						arrow  = gtk_arrow_new(GTK_ARROW_DOWN, GTK_SHADOW_OUT);
						gtk_table_attach(GTK_TABLE (controlsTable), GTK_WIDGET (button), 4, 5, 2, 3, GTK_SHRINK, GTK_SHRINK, 0, 0);
						gtk_widget_show(button);
					gtk_widget_show(controlsTable);
				gtk_widget_show(boxSub0);
			gtk_widget_show(boxH);
		gtk_widget_show(boxV);

	/*	Ready to rumble	*/
	gtk_widget_show(mainWindow);

	/*	Run everything	*/
	gtk_main();
	
	/*	Quit	*/
	return 0;
}