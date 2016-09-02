#pragma once 

#include <gtkmm.h>
#include <gtkmm/grid.h>
#include <gtkmm/entry.h>
#include <gtkmm/button.h>
#include <gtkmm/radiobutton.h>
#include <gtkmm/messagedialog.h>
#include <gtkmm/window.h>
#include <gtkmm/scrolledwindow.h>
#include <gtkmm/application.h>
#include <gtkmm/comboboxtext.h>
#include <gtkmm/liststore.h>
#include <gtkmm/textview.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <dirent.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <cstring>
#include <sstream>
#include <string>


namespace od
{
	namespace g2d
	{

		class Annotation : public Gtk::Window
		{
			public:
				Annotation();
				virtual ~Annotation();

			protected:
				void on_button_clicked(Glib::ustring data);
				void on_combo_changed();
				void on_cell_data_extra(const Gtk::TreeModel::const_iterator& iter);
				void showWindow_main();
				void showWindow_imageLoad(Glib::ustring data);
				bool on_button_press_event(GdkEventButton *event);
				void createDot(int x, int y);
				void loadOriginalImage(std::string data, bool st);
				void createVisualROI(int xPressed, int yPressed, int xReleased, int yReleased);
				void loadOriginalImageWithSavedMarkings(std::string filename, std::vector<std::vector<int> > & storageROILocationMultiple, bool st);
				void loadResetedMarkings(std::string data, std::vector<std::vector<int> > & roi, bool st);
				// Child widgets:
				Gtk::Grid 		m_grid1,
							m_grid_imageLoad;

				Gtk::ScrolledWindow 	m_sw1, 
							m_sw_imageLoad;

				Gtk::Button	 	button_selectImageFileName,
							button_loadImage,
							button_loadAnotherImage,
							button_resetMarkings,
							button_loadMainWindow,
							button_selectRoi,
							button_saveMarked,
							button_saveCropMarked,
							button_resetMarkingsCurrent,
							button_selectRoiCurrent,
							button_saveMarkedMultiple,
							button_selectDatasetFolder,
							button_saveCropMarkedMultiple,
							button_loadNextImage,
							button_resetSegnetMaskCurrent,
							button_selectSegnetMaskCurrent,
							button_saveSegnetMask,
							button_quit;

				Gtk::RadioButton	rbutton_markBB, rbutton_cropBB, rbutton_markBBWithLabel,
							rbutton_cropMultipleBB, rbutton_segnetMaskedBased;

				Gtk::Label		label_annotationType,
							label_outputFile,
							label_annotationLabel;

				Gtk::Entry		text_outputFile,
							text_annotationLabel;

			private:
				Glib::ustring imageFileLocation;
				std::string	filename, foldername, file_folder;
				Gtk::Image image;
				double imgFocusX,imgFocusY, scale;
				int lastXMouse, lastYMouse, xPressed, yPressed, xReleased, yReleased, storage, imagesInFolder, storageID;
				bool resetFlag, moveFlag, targetFlag, status;

				std::string	currentWindow;
				std::vector<std::string> storageFileName, imageFileNames;
				std::vector<float> widthMultiplier, heightMultiplier;
				std::vector<std::vector<int> > storageROILocationCurrent, roiPointsForMask, roiPointsForMaskPermanent, storageROILocation;
				float	w,h;
		};

	}
}