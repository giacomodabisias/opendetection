#include "od/detectors/global2D/annotation/Annotation.h"

namespace od
{
	namespace g2d
	{

		void Annotation::showWindow_main()
		{
			storage = 0;
			storageROILocation.clear();
			storageROILocationCurrent.clear();
			storageFileName.clear();
			imageFileNames.clear();
			widthMultiplier.clear();
			heightMultiplier.clear();
			currentWindow = "mainWindow";
			remove();
			set_title("Network Creator");
			set_border_width(10);
			add(m_sw1);
			m_grid1.set_column_spacing (10);
			m_grid1.set_row_spacing (50);
		//	m_sw1.add(m_grid1);
			m_sw1.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
		//	m_grid1.show();
			show_all_children();
			m_sw1.show();	
		}

		void Annotation::showWindow_imageLoad(Glib::ustring data)
		{
			cv::Mat img = cv::imread(filename, 1);
			cv::imwrite("../examples/objectdetector/Annotation/temp.png", img);
			currentWindow = "imageLoadWindow";
			remove();
			set_title("Image Window");
			set_border_width(10);
			add(m_sw_imageLoad);
			m_grid_imageLoad.set_column_spacing (10);
			m_grid_imageLoad.set_row_spacing (10);

			if(rbutton_markBB.get_active() == 1)
			{
				label_annotationType.set_text("Annotion Type: Selecting Single Bounding Box");
			}
			else if(rbutton_cropBB.get_active() == 1)
			{
				label_annotationType.set_text("Annotion Type: Cropping Single Bounding Box");
			}
			else if(rbutton_markBBWithLabel.get_active() == 1)
			{
				label_annotationType.set_text("Annotion Type: Selecting Multiple Bounding Boxes and labelling them");
			}

			label_annotationType.set_line_wrap();
			label_annotationType.set_justify(Gtk::JUSTIFY_FILL);
		//	m_grid_imageLoad.attach(label_annotationType,0,0,2,1);
			label_annotationType.show();

			
			if(rbutton_markBB.get_active() == 1 && file_folder == "file")
			{
				
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();

				loadOriginalImage(data, 0);
				button_resetMarkings.show();
				button_selectRoi.show();
				button_loadAnotherImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();
				
				button_saveMarked.show();
				
			}
			else if(rbutton_cropBB.get_active() == 1 && file_folder == "file")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();


				loadOriginalImage(data, 0);
				button_resetMarkings.show();
				button_selectRoi.show();
				button_loadAnotherImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();
				
				button_saveCropMarked.show();
			}
			else if(rbutton_markBBWithLabel.get_active() == 1 && file_folder == "file")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();

				loadOriginalImage(data, 0);
				button_resetMarkingsCurrent.show();
			
				label_annotationLabel.set_text("Give a label to ROI: ");
				label_annotationLabel.set_line_wrap();
				label_annotationLabel.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_annotationLabel,0,4,2,1);
				label_annotationLabel.show();
				

				text_annotationLabel.set_max_length(100);
				text_annotationLabel.set_text("");
				text_annotationLabel.select_region(0, text_annotationLabel.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,4,1,1);	
				text_annotationLabel.show();
				
				button_selectRoiCurrent.show();
				button_loadAnotherImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();
				
				button_saveMarkedMultiple.show();

				
			}
			else if(rbutton_cropMultipleBB.get_active() == 1 && file_folder == "file")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();

				loadOriginalImage(data, 0);
				button_resetMarkingsCurrent.show();
			
				label_annotationLabel.set_text("Give a label to ROI: ");
				label_annotationLabel.set_line_wrap();
				label_annotationLabel.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_annotationLabel,0,4,2,1);
				label_annotationLabel.show();
				

				text_annotationLabel.set_max_length(100);
				text_annotationLabel.set_text("");
				text_annotationLabel.select_region(0, text_annotationLabel.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,4,1,1);	
				text_annotationLabel.show();
				
				button_selectRoiCurrent.show();
				button_loadAnotherImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();
				
				button_saveCropMarkedMultiple.show();

				
			}
			else if(rbutton_segnetMaskedBased.get_active() == 1 && file_folder == "file")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_saveCropMarked.hide();
				button_loadAnotherImage.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();
				button_resetSegnetMaskCurrent.hide();	
				button_selectSegnetMaskCurrent.hide();
				button_saveSegnetMask.hide();

				loadOriginalImage(data, 0);
				button_resetSegnetMaskCurrent.show();	
				button_selectSegnetMaskCurrent.show();
				
				label_annotationLabel.set_text("Give a label to ROI: ");
				label_annotationLabel.set_line_wrap();
				label_annotationLabel.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_annotationLabel,0,4,2,1);
				label_annotationLabel.show();
				

				text_annotationLabel.set_max_length(100);
				text_annotationLabel.set_text("");
				text_annotationLabel.select_region(0, text_annotationLabel.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,4,1,1);	
				text_annotationLabel.show();
			
				button_loadAnotherImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();
				
				button_saveSegnetMask.show();
			}
			else if(rbutton_markBB.get_active() == 1 && file_folder == "folder")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();

				
				DIR *dpdf;
				struct dirent *epdf;

				dpdf = opendir(foldername.c_str());
				if (dpdf != NULL)
				{
					while (epdf = readdir(dpdf))
					{
						std::string val = epdf->d_name;
						if(val.length() > 3)
						{
							std::string fName = foldername + "/" + epdf->d_name;
							std::cout << fName << std::endl; 
							imageFileNames.push_back(fName); 
							imagesInFolder++;
						}
				   	}
				}
				filename = imageFileNames[--imagesInFolder];
				loadOriginalImage(filename, 0);

				button_resetMarkings.show();
				button_selectRoi.show();
				button_loadNextImage.show();

				button_loadAnotherImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();
				
				button_saveMarked.show();
			}
			else if(rbutton_cropBB.get_active() == 1 && file_folder == "folder")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();

				
				DIR *dpdf;
				struct dirent *epdf;

				dpdf = opendir(foldername.c_str());
				if (dpdf != NULL)
				{
					while (epdf = readdir(dpdf))
					{
						std::string val = epdf->d_name;
						if(val.length() > 3)
						{
							std::string fName = foldername + "/" + epdf->d_name;
							std::cout << fName << std::endl; 
							imageFileNames.push_back(fName); 
							imagesInFolder++;
						}
				   	}
				}
				filename = imageFileNames[--imagesInFolder];
				loadOriginalImage(filename, 0);

				button_resetMarkings.show();
				button_selectRoi.show();
				button_loadNextImage.show();

				button_loadAnotherImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();

				button_saveCropMarked.show();

			}
			else if(rbutton_markBBWithLabel.get_active() == 1 && file_folder == "folder")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();

				
				DIR *dpdf;
				struct dirent *epdf;

				dpdf = opendir(foldername.c_str());
				if (dpdf != NULL)
				{
					while (epdf = readdir(dpdf))
					{
						std::string val = epdf->d_name;
						if(val.length() > 3)
						{
							std::string fName = foldername + "/" + epdf->d_name;
							std::cout << fName << std::endl; 
							imageFileNames.push_back(fName); 
							imagesInFolder++;
						}
				   	}
				}
				filename = imageFileNames[--imagesInFolder];
				loadOriginalImage(filename, 0);

				button_resetMarkingsCurrent.show();
			
				label_annotationLabel.set_text("Give a label to ROI: ");
				label_annotationLabel.set_line_wrap();
				label_annotationLabel.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_annotationLabel,0,4,2,1);
				label_annotationLabel.show();
				

				text_annotationLabel.set_max_length(100);
				text_annotationLabel.set_text("");
				text_annotationLabel.select_region(0, text_annotationLabel.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,4,1,1);	
				text_annotationLabel.show();
				
				button_selectRoiCurrent.show();
				button_loadAnotherImage.show();
				button_loadNextImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();
				
				button_saveMarkedMultiple.show();
			}
			else if(rbutton_cropMultipleBB.get_active() == 1 && file_folder == "folder")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();

				
				DIR *dpdf;
				struct dirent *epdf;

				dpdf = opendir(foldername.c_str());
				if (dpdf != NULL)
				{
					while (epdf = readdir(dpdf))
					{
						std::string val = epdf->d_name;
						if(val.length() > 3)
						{
							std::string fName = foldername + "/" + epdf->d_name;
							std::cout << fName << std::endl; 
							imageFileNames.push_back(fName); 
							imagesInFolder++;
						}
				   	}
				}
				filename = imageFileNames[--imagesInFolder];
				loadOriginalImage(filename, 0);

				button_resetMarkingsCurrent.show();
			
				label_annotationLabel.set_text("Give a label to ROI: ");
				label_annotationLabel.set_line_wrap();
				label_annotationLabel.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_annotationLabel,0,4,2,1);
				label_annotationLabel.show();
				

				text_annotationLabel.set_max_length(100);
				text_annotationLabel.set_text("");
				text_annotationLabel.select_region(0, text_annotationLabel.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,4,1,1);	
				text_annotationLabel.show();
				
				button_selectRoiCurrent.show();
				button_loadAnotherImage.show();
				button_loadNextImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();

				button_saveCropMarkedMultiple.show();
			}
			else if(rbutton_segnetMaskedBased.get_active() == 1 && file_folder == "folder")
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();
				button_resetSegnetMaskCurrent.hide();	
				button_selectSegnetMaskCurrent.hide();
				button_saveSegnetMask.hide();

				
				DIR *dpdf;
				struct dirent *epdf;

				dpdf = opendir(foldername.c_str());
				if (dpdf != NULL)
				{
					while (epdf = readdir(dpdf))
					{
						std::string val = epdf->d_name;
						if(val.length() > 3)
						{
							std::string fName = foldername + "/" + epdf->d_name;
							std::cout << fName << std::endl; 
							imageFileNames.push_back(fName); 
							imagesInFolder++;
						}
				   	}
				}
				filename = imageFileNames[--imagesInFolder];
				loadOriginalImage(filename, 0);
				button_resetSegnetMaskCurrent.show();	
				button_selectSegnetMaskCurrent.show();
				
				label_annotationLabel.set_text("Give a label to ROI: ");
				label_annotationLabel.set_line_wrap();
				label_annotationLabel.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_annotationLabel,0,4,2,1);
				label_annotationLabel.show();
				

				text_annotationLabel.set_max_length(100);
				text_annotationLabel.set_text("");
				text_annotationLabel.select_region(0, text_annotationLabel.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,4,1,1);	
				text_annotationLabel.show();
			
				button_loadAnotherImage.show();
				button_loadNextImage.show();

				label_outputFile.set_text("OutPut Text File Name: ");
				label_outputFile.set_line_wrap();
				label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
		//		m_grid_imageLoad.attach(label_outputFile,0,2,2,1);
				label_outputFile.show();
				

				text_outputFile.set_max_length(100);
				text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
				text_outputFile.select_region(0, text_outputFile.get_text_length());
		//		m_grid_imageLoad.attach(text_outputFile,2,2,1,1);	
				text_outputFile.show();
				
				button_saveSegnetMask.show();
			}
			else
			{
				label_outputFile.hide();
				text_outputFile.hide();
				button_resetMarkings.hide();
				button_selectRoi.hide();
				button_saveMarked.hide();
				button_loadAnotherImage.hide();
				button_saveCropMarked.hide();
				button_resetMarkingsCurrent.hide();
				button_selectRoiCurrent.hide();
				label_annotationLabel.hide();
				text_annotationLabel.hide();
				button_saveMarkedMultiple.hide();
				button_loadNextImage.hide();
				button_saveCropMarkedMultiple.hide();
			}

				
			
			button_loadMainWindow.show();	
			
			
			m_sw_imageLoad.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
			m_grid_imageLoad.show();
		//	show_all_children();
			m_sw_imageLoad.show();
		}


		bool Annotation::on_button_press_event(GdkEventButton *event)
		{
			// Check if the event is a left button click.
			if (event->button == 1)
			{
				// Memorize pointer position
				lastXMouse=event->x;
				lastYMouse=event->y;
				xPressed=event->x;
				yPressed=event->y;
				// Start moving the view
				moveFlag=true;
				// Event has been handled
		//		cout << lastXMouse << " " << lastYMouse << endl;
				if(currentWindow == "imageLoadWindow")
				{
		//			std::cout << lastXMouse - 10 << " " << lastYMouse - 10 << std::endl;
					createDot(lastXMouse-10, lastYMouse-35);
				}
				if(rbutton_segnetMaskedBased.get_active() == 1)
				{
					std::vector <int> temp;
					std::string label = text_annotationLabel.get_text();
					int Result; 
					std::stringstream convert(label);
					if ( !(convert >> Result) )
						Result = 0;
					temp.push_back(Result);
					temp.push_back(lastXMouse-10);
					temp.push_back(lastYMouse-35);
					roiPointsForMask.push_back(temp);
				}
				return true;
			}

			// Check if the event is a right button click.
			if(event->button == 4)
			{
				// Memorize mouse coordinates
				lastXMouse=event->x;
				lastYMouse=event->y;
				xReleased=event->x;
				yReleased=event->y;
				if(currentWindow == "imageLoadWindow")
				{
					createVisualROI(xPressed-10, yPressed-35, xReleased-10, yReleased-35);
				}
				// Display the popup menu
		//		m_Menu_Popup.popup(event->button, event->time);
				// The event has been handled.
				return true;
			}
		 
			// Check if the event is a right button click.
			if(event->button == 3)
			{
				// Memorize mouse coordinates
				lastXMouse=event->x;
				lastYMouse=event->y;
				xReleased=event->x;
				yReleased=event->y;
				createVisualROI(xPressed-10, yPressed-35, xReleased-10, yReleased-35);
				// Display the popup menu
		//		m_Menu_Popup.popup(event->button, event->time);
				// The event has been handled.
				return true;
			}
			return false;
		}


		void Annotation::createDot(int x, int y)
		{
			cv::Mat img = cv::imread("../examples/objectdetector/Annotation/temp_resized_dotted.png", 1);
			cv::circle(img, cv::Point(x,y), 3, cv::Scalar(255,0,0), 3, 8, 0 );
			cv::imwrite("../examples/objectdetector/Annotation/temp_resized_dotted.png", img);
			image.set("../examples/objectdetector/Annotation/temp_resized_dotted.png");
			image.show();
		}

		void Annotation::createVisualROI(int x1, int y1, int x2, int y2)
		{
			std::string text = text_annotationLabel.get_text();
			int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
			double fontScale = 1;
			int thickness = 2;  
			cv::Point textOrg(x1, y1);
			cv::Mat img = cv::imread("../examples/objectdetector/Annotation/temp_resized_dotted.png", 1);
			cv::rectangle(img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(0,255,0), 3, 8, 0 );
			cv::putText(img, text, textOrg, fontFace, fontScale, cv::Scalar(0,0,255), thickness,8);
			cv::imwrite("../examples/objectdetector/Annotation/temp_resized_dotted.png", img);
			image.set("../examples/objectdetector/Annotation/temp_resized_dotted.png");
			image.show();
		}

		void Annotation::loadOriginalImage(std::string data, bool st)
		{
			cv::Mat img = cv::imread(data, 1);
			int c = img.cols;
			int r = img.rows;
			float wTemp = 1;
			float hTemp = 1;
		//	if(img.rows > 480 or img.cols > 640)
		//	{
				cv::resize(img, img, cv::Size(640,480));
				wTemp = (float)c/640.0;
				hTemp = (float)r/480.0;			
		//	}
			if(st)
			{
				widthMultiplier.push_back(wTemp);
				heightMultiplier.push_back(hTemp);
			}
			cv::imwrite("../examples/objectdetector/Annotation/temp_resized.png", img);
			cv::imwrite("../examples/objectdetector/Annotation/temp_resized_dotted.png", img);
			image.set("../examples/objectdetector/Annotation/temp_resized.png");
			image.show();
		}

		void Annotation::loadOriginalImageWithSavedMarkings(std::string data, std::vector<std::vector<int> > & roi, bool st)
		{
			cv::Mat img = cv::imread(data, 1);
			int c = img.cols;
			int r = img.rows;
			float wTemp = 1;
			float hTemp = 1;
		//	if(img.rows > 480 or img.cols > 640)
		//	{
				cv::resize(img, img, cv::Size(640,480));
				wTemp = (float)c/640.0;
				hTemp = (float)r/480.0;			
		//	}
			if(st)
			{
				widthMultiplier.push_back(wTemp);
				heightMultiplier.push_back(hTemp);
			}
			cv::imwrite("../examples/objectdetector/Annotation/temp_resized.png", img);
			for(int i = 0; i < roi.size(); i++)
			{
				if(storageFileName[i] == data)
					cv::rectangle(img, cv::Point(roi[i][1],roi[i][2]), cv::Point(roi[i][3],roi[i][4]), cv::Scalar(0,255,0), 3, 8, 0 );
			}
			cv::imwrite("../examples/objectdetector/Annotation/temp_resized_dotted.png", img);
			image.set("../examples/objectdetector/Annotation/temp_resized_dotted.png");
			image.show();
		}

		void Annotation::loadResetedMarkings(std::string data, std::vector<std::vector < int > > & roi, bool st)
		{
			

			cv::Mat img1 = cv::imread(data, 1);
			int c = img1.cols;	
			int r = img1.rows;
			float wTemp = 1.0;
			float hTemp = 1.0;
		//	if(img1.rows > 480 or img1.cols > 640)
		//	{
				cv::resize(img1, img1, cv::Size(640,480));
				wTemp = (float)c/640.0;
				hTemp = (float)r/480.0;			
		//	}
		//	if(st)
		//	{
		//		widthMultiplier.push_back(wTemp);
		//		heightMultiplier.push_back(hTemp);
		//	}
			cv::imwrite("../examples/objectdetector/Annotation/temp_resized.png", img1);
			if(roi.size() > 0)
			{
				for(int i = 0; i < roi.size()-1; i++)
				{
		//			cout << roi.size() << " " << roi[i+1][3] << " " << roi[i][3] << endl;
					if((roi[i+1][0] == roi[i][0]) && (roi[i+1][3] == roi[i][3]))
					{
						if(storageFileName[i] == filename)
							cv::line(img1, cv::Point(roi[i][1],roi[i][2]), cv::Point(roi[i+1][1],roi[i+1][2]), cv::Scalar(0,255,0), 3, 8, 0 );
					}
				}
			}
			cv::imwrite("../examples/objectdetector/Annotation/temp_resized_dotted.png", img1);
			image.set("../examples/objectdetector/Annotation/temp_resized_dotted.png");
			image.show(); 	
		}




		Annotation::Annotation(): 
			button_selectImageFileName("Select The Image Location"),
			button_loadImage("Load the image"),
			button_loadAnotherImage("Load Next Image from a different location"),
			rbutton_markBB("Save points for a single Bounding Box per Image"), 
			rbutton_cropBB("Save points and crop single Bounding Box per image"),
			rbutton_markBBWithLabel("Save Multiple Bounding Boxes' Points with attached class labels"),
			rbutton_cropMultipleBB("Save points and crop multiple Bounding Boxes per image with attached class labels"),
			rbutton_segnetMaskedBased("Create masks based on labels, as required by segnet"),
			label_annotationType(""),
			text_outputFile(),
			label_outputFile(""),
			button_resetMarkings("Reset Markings"),
			button_loadMainWindow("Load Main Window"),
			button_selectRoi("Select the ROI"),
			button_saveMarked("Save the ROI details of all images into the stated file"),
			button_saveCropMarked("Save the ROI details with of all images into the stated file and crop the ROI and save"),
			button_resetMarkingsCurrent("Reset the current Marking"),
			button_selectRoiCurrent("Select the roi with mentioned label"),
			button_saveMarkedMultiple("Save the ROI details with labels of all images into the stated file"),
			button_saveCropMarkedMultiple("Save and crop the ROIs with labels of all images into the stated file"),
			text_annotationLabel(),
			label_annotationLabel(""),
			button_selectDatasetFolder("Select the Dataset Folder"),
			button_loadNextImage("Load Next Image from same folder"),
			button_resetSegnetMaskCurrent("Reset Current Markings"),
			button_selectSegnetMaskCurrent("Save the region's points with stated label"),
			button_saveSegnetMask("Save all Masks"),

			button_quit("Quit")
		{
			storage = 0;
			storageID=0;
			imagesInFolder = 0;
			currentWindow = "mainWindow";
			set_title("Annotator");
			set_border_width(10);
			add(m_sw1);
			m_grid1.set_column_spacing (10);
			m_grid1.set_row_spacing (50);

			
			button_selectImageFileName.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "selectImageFileName"));
			m_grid1.attach(button_selectImageFileName,0,1,1,1);
			button_selectImageFileName.show();

			button_selectDatasetFolder.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "selectDatasetFolder"));
			m_grid1.attach(button_selectDatasetFolder,1,1,1,1);
			button_selectDatasetFolder.show();

			Gtk::RadioButton::Group group1 = rbutton_markBB.get_group();
			rbutton_cropBB.set_group(group1);
			rbutton_markBBWithLabel.set_group(group1);
			rbutton_cropMultipleBB.set_group(group1);
			rbutton_segnetMaskedBased.set_group(group1);
		 	rbutton_markBB.set_active();
			m_grid1.attach(rbutton_markBB,0,2,1,1);
			rbutton_markBB.show();
			m_grid1.attach(rbutton_cropBB,1,2,1,1);
			rbutton_cropBB.show();
			m_grid1.attach(rbutton_markBBWithLabel,2,2,1,1);
			rbutton_markBBWithLabel.show();
			m_grid1.attach(rbutton_cropMultipleBB,0,3,2,1);
			rbutton_cropMultipleBB.show();
			m_grid1.attach(rbutton_segnetMaskedBased,2,3,2,1);
			rbutton_segnetMaskedBased.show();

			button_loadImage.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "loadImage"));
			m_grid1.attach(button_loadImage,0,4,1,1);
			button_loadImage.show();

		/*
			Gtk::RadioButton::Group group1 = rbutton_extraLayerLMDB.get_group();
			rbutton_extraLayerLEVELDB.set_group(group15);
		 	rbutton_extraLayerLMDB.set_active();
			m_grid_extraLayerType.attach(rbutton_extraLayerLMDB,2,16,1,1);
			rbutton_extraLayerLMDB.show();
			m_grid_extraLayerType.attach(rbutton_extraLayerLEVELDB,3,16,1,1);
			rbutton_extraLayerLEVELDB.show();
		*/
			button_quit.signal_pressed().connect(sigc::mem_fun(*this,&Annotation::close));
			m_grid1.attach(button_quit,0,10,1,1);



			//imageLoadWindow

			button_loadMainWindow.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "loadMainWindow"));
			m_grid_imageLoad.attach(button_loadMainWindow,0,10,1,1);
			button_resetMarkings.show();

			m_grid_imageLoad.attach(image,0,1,2,1);
			
			
			
			label_annotationType.set_text("Annotion Type: Selecting Single Bounding Box");
			label_annotationType.set_line_wrap();
			label_annotationType.set_justify(Gtk::JUSTIFY_FILL);
			m_grid_imageLoad.attach(label_annotationType,0,0,2,1);
			label_annotationType.show();

			button_resetMarkings.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "resetMarkings"));
			m_grid_imageLoad.attach(button_resetMarkings,0,3,1,1);
			button_resetMarkings.show();

			button_resetMarkingsCurrent.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "resetMarkingsCurrent"));
			m_grid_imageLoad.attach(button_resetMarkingsCurrent,0,3,1,1);
			button_resetMarkingsCurrent.show();

			button_resetSegnetMaskCurrent.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "resetSegnetMaskCurrent"));
			m_grid_imageLoad.attach(button_resetSegnetMaskCurrent,0,3,1,1);
			button_resetSegnetMaskCurrent.show();

			button_selectRoi.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "selectRoi"));
			m_grid_imageLoad.attach(button_selectRoi,0,4,1,1);
			button_selectRoi.show();

			label_annotationLabel.set_text("Give a label to ROI: ");
			label_annotationLabel.set_line_wrap();
			label_annotationLabel.set_justify(Gtk::JUSTIFY_FILL);
			m_grid_imageLoad.attach(label_annotationLabel,0,4,1,1);
			label_annotationLabel.show();
				

			text_annotationLabel.set_max_length(100);
			text_annotationLabel.set_text("");
			text_annotationLabel.select_region(0, text_annotationLabel.get_text_length());
			m_grid_imageLoad.attach(text_annotationLabel,1,4,1,1);	
			text_annotationLabel.show();

			button_selectRoiCurrent.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "selectRoiCurrent"));
			m_grid_imageLoad.attach(button_selectRoiCurrent,2,4,1,1);
			button_selectRoiCurrent.show();

			button_selectSegnetMaskCurrent.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "selectSegnetMaskCurrent"));
			m_grid_imageLoad.attach(button_selectSegnetMaskCurrent,2,4,1,1);
			button_selectSegnetMaskCurrent.show();

			button_loadAnotherImage.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "loadAnotherImage"));
			m_grid_imageLoad.attach(button_loadAnotherImage,0,5,2,1);
			button_loadMainWindow.show();
			
			button_loadNextImage.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "loadNextImage"));
			m_grid_imageLoad.attach(button_loadNextImage,2,5,1,1);
			button_loadNextImage.show();

			label_outputFile.set_text("OutPut Text File Name: ");
			label_outputFile.set_line_wrap();
			label_outputFile.set_justify(Gtk::JUSTIFY_FILL);
			m_grid_imageLoad.attach(label_outputFile,0,6,1,1);
			label_outputFile.show();
				

			text_outputFile.set_max_length(100);
			text_outputFile.set_text("../examples/objectdetector/Annotation/output.txt");
			text_outputFile.select_region(0, text_outputFile.get_text_length());
			m_grid_imageLoad.attach(text_outputFile,1,6,1,1);	
			text_outputFile.show();

			button_saveMarked.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "saveMarked"));
			m_grid_imageLoad.attach(button_saveMarked,2,6,1,1);
			button_saveMarked.show();

			button_saveCropMarked.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "saveCropMarked"));
			m_grid_imageLoad.attach(button_saveCropMarked,2,6,2,1);
			button_saveCropMarked.show();

			button_saveMarkedMultiple.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "saveMarkedMultiple"));
			m_grid_imageLoad.attach(button_saveMarkedMultiple,2,6,1,1);
			button_saveMarkedMultiple.show();

			button_saveCropMarkedMultiple.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "saveCropMarkedMultiple"));
			m_grid_imageLoad.attach(button_saveCropMarkedMultiple,2,6,1,1);
			button_saveCropMarkedMultiple.show();

			button_saveSegnetMask.signal_clicked().connect(sigc::bind<Glib::ustring>(
		              sigc::mem_fun(*this, &Annotation::on_button_clicked), "saveSegnetMask"));
			m_grid_imageLoad.attach(button_saveSegnetMask,2,6,1,1);
			button_saveSegnetMask.show();

			m_sw_imageLoad.add(m_grid_imageLoad);

			m_sw1.add(m_grid1);
			m_sw1.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
		//	m_grid1.show();
			show_all_children();
			m_sw1.show();

		}

		Annotation::~Annotation()
		{}

		void Annotation::on_button_clicked(Glib::ustring data)
		{
			if(data == "selectImageFileName" || data == "loadAnotherImage")
			{
				if(data == "selectImageFileName")
					file_folder = "file";
				// Create the dialog box FileChooser
				Gtk::FileChooserDialog dialog("Please choose a file",Gtk::FILE_CHOOSER_ACTION_OPEN);
				dialog.set_transient_for(*this);

				//Add response buttons the the dialog:
				dialog.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
				dialog.add_button("_Open", Gtk::RESPONSE_OK);

				//Add filters, so that only certain file types can be selected:
				Glib::RefPtr<Gtk::FileFilter> filter_any = Gtk::FileFilter::create();
				filter_any->set_name("Any files");
				filter_any->add_pattern("*");
				dialog.add_filter(filter_any);

				Glib::RefPtr<Gtk::FileFilter> filter_text = Gtk::FileFilter::create();
				filter_text->set_name("Text files");
				filter_text->add_mime_type("text/plain");
				dialog.add_filter(filter_text);

				Glib::RefPtr<Gtk::FileFilter> filter_cpp = Gtk::FileFilter::create();
				filter_cpp->set_name("C/C++ files");
				filter_cpp->add_mime_type("text/x-c");
				filter_cpp->add_mime_type("text/x-c++");
				filter_cpp->add_mime_type("text/x-c-header");
				dialog.add_filter(filter_cpp);



				//Show the dialog and wait for a user response:
				int result = dialog.run();

				//Handle the response:
				switch(result)
				{
					case(Gtk::RESPONSE_OK):
					{
						// The user selected a file
						std::cout << "Open clicked." << std::endl;
						filename = dialog.get_filename();
						std::cout << "File selected: " <<  filename << std::endl;
						break;
					}
					case(Gtk::RESPONSE_CANCEL):
					{
						// The user clicked cancel
						std::cout << "Cancel clicked." << std::endl;
						break;
					}
					default:
					{
						// The user closed the dialog box
						std::cout << "Unexpected button clicked." << std::endl;
						break;
					}
				}
				if(data == "loadAnotherImage")
					loadOriginalImage(filename, 1);
			}
			else if(data == "selectDatasetFolder")
			{
				file_folder = "folder";
				// Create the dialog box FileChooser
				Gtk::FileChooserDialog dialog("Please choose a file",Gtk::FILE_CHOOSER_ACTION_SELECT_FOLDER);
				dialog.set_transient_for(*this);

				//Add response buttons the the dialog:
				dialog.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
				dialog.add_button("_Open", Gtk::RESPONSE_OK);

				//Add filters, so that only certain file types can be selected:
				Glib::RefPtr<Gtk::FileFilter> filter_any = Gtk::FileFilter::create();
				filter_any->set_name("Any files");
				filter_any->add_pattern("*");
				dialog.add_filter(filter_any);

				Glib::RefPtr<Gtk::FileFilter> filter_text = Gtk::FileFilter::create();
				filter_text->set_name("Text files");
				filter_text->add_mime_type("text/plain");
				dialog.add_filter(filter_text);

				Glib::RefPtr<Gtk::FileFilter> filter_cpp = Gtk::FileFilter::create();
				filter_cpp->set_name("C/C++ files");
				filter_cpp->add_mime_type("text/x-c");
				filter_cpp->add_mime_type("text/x-c++");
				filter_cpp->add_mime_type("text/x-c-header");
				dialog.add_filter(filter_cpp);



				//Show the dialog and wait for a user response:
				int result = dialog.run();

				//Handle the response:
				switch(result)
				{
					case(Gtk::RESPONSE_OK):
					{
						// The user selected a file
						std::cout << "Open clicked." << std::endl;
						foldername = dialog.get_filename();
						std::cout << "File selected: " <<  filename << std::endl;
						break;
					}
					case(Gtk::RESPONSE_CANCEL):
					{
						// The user clicked cancel
						std::cout << "Cancel clicked." << std::endl;
						break;
					}
					default:
					{
						// The user closed the dialog box
						std::cout << "Unexpected button clicked." << std::endl;
						break;
					}
				}
				if(data == "loadAnotherImage")
					loadOriginalImage(filename, 1);
			}
			else if(data == "loadNextImage")
			{
				if(imagesInFolder > 0)
				{
					filename = imageFileNames[--imagesInFolder];
					loadOriginalImage(filename, 1);
				}
			}
			else if(data == "loadImage")
			{
				showWindow_imageLoad(filename);
			}
			else if(data == "loadMainWindow")
			{
				showWindow_main();
			}
			else if(data == "resetMarkings")
			{
				loadOriginalImage(filename, 0);
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("Recently marked Points have been discarded");
				dialog.run();
			}
			else if(data == "resetMarkingsCurrent")
			{
				loadOriginalImageWithSavedMarkings(filename, storageROILocationCurrent, 0);
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("Recently marked Points have been discarded");
				dialog.run();
			}
			else if(data == "resetSegnetMaskCurrent")
			{
		/*		int length = roiPointsForMask.size();
				std::string label = text_annotationLabel.get_text();
				int Result; 
				istringstream convert(label);
				if ( !(convert >> Result) )
					Result = 0;
				for( int i = length-1; i > -1; i--)
				{
					if(roiPointsForMask[roiPointsForMask.size()-1][0] == Result)	
						roiPointsForMask.pop_back();
					else
						break;
				}
		*/
				roiPointsForMask.clear();
				loadResetedMarkings(filename, roiPointsForMaskPermanent, 0);
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("Recently marked Points have been discarded");
				dialog.run();	
			}
			else if(data == "selectRoi")
			{
				std::vector<int> temp;
				temp.push_back(xPressed-10);
				temp.push_back(yPressed-35);
				temp.push_back(xReleased-10);
				temp.push_back(yReleased-35);

				storageROILocation.push_back(temp);
				storageFileName.push_back(filename);	
				storage++;
				loadOriginalImage(filename, 1);	
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("ROI has been added to the list");
				dialog.run();	
			}
			else if(data == "selectRoiCurrent")
			{
				std::vector<int> temp;
				std::string label = text_annotationLabel.get_text();
				int Result; 
				std::stringstream convert(label);
				if(!(convert >> Result))
					Result = 0;
				temp.push_back(Result);
				temp.push_back(xPressed-10);
				temp.push_back(yPressed-35);
				temp.push_back(xReleased-10);
				temp.push_back(yReleased-35);
				storageROILocationCurrent.push_back(temp);
				storageFileName.push_back(filename);
				storage++;
				std::cout << "filename = " << filename << std::endl;
				loadOriginalImageWithSavedMarkings(filename, storageROILocationCurrent, 1);
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("ROI has been added to the list");
				dialog.run();	
				
			}
			else if(data == "selectSegnetMaskCurrent")
			{
				storageID++;
				cv::Mat img1 = cv::imread(filename, 1);
				int c = img1.cols;	
				int r = img1.rows;
				cv::resize(img1, img1, cv::Size(640,480));
				float wTemp = (float)c/640.0;
				float hTemp = (float)r/480.0;			
				
				for(unsigned int i = 0; i < roiPointsForMask.size(); ++i)
				{
					std::vector<int> temp;		
					temp.push_back(roiPointsForMask[i][0]);
					temp.push_back(roiPointsForMask[i][1]);
					temp.push_back(roiPointsForMask[i][2]);
					temp.push_back(storageID);
					roiPointsForMaskPermanent.push_back(temp);
					storageFileName.push_back(filename);
					widthMultiplier.push_back(wTemp);
					heightMultiplier.push_back(hTemp);
					storage++;
				}
				roiPointsForMask.clear();

				loadResetedMarkings(filename, roiPointsForMaskPermanent, 1);	
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("ROI has been added to the list");
				dialog.run();		
		//		roiPointsForMaskPermanent
			}
			else if(data == "saveMarked")
			{
				std::ofstream myfile;
				myfile.open(text_outputFile.get_text());
				for(unsigned int i = 0; i < storage; ++i)
				{
					myfile << storageFileName[i] << " " << int(storageROILocation[i][0]*widthMultiplier[i]) << " " << int(storageROILocation[i][1]*heightMultiplier[i]) << " " << int(storageROILocation[i][2]*widthMultiplier[i]) << " " << int(storageROILocation[i][3]*heightMultiplier[i]) << std::endl;
				}
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("The details have been saved into the file " + text_outputFile.get_text());

		  		dialog.run();
			}
			else if(data == "saveMarkedMultiple")
			{
				std::ofstream myfile;
				myfile.open(text_outputFile.get_text());
				std::string tempFileName(storageFileName[0]);
				
				for(unsigned int i = 0; i < storage; i++)
				{
					if(i == 0)
					{
						myfile << storageFileName[i] << " " << storageROILocationCurrent[i][0] << " " << int(storageROILocationCurrent[i][1]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][2]*heightMultiplier[i]) << " " << int(storageROILocationCurrent[i][3]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][4]*heightMultiplier[i]) << " ";
					}
					else
					{	
						if(tempFileName == storageFileName[i])
						{
							myfile << storageROILocationCurrent[i][0] << " " << int(storageROILocationCurrent[i][1]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][2]*heightMultiplier[i]) << " " << int(storageROILocationCurrent[i][3]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][4]*heightMultiplier[i]) << " ";
						}
						else
						{
							myfile << std::endl;
							myfile << storageFileName[i] << " " << storageROILocationCurrent[i][0] << " " << int(storageROILocationCurrent[i][1]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][2]*heightMultiplier[i]) << " " << int(storageROILocationCurrent[i][3]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][4]*heightMultiplier[i]) << " ";
							tempFileName = storageFileName[i];
						}
					}
				}
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("The details have been saved into the file " + text_outputFile.get_text());

		  		dialog.run();
			}
			else if(data == "saveCropMarked")
			{
				std::ofstream myfile;
				myfile.open(text_outputFile.get_text());
				for(int i = 0; i < storage; i++)
				{
					myfile << storageFileName[i] << " " << int(storageROILocation[i][0]*widthMultiplier[i]) << " " << int(storageROILocation[i][1]*heightMultiplier[i]) << " " << int(storageROILocation[i][2]*widthMultiplier[i]) << " " << int(storageROILocation[i][3]*heightMultiplier[i]) << std::endl;
					std::string str2 = storageFileName[i].substr (0,storageFileName[i].length()-4);
					str2 += "_cropped.png";
					cv::Rect roi(int(storageROILocation[i][0]*widthMultiplier[i]), int(storageROILocation[i][1]*heightMultiplier[i]), int(storageROILocation[i][2]*widthMultiplier[i]) - int(storageROILocation[i][0]*widthMultiplier[i]), int(storageROILocation[i][3]*heightMultiplier[i]) - int(storageROILocation[i][1]*heightMultiplier[i]));
					cv::Mat img = cv::imread(storageFileName[i],1);
					cv::Mat image_roi = img(roi);
					imwrite(str2, image_roi);
				}
				
		//Create the cv::Mat with the ROI you need, where "image" is the cv::Mat you want to extract the ROI from
				
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("The details have been saved into the file " + text_outputFile.get_text() + " and the cropped images have been saved");

		//  		dialog.run();
			}
			else if(data == "saveSegnetMask")
			{
				std::ofstream myfile;
				myfile.open(text_outputFile.get_text());
				std::string tempFileName = storageFileName[0];
				
				for(unsigned int i = 0; i < storage; i++)
				{
					if(i == 0)
					{
						myfile << storageFileName[i] << " ID" << roiPointsForMaskPermanent[i][3] << " " << roiPointsForMaskPermanent[i][0] << " " << int(roiPointsForMaskPermanent[i][1]*widthMultiplier[i]) << " " << int(roiPointsForMaskPermanent[i][2]*heightMultiplier[i]) << " ";
					}
					else
					{	
						if(tempFileName == storageFileName[i])
						{
							if(roiPointsForMaskPermanent[i][3] == roiPointsForMaskPermanent[i-1][3])
								myfile << int(roiPointsForMaskPermanent[i][1]*widthMultiplier[i]) << " " << int(roiPointsForMaskPermanent[i][2]*heightMultiplier[i]) << " ";
							else
								myfile << " ID" << roiPointsForMaskPermanent[i][3] << " " << roiPointsForMaskPermanent[i][0] << " " << int(roiPointsForMaskPermanent[i][1]*widthMultiplier[i]) << " " << int(roiPointsForMaskPermanent[i][2]*heightMultiplier[i]) << " ";
						}
						else
						{
							myfile << std::endl;
							myfile << storageFileName[i] << " ID" << roiPointsForMaskPermanent[i][3] << " " << roiPointsForMaskPermanent[i][0] << " " << int(roiPointsForMaskPermanent[i][1]*widthMultiplier[i]) << " " << int(roiPointsForMaskPermanent[i][2]*heightMultiplier[i]) << " ";
							tempFileName = storageFileName[i];
						}
					}
				}
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("The details have been saved into the file " + text_outputFile.get_text());
				dialog.run();
							
			}
			else if(data == "saveCropMarkedMultiple")
			{
				std::ofstream myfile;
				myfile.open(text_outputFile.get_text());
				std::string tempFileName = storageFileName[0];
				
				for(unsigned int i = 0; i < storage; i++)
				{
		//			cout << storageFileName[i] << " " << storageROILocationCurrent[i][0] << " " << widthMultiplier[i] << " " << heightMultiplier[i] << endl;
					if(i == 0)
					{
						myfile << storageFileName[i] << " " << storageROILocationCurrent[i][0] << " " << int(storageROILocationCurrent[i][1]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][2]*heightMultiplier[i]) << " " << int(storageROILocationCurrent[i][3]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][4]*heightMultiplier[i]) << " ";
					}
					else
					{	
						if(tempFileName == storageFileName[i])
						{
							myfile << storageROILocationCurrent[i][0] << " " << int(storageROILocationCurrent[i][1]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][2]*heightMultiplier[i]) << " " << int(storageROILocationCurrent[i][3]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][4]*heightMultiplier[i]) << " ";
						}
						else
						{
							myfile << std::endl;
							myfile << storageFileName[i] << " " << storageROILocationCurrent[i][0] << " " << int(storageROILocationCurrent[i][1]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][2]*heightMultiplier[i]) << " " << int(storageROILocationCurrent[i][3]*widthMultiplier[i]) << " " << int(storageROILocationCurrent[i][4]*heightMultiplier[i]) << " ";
							tempFileName = storageFileName[i];
						}
					}
					std::string str2 = storageFileName[i].substr (0,storageFileName[i].length()-4);
					std::string result;
					std::ostringstream convert;
					convert << i;
					result = convert.str();
					str2 = str2 + "_cropped_";
					std::string label;
					std::ostringstream convert1;
					convert1 << storageROILocationCurrent[i][0];
					label = convert.str();
					str2 = str2 + label + "_" + result + ".png";
					cv::Rect roi(int(storageROILocationCurrent[i][1]*widthMultiplier[i]), int(storageROILocationCurrent[i][2]*heightMultiplier[i]), int(storageROILocationCurrent[i][3]*widthMultiplier[i]) - int(storageROILocationCurrent[i][1]*widthMultiplier[i]), int(storageROILocationCurrent[i][4]*heightMultiplier[i]) - int(storageROILocationCurrent[i][2]*heightMultiplier[i]));
					cv::Mat img = cv::imread(storageFileName[i],1);
					cv::Mat image_roi = img(roi);
					imwrite(str2, image_roi);
				}
				Gtk::MessageDialog dialog(*this, "Message");
		 		dialog.set_secondary_text("The details have been saved into the file " + text_outputFile.get_text());
				dialog.run();
			}
		}

	}
}