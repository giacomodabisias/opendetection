#include "od/detectors/global2D/training/Network.h"
#include <iostream>

struct Node {
	Glib::ustring data;
	Node* nextLayer;
};


void initializeLayer(struct Node *headLayer, Glib::ustring data)
{
	headLayer->data = data;
	headLayer->nextLayer = NULL;
}

void appendLayer(struct Node *headLayer, Glib::ustring data)
{
	Node *newLayer = new Node;
	newLayer->data = data;
	newLayer->nextLayer = NULL;

	Node *currentLayer = headLayer;
	while(currentLayer)
	{
		if(currentLayer->nextLayer == NULL)
		{
			currentLayer->nextLayer = newLayer;
			return;
		}
		currentLayer = currentLayer->nextLayer;
	}
}


bool deleteLayer(struct Node **headLayer, Node *deleteLayer) 
{
	Node *currentLayer = *headLayer;
	if(deleteLayer == *headLayer)
	{
		*headLayer = currentLayer->nextLayer;
		delete deleteLayer;
		return true;
	}
	
	while(currentLayer)
	{
		if(currentLayer->nextLayer == deleteLayer)
		{
			currentLayer->nextLayer = deleteLayer->nextLayer;
			delete deleteLayer;
			return true;
		}
		currentLayer = currentLayer->nextLayer;
	}
	return false;
}


struct Node *searchLayer(struct Node *headLayer, Glib::ustring data)
	{
	Node *currentLayer = headLayer;
	while(currentLayer)
	{
		if(currentLayer->data == data) 
		{
			return currentLayer;
		}
		currentLayer = currentLayer->nextLayer;
	}
	std::cout << "No Layer " << data << " in the CNN." << std::endl;
}

Glib::ustring displayCNN(struct Node *headLayer)
{
	Node *cnn = headLayer;
	Glib::ustring fullCnnLayerMatter = "";
	while(cnn)
	{
//		std::cout << cnn->data << std::endl;
		fullCnnLayerMatter += cnn->data;
		cnn = cnn->nextLayer;
	}
//	std::cout << std::endl;
//	std::cout << std::endl;
	return fullCnnLayerMatter;
}
