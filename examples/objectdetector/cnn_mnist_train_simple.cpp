#include "od/detectors/global2D/training/ConvTrainer.h"
#include "od/detectors/global2D/detection/ConvClassification.h"

int main(int argc, char **argv)
{
	od::g2d::ConvTrainer mnist_trainer("", "");
	mnist_trainer.setSolverLocation(argv[1]);
	mnist_trainer.startTraining();
	return 0;
}
	
