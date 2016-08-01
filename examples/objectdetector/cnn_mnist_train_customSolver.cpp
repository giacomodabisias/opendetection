#include "od/detectors/global2D/training/ConvTrainer.h"
#include "od/detectors/global2D/detection/ConvClassification.h"

int main(int argc, char *argv[])
{
	argc = 1;
	od::g2d::ConvTrainer mnist_trainer("","");
	mnist_trainer.setSolverProperties(argc, argv);
	mnist_trainer.startTraining();
	return 0;
}
