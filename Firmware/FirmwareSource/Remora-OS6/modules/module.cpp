#include "module.h"

#include <cstdio>

Module::Module()
{
	this->counter = 0;
	this->updateCount = 1;
}


Module::Module(int32_t threadFreq, int32_t slowUpdateFreq) :
	threadFreq(threadFreq),
	slowUpdateFreq(slowUpdateFreq)
{
	this->counter = 0;
	this->updateCount = this->threadFreq / this->slowUpdateFreq;
}

Module::~Module(){}


void Module::runModule()
{
	++this->counter;

	if (this->counter >= this->updateCount)
	{
		this->slowUpdate();
		this->counter = 0;
	}

	this->update();
}


void Module::update(){}
void Module::slowUpdate(){}
void Module::configure(){}
