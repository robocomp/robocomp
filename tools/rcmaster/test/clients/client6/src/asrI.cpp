/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "asrI.h"

ASRI::ASRI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


ASRI::~ASRI()
{
}

void ASRI::listenWav(const string  &path, const Ice::Current&)
{
	worker->listenWav(path);
}

void ASRI::listenVector(const audioVector  &audio, const Ice::Current&)
{
	worker->listenVector(audio);
}

void ASRI::resetPhraseBuffer(const Ice::Current&)
{
	worker->resetPhraseBuffer();
}

string ASRI::getLastPhrase(const Ice::Current&)
{
	return worker->getLastPhrase();
}

bool ASRI::phraseAvailable(const Ice::Current&)
{
	return worker->phraseAvailable();
}






