/*
                                   
  _____   ____ _   ___ _ __  _ __  
 / _ \ \ / / _` | / __| '_ \| '_ \ 
|  __/\ V / (_| || (__| |_) | |_) |
 \___| \_/ \__,_(_)___| .__/| .__/ 
                      |_|   |_|    
*/
/* Copyright (c) 2014 AVBotz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ========================================================================== */
/*                                                                                                                                                                                                                                                                          
............................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.............................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................................................................,..............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..............................................,:::,,,,,,,,,,,,,,,,..................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
........................................,:::::::::::::,,,,,,........,,,...............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
......................................:~::~~~~~~~::::::,,,,,,.....  . ..,,..............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................................:::~~=~~~~::::::::::,,,,,,,.... ..  ..,,....,.......,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.................................,,,:~~~:::,,,,,,,,,,,,,,,,,,,,,...... .  ..,,............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
................................:,::::,,..,,,,,:::::,,,,,,,,,,,,....... . . ..,,.,........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..............................::::~:......,,:::::::::::,,,,,,,.,.........     ..,,,...,.....,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
............................::~~=~......,::~~~~~~~::::::,,,,,,,...........   .. ..,..,.,.....,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
...........................,:~==:,....,,:~~~~~~~:,.,,,:~~~,,..............   . . ..,..,......,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..........................::~==~,,.,,,::~~=~,,IZNMMMMMMMMMMMMMND?:.......  .      ..,.........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.........................:~~==:::::~~~=~=ZMMNNNNMMMMMMMMMMMMMMNNNNNNMO......       ............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
........................~:~==~:::~===~7MMMNNNNNNNNMMMMMMMMMMMMMMNNNNNNMM~.. .       ..,,........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.......................,:~==~~:~~===IMMMMNDD88DDDDNNNNNNNNNNNNNNNNNNNDDDNM~ .     . . .,.........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
......................::~~==~~=~=~8MMMMMNDD88888DDNNNNNNNNNNNNNNNNNND8OODDNMZ,...  . . .,.........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
......................::~==~~~==,$MMMMMMNNDDDDDDDDNNNNNNNNNNNNNNNNNNDD88DDNNMM,.   . .  ,,........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................,:~~==~===~MMMMMMMNDDDDDDDDNNNNNNNNNNNNNNNNNNNNNNNNNNNNMMMZ.....   .,........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................,::~~~====~NMMMMNDOZ$$7$ZZ88DDNNNNNNNNNNNNNNNNNNDNDDDDNNNNMMD, .... ..,.........,,,,,.,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................::~~~=====NMMMMMDO7I??II77$ZOODDNNNNNNNNNNNNDDD8OOOOOO8DNNMMMM ... ...,..........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................::~~~~===:MMMMMMO$??+?II77777$O8DNNNNNNNNNDD8O$$77IIII$ZDNMMMM? ... ..,,..........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................::~~~=++:OMMMMNNO$7III??????I7$Z8DNNNNNNDDOZ$777I7IIIII7O8NNMMMZ.......:..........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
...................,::~~~=++=MMMMMMN8O77II??????777$O8NNNNND8O$7777777IIIII7O8NMMMMM ......:..........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
...................,::~~~=++IMMMMMNNNDZ$II????II777$Z8DNNNNDOZ777777777I7II7ODNMMMMM ......:..........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
...................:::~~==++?MMMMNNNNND8OZ$$7III??7$8DNNNNDDZ7I?????????+?7$8DNNMMMM: .....:..........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
...................::~~~~=++~DMMMNNNMMNNDDOOZ$$77$O8DNNNNNNDOZ7I?I??I?II7$O8NNNMMMMM: .....,.............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
...................,~~=~==++==MMMMNNMMMMNNND88OO88DNNNNNNNNND8Z$77II77$ZO8DNNNMMMMMM,......:.............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................=======+=++MMMNNNNNNNNNNNNNNNNNNNNNNNNNNNNDD88888DDNNNMMMMMMMMMM. .....:.............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................:+======+++:MMMMNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNMMMMMMMMM......,:..............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................,+=++=====+~~MMMNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNMMMMMMMMM=......::..............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.......................:++++++====+~+OMMNNNNNNNNNNNNNNNNNNNNNNNNNNMMMMMMMMMMNMMMMM ......,:...............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
........................:++++++=======,~DMMMNNNNNNNNNNNNNNNNNNNNNNMMMMMMMMMMMMMM~  .....,~................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..........................=+++++++======~::+DMMMNNMNNNNNNNNNNNNNNNMMMMMMMMMMMM~ . .....,~:..................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.............................=??+++=======~~~~~~,,,:?IO8DNMMMMMMMMMMNDZ7~,   . .... .,~~...,................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..............................,~??++========~~~~~~~:::::,,......     ..... .     ..,:~,.....................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.................................:+?++======~~~~~~:::::::,,,,,,,........... .....,:~:........................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................................,++??+==~~~~~:::::::,,,,,,,............,,:~~:...........................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
........................................,~+?++===~~~::::::,,,,,,,,..,,,,,::~~=,..............................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.............................................:=+++++===~~~~~:~:::~~~~~~=~~,....,,,:::::,,,...................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.........................................................,,::::~~~~~~~~~::::::::::::~:~~~~~::,...............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.............................................,,,:::,,,,,,,,,,:::~~~~========++++++????=~,,,,:,...............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
................................,,::::,,,,......,,,,::~~~===++++?????IIIIIIIII7II?~~,,,.....:,...............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................,:,,,,,,,,,,,,,,,,::::::::~~~~~~~===========~~~~::::,,,,,,,,............,:................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................:+====+++++++++++++++========~~~~~~~:::::::,,,,,,,,,,,,.................,,................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................,+====++++++++++++++========~~~~~~~:::::::,,,,,,,,......................,,..,,............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................~~=+=+=++?+++++========~~~~~~~~::::::,,,,,,,...........      ..........,,,,..............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................,==++=+???????++=======~~~~~~~:::::::,,,,,,,...........      .........,,,~:.:.,,.........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
......................==+=????II???+++======~~~~~~~:::::::,,,,,,,...........      ........,,,,=~:,:...........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
......................====+???II??I??++=====~~~~~~~:::::::,,,,,,,...........      ........,,,,?+~~:,,,........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
 .....................====????????????++~===~~~~~~~:::::::,,,,,,,,..........      ........,,,,I?==::,,..,..,...,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
. ....................~=:.....:+??I?????=~~~~~~~~~~:::::::,,,,,,,,..........      ........,,,,7I?+~~::,,..,.,..,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
....................,,........,.+???????+=~~~~~~~~~:::::::,,,,,,,,..........      ........,,,,77II++~~:,,,.....,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
...................,,.........,,~I???????=~~~~~~~~~~::::::,,,,,,,,..........      ........,,,,777II?==~:,,,,...,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..................:,..........,,~????????+~~~~~~~~~~::::::,,,,,,,,..........      ........,,,,$777II?+=~::,,..,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
  ............. ::............,,=?????????~~~~~~~~~~::::::,,,,,,,,..........      ........,,,:,I$777II?+~~~,,,.,..,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
................,.,,..........,,+?????????=~~~~~~~~~::::::,,,,,,,,..........      ........,,,:..I$777II?==~:,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..............,:,,,,..........,.??????????+~~~~~~~~~::::::,,,,,,,,..........     .........,,,:...:$777II?+=~::,,,.,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..............:,,,,,..........,.??????????+=~~~~~~~~::::::,,,,,,,,..........     .........,,,:.....:$777II?+=~:,,,..,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.............:,,:,,,...........:??????????++~~~~~~~~::::::,,,,,,,,..........     .........,,,:..,...,I777II?+=~~,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
............::,::,,,...........=??????????++~~~~~~~~::::::,,,,,,,,...........    .........,,,:.....,,.?$77II?+=~~,,,,.,.,,,,,,,,,,,,,,,,,,,,,,,~
............:,:::,,,.........,~?++????????++~~~~~~~~::::::,,,,,,,,...........    .........,,,:........,.?777II?+=~,:,,.,,,,,,,,,,,,,,,,,,,,,,,,~
...........,,,:::,,,........,,+++++??????+++=~~~~~~~::::::,,,,,,,,...........    .........,,,:......,.,.,.$77III+=~~:,,,,,,,,,,,,,,,,,,,,,,,,,,~
..........,~,::::,,,........,.+++++++????++++~~~~~~~::::::,,,,,,,,...........    .........,,,:........,.,.,=77III+=~,::,,,,,,,,,,,,,,,,,,,,,,,,~
..........~::::::,,,.........:=+++++++++?++++=~~~~~~::::::,,,,,,,,...........    .........,,,:...........,,.,=7III?+=~:,,..,,,,,,,,,,,,,,,,,,,,~
..........:::~::,.,,........,===++++++++++++++~~~~~~::::::,,,,,,,,...........    .........,,,:..........,,....=7III?+==,,,,,,,,,,,,,,,,,,,,,,,,~
.........,::~~::..,,.......,:===+++++++?++++++~~~~~~::::::,,,,,,,............    .........,,::...............,,,77III?==:,,,,,,,,,,,,,,,,,,,,,,~
.........~::~~::.,,,......,,.~===+++++++++++++=~~~~~::::::,,,,,,,...........     .........,,::..............,,,,.,77I??+=~,:,,,,,,,,,,,,,,,,,,,~
........:::~~::,,,,,......,,..===++++++++++++++~~~~:::::::,,,,,,,..........      .........,,::..............,,,.,.,+III?++::,,,,,,,,,,,,,,,,,,,~
........~::~~::.:,,,......,:..+===+++++++++++++=~~~::::::,,,,,,,,.........       .........,,:,..............,.,,,,,,?7II??+:,,,,,,,,,,,,,,,,,,,~
........::~~~~,,,,,,,....,,...,+==++++++++++++++:~:::::::,,,,,,,..........        ........,,:,............,,,,,,,,,,,.III??+~~,,,,,,,,,,,,,,,,,~
.......,::~~~~,,:,,,,...,,.....:===+++++++++++=+~::::::::,,,,,,...........        ........,,:.............,,,,,,,,,,,,,=III?+=:,,,,,,,,,,,,,,,,~
.......:::~~~~,::,,,,...,,......+===++++++++++===::::::::,,,,,,..........         ........,,,.............,,,,,,,,.,,.,.?I?I?+~~,,,,,,,,,,,,,,,~
.......~:~~~~::::,,,,,,.,,......:+==++++++++=+===~:::::::,,,,,,.........          ........,,,.............,,,,,,,,,,.,,,,.II????~,,,,,,,,,,,,,,~
......,~:~~~~::::,,,,,.,,........~===+=+++++======::::::,,,,,,..........          ........,:.............,,,,,,,,,,,,,,,,,,+????==,.,,,,,,,,,,,~
......:~:~~~~::::,,,,,.,,.........+=====++=+======::::::,,,,,,..........          .......,,:.............,,,,,,,,,,,,,,,.,,,?I???=,,,,,,,,,,,,,~
......~:~~~~~::::,,,,,,,..........:+==============~:::::,,,,,,........            ........,:.............,,,,,,,,,,,,,,,,,,,,,??I?+=,,,,,,,,,,,~
......~:~~~~~::::,,,,,,,...........:==============~:::::,,,,,,.........      . .  ........::............,,,,,,,,,,,,,,,,,,,,,,,+?II+,,,,,,,,,,,~
......~:~~~~~::::,,,,,,,............+=============~~::::,,,,,,........       . . .........:,............,,,,,,,,,,,,,,,,,,,,,,,,?III=,,,,,,,,,,~
......::~~~~~~:::,,,,,,,............,=============~~::::,,,,,,........       .   ........,,.............,,,,,,,,,,,,,,,,,,,,,,,,,,?I?=,,,,,,,,,~
.....,::~~~~~~:::,,,,,:..............,==~==========~::::,,,,,,........       .   ........,..............,,,,,,,,,,,,,,,,,,,,,,,,,,,,?~,,,,,,,,,~
.....,::~~~~~~::::,,,,,...............===~==========:::,,,,,,,........          ........,:............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....:::~~~~~~::::,,,,.................:==~======~~~~::,,,,,,,........       .  ........::............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....:::~~~~~~::::,,,,..................===~==~~==~~~:::,,,,,,........         .........:.............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....~:~~~~~~~::::,,,,...................===~=~~~=~~~:::,,,,,,..........      .........,,..,........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....::~~~~~~~::::,:,.....................~==~~~~==~~::::,,,,,..........     .........,:............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....::~~~~~~~::::,,,......................:==~~~~=~~:::,,,,,,,.......................:,...,......,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....::~~~~~~~:::::,........................==~~~==~~:::::,,,,,,.....................,:...,.,.....,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....:~~~~~~~~~::,,:..........................==~~:~::::::,,,,,,....................::............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....,~:~~~~~~:~:::...........................,==~~~~~::::::,,,.............,,.....,:............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
......~~~~~~~~~:::,.............................==~~~~~::::::,........,,,,,,,.....,:.........,..,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
......~~~=~~~~~:::................................===~::::::::,,......,,,,,,....,,:,...........,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
......:~~~=~~~:::,..................................==~~:::::::,,,...,,,,,,,...,:~............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.......~~~===~:~:....................................,==~::::,::,,,,,,,,,,....:~,............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
........~~~~::~:........................................:~=~~:,,,,..,.....,,~~,.............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.........:~=~~,............................................,===~~:::,,,,:~~~,...............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..........,,:,................................................,,~~====~~:,..,.............,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.........................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
........................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
........................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
.....................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
..................................................................................,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,~
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include "eva.hpp"
#include <csignal>
#include <stdio.h>
#include <stdlib.h>

volatile int eva::stopEVA = 0;

eva::eva(libconfig::Config *config) :
	evaState(new State)
{
	printDisclaimer();
	
	const char* mbedName;
	int mbed_print_interval;
	config->lookupValue("serial.mbed_port", mbedName);
	config->lookupValue("serial.status_print_interval", mbed_print_interval);
	bool simulation;
	config->lookupValue("eva.simulation", simulation);
	mbed = new SerialMbed(mbedName, evaState, mbed_print_interval);
	
	if (simulation)
	{
		vision = new VisionSim(config);
	}
	else
	{
		vision = new VisionNormal(config);
	}

	conf = config;
	for (int i = 0; i < NUM_TASKS; i++) {
		taskList[i] = nullptr;
	}
	// construct stuff
}

eva::~eva()
{
	for (int i = 0; i < NUM_TASKS; i++) {
		delete taskList[i];
	}
	delete mbed;
	delete vision;
	delete evaState;
}

void eva::printDisclaimer()
{
	lprintf("This program is distributed in the hope that it will be useful, but WITHOUT ANY ");
	lprintf("WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A ");
	lprintf("PARTICULAR PURPOSE.\n");
	lprintf("\n");
	lprintf("Press Control-C to quit this program. Your continued use constitutes ");
	lprintf("acceptance of these terms and conditions.\n");
	lprintf("\n");
}

void eva::runTasks()
{
	// the gate is the first task so go straight there without trying to go through some loop
	taskList[T_SEARCH] = instantiator(T_SEARCH);
	taskList[T_GATE] = instantiator(T_GATE);
	// set the current task to gate. This value will be read by the loop below.
	State::setProperty(State::missionState, T_GATE);

	// loop while a stop is not requested (for example, by Ctrl-C keystroke).
	while (!(State::getProperty(State::runState) & R_STOPPED))
	{
		
		if (taskList[State::getProperty(State::missionState)]->stage == 0)
		{
			lprintf("--> % has started @ % <--\n", TASK_NAME[State::getProperty(State::missionState)].c_str(), State::getTimeStamp());
			lastStage = 0;
		}
		else if ((taskList[State::getProperty(State::missionState)]->stage != lastStage) && !taskList[State::getProperty(State::missionState)]->pause)
		{
			lastStage = taskList[State::getProperty(State::missionState)]->stage;
			if(taskList[State::getProperty(State::missionState)]->stageName.size() >= lastStage)
			{
				lprintf("-> % has proceeded to stage % ['%'] at % <-\n", TASK_NAME[State::getProperty(State::missionState)].c_str(), lastStage, taskList[State::getProperty(State::missionState)]->stageName[lastStage-1], State::getTimeStamp());
			}
			else
			{
				lprintf("-> % has proceeded to stage % at % <-\n", TASK_NAME[State::getProperty(State::missionState)].c_str(), lastStage, State::getTimeStamp());
			}
		}
		
		// wait for any picture or something else that the current task needs
		waitForHardwareRequest(taskList[State::getProperty(State::missionState)]->hardware_request);
		// handles task pauses
		if (taskList[State::getProperty(State::missionState)]->pause)
		{
			static long long pauseTime = 0;
			// sets initial pause time
			if (pauseTime == 0)
			{
				pauseTime = State::getTimeStamp();
			}
			// decrements pausetime each cycle
			else
			{
				long long currentTime = State::getTimeStamp();
				taskList[State::getProperty(State::missionState)]->pause -= currentTime - pauseTime;
				pauseTime = currentTime;
				// done pausing
				if (taskList[State::getProperty(State::missionState)]->pause <= 0)
				{
					taskList[State::getProperty(State::missionState)]->pause = 0;
					pauseTime = 0;
				}
			}
			// task waiting for nothing, so make it wait for something
			if (!taskList[State::getProperty(State::missionState)]->hardware_request && pauseTime != 0)
			{
				Timer::sleep(5);
			}
			lprintf("pausing for %\n", taskList[State::getProperty(State::missionState)]->pause);
		}
		else
		{
			taskList[State::getProperty(State::missionState)]->action();
		}
		
		// save the frame and the processed frame if it is supposed to, not on the search task (since search itself doesnt process the images),
		// the current task is a vision task, and it has skipped the appropriate number of frames
		if ((vision->snapshotEnable) &&
			(taskList[State::getProperty(State::missionState)]->hardware_request & (CAM_FRONT | CAM_DOWN)) &&
			((vision->skippedFrames++ == vision->snapshotTime) ||
			taskList[State::getProperty(State::missionState)]->forceSaveFrame))
		{
			cv::Mat processed;
			if(taskList[State::getProperty(State::missionState)]->forceSaveFrame)
			{
				processed = (dynamic_cast<VisionTask*>(taskList[State::getProperty(State::missionState)]))->processedImage;
				taskList[State::getProperty(State::missionState)]->forceSaveFrame = false;
			}
			else if ((State::getProperty(State::missionState) == T_SEARCH) || taskList[State::getProperty(State::missionState)]->pause)
			{
				processed.create(640, 360, CV_8UC3);
				memset(processed.ptr(), 255, 640*360*3);
			}
			else
			{
				processed = (dynamic_cast<VisionTask*>(taskList[State::getProperty(State::missionState)]))->processedImage;
			}
			// save only the correct unprocessed image since the down camera probably wont have any action when the buoy is in front of the sub for example and vice versa
			// always save the processed image we can assume its a visiontask if it requested an image
			// the reason we use dynamic_cast is because visiontask is an abstract class so we can't use a normal cast for it
			vision->saveFrame(taskList[State::getProperty(State::missionState)]->hardware_request & (CAM_FRONT | CAM_DOWN), processed);
			// reset the frame counter
			vision->skippedFrames = 0;
			long long currentTime = State::getTimeStamp();
			lprintf("@ % FPS\n", ((float)vision->snapshotTime) / (currentTime - lastFrameSaveTime) * 1000);
			lastFrameSaveTime = currentTime;
		}
		
		// clean up
		if (taskList[State::getProperty(State::missionState)]->isCompleted)
		{
			if (State::getProperty(State::missionState) == T_SEARCH)
			{
				
				// search returned
				if (((Search*)taskList[T_SEARCH])->taskFound == T_NUL)
				{
					// didn't find anything
				}
				else
				{
					// set the mission state to what was found and clear the array of the tasks not found
					State::setProperty(State::missionState, ((Search*)taskList[T_SEARCH])->taskFound);
					for (int i = 0; i < NUM_TASKS; i++)
					{
						if (i != T_SEARCH && i != State::getProperty(State::missionState))
						{
							delete taskList[i];
							taskList[i] = nullptr;
						}
					}
				}
			}
			else
			{
				
				// some other task just completed
				delete taskList[State::getProperty(State::missionState)];
				taskList[State::getProperty(State::missionState)] = nullptr;
				
				std::string possibleNextTaskName;
				char numberString[3];
				for (int i = 0; ;i++) {
					// build the config index step by step
					std::string configTaskIndex(TASK_NAME[State::getProperty(State::missionState)]);
					if (State::getProperty(State::missionState) == T_PATH)
					{
						// because after we follow a path, the next task is dependent on what came right before the path
						configTaskIndex += "." + TASK_NAME[State::getProperty(State::lastTask)];
					}
					
					sprintf(numberString, "%d", i);
					configTaskIndex += ".next_task.[";
					configTaskIndex += numberString;
					configTaskIndex += ']';
					// if lookupValue returnes false it means it didnt find anything so its done iterating through the possibilities
					if (!conf->lookupValue(configTaskIndex, possibleNextTaskName))
					{
						break;
					}
					// otherwise initialize this task
					TASK_ID nextTaskId = identify(possibleNextTaskName);
					// if this was a valid task, then instantiate it
					if (nextTaskId != T_NUL)
					{
						taskList[(int)nextTaskId] = instantiator(nextTaskId);
					}
				}
				
				State::setProperty(State::lastTask, State::getProperty(State::missionState));
				State::setProperty(State::missionState, T_SEARCH);
			}
		}
		
		// restarts to the gate when the kill switch is reinserted
		if (waitForWall_e()) {
			taskList[T_SEARCH] = instantiator(T_SEARCH);
			taskList[T_GATE] = instantiator(T_GATE);
			State::setProperty(State::missionState, T_GATE);
		}
	}
}

void eva::waitForHardwareRequest(char hardware)
{
	// if a picture was requested, then wait for it
	if (hardware & CAM_FRONT || hardware & CAM_DOWN)
	{
		vision->requestImage = hardware & (CAM_FRONT | CAM_DOWN);
		vision->waitForImage();
		
		if (hardware & CAM_FRONT) {
			State::setImage(State::imageFront, vision->i_front);
		}
		if (hardware & CAM_DOWN) {
			State::setImage(State::imageDown, vision->i_down);
		}
	}
}


/*
 * Returns a pointer to an instance of the Task specified by t.
 * If t is unknown, returns nullptr.
 */
Task* eva::instantiator(TASK_ID t)
{
	switch (t)
	{
		case T_SEARCH:
			return new Search(conf, taskList);
		case T_GATE:
			return new Gate(conf);
		case T_HYDROPHONE:
			return new Hydrophone(conf);
		case T_BUOY:
			return new Buoy(conf);
		case T_DROPPER:
			return new Dropper(conf);
		case T_LGATE:
			return new L_gate(conf);
		case T_PATH:
			return new Path(conf);
		case T_TORPEDO:
			return new Torpedo(conf);
		default:
			lprintf("eva::instantiator() doesn't know about TASKID %.\n", (int)t);
			return nullptr;
	}
}

/*
 * identify the id of a task based on its name
 */
TASK_ID eva::identify(std::string name)
{
	if (name == TASK_NAME[T_SEARCH])
		return T_SEARCH;
	if (name == TASK_NAME[T_GATE])
		return T_GATE;
	if (name == TASK_NAME[T_HYDROPHONE])
		return T_HYDROPHONE;
	if (name == TASK_NAME[T_BUOY])
		return T_BUOY;
	if (name == TASK_NAME[T_DROPPER])
		return T_DROPPER;
	if (name == TASK_NAME[T_LGATE])
		return T_LGATE;
	if (name == TASK_NAME[T_PATH])
		return T_PATH;
	if (name == TASK_NAME[T_TORPEDO])
		return T_TORPEDO;
	
	lprintf("eva::identify() doesn't know about TASK_NAME %\n", name.c_str());
	return T_NUL;
}

	

/*
 * This function handles Ctrl-C. It is called when Ctrl-C is pressed.
 */
void wall_e(int sig)
{
	if (sig == SIGINT)
	{
		// basically does runState |= R_STOPPED
		State::setProperty(State::runState, R_STOPPED | State::getProperty(State::runState));
		lprintf("\n--------------------------------------------------------------------------------\n");
		lprintf("SIGINT received\n");
		exit(0);
	}
}

/*
 * If the killswitch is in, does nothing.
 * If not, waitForWall_e() will wait for the killswitch to be put in
 * and reset the list of completed tasks. It also resets the mbed.
 */
int eva::waitForWall_e()
{
	if (!(State::getProperty(State::runState) & R_KILLED)) return 0;
	for (int i = 0; i < NUM_TASKS; i++)
	{
		delete taskList[i];
		taskList[i] = nullptr;
	}
	
	State::setProperty(State::desiredPower, 100);
	State::setProperty(State::desiredHeading, 0);
	State::setProperty(State::desiredDepth, 0);
	
	lprintf("Wall-e has departed\n");

	while (State::getProperty(State::runState) & R_KILLED) 
	{
		Timer::sleep(5);
		if (State::getProperty(State::runState) & R_STOPPED) return 0;
	}
	lprintf("Wall-e has arrived\n");
	
	return 1;
}


int main (int argc, char* argv[])
{
	// open the log
	lopen();

	// specify handleSignal() as the handler for the signal SIGINT
	signal(SIGINT, wall_e);
	
	// if there are not enough arguments passed in at the terminal prompt
	if (argc != 2)
	{
		lprintf("No configuration file; you're stupid.\n");
		// quit the program; we don't have enough information to run
		exit(0);
	}
	
	// make a new libconfig object, which will parse the configuration file
	// passed in at the command prompt
	libconfig::Config conf;
	// tell libconfig to read the filepath specified in the first argument
	// note: argv[0] is the name of the program.
	conf.readFile(argv[1]);
	
	lprintf("Using configuration file '%'\n", argv[1]);
	
	// initialize eva object pointer and give it a pointer to the libconfig
	// object so eva object knows what we want to do
	barracuda = new eva(&conf);

	lprintf("Barracuda has probably started successfully.\n");

	barracuda->waitForWall_e();
	try
	{
		barracuda->runTasks();
	}
	catch (cv::Exception &e)
	{
		lprintf("UH-OH UNCAUGHT EXCEPTION\n");
		lprintf("Type: cv::Exception\tDescription: %\n", e.msg.c_str());
		exit(0);
	}
	catch (std::exception &e)
	{
		lprintf("UH-OH UNCAUGHT EXCEPTION\n");
		lprintf("Type: std::exception\tDescription: %\n", e.what());
		exit(0);
	}
	catch (int &e)
	{
		lprintf("UH-OH UNCAUGHT EXCEPTION\n");
		lprintf("Type: int\tValue: %\n", e);
		exit(0);
	}
	catch (...)
	{
		lprintf("UH-OH UNCAUGHT EXCEPTION\n");
		lprintf("UH-OH UNKNOWN TYPE\n");
		exit(0);
	}
	
	lprintf("Barracuda has probably exited.\n");
	return 0;
}
