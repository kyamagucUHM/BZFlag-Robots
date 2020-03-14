/*
 * Defines the classes used for decision trees.
 *
 * Part of the Artificial Intelligence for Games system.
 *
 * Copyright (c) Ian Millington 2003-2006. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */
#include <exception>
#include "dectree.h"
#include "playing.h" // needed for controlPanel
#define TRACE_DECTREE

namespace aicore
{

    
	DecisionTreeNode* DecisionPtr::makeDecision(RobotPlayer* bot, float dt)
    {
        // Choose a branch based on the getBranch method
        if ( getBranch(bot, dt) ) {
            // Make sure its not null before recursing.
            if (trueBranch == NULL) {
#ifdef TRACE_DECTREE
				controlPanel->addMessage("NULL true branch");
				throw "NULL true branch";
#endif
				return NULL;
			}
            else return trueBranch->makeDecision(bot, dt);
        } else {
            // Make sure its not null before recursing.
            if (falseBranch == NULL) {
#ifdef TRACE_DECTREE
				controlPanel->addMessage("NULL false branch");
				throw "NULL false branch";
#endif
				return NULL;
			}
            else return falseBranch->makeDecision(bot, dt);
        }
    }

	bool DecisionPtr::getBranch(RobotPlayer* bot, float dt)
	{
		if (decFuncPtr == NULL) {
#ifdef TRACE_DECTREE
			controlPanel->addMessage("NULL decFunctPtr");
			throw "NULL decFunctPtr";
#endif
		}
        return (bot->*decFuncPtr)(dt);
	}

	void DecisionPtr::runDecisionTree(DecisionPtr decTree[], RobotPlayer* bot, float dt)
	{
		// Find the decision
		DecisionTreeNode *node = decTree[0].makeDecision(bot, dt);
		void (RobotPlayer::*actFuncPtr)(float dt) = ((ActionPtr*)node)->actFuncPtr;
		if (actFuncPtr == NULL) {
#ifdef TRACE_DECTREE
			controlPanel->addMessage("NULL action function pointer in decision tree.");
			throw "NULL action function pointer in decision tree.";
#endif // TRACE_DECTREE
		}
		else {
			(bot->*actFuncPtr)(dt);
		}
	}

	// Set up the trees
	void DecisionTrees::init()
	{
        /*//updateMotion Decision Tree
		doUpdateMotionDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
		doUpdateMotionDecisions[0].trueBranch = &doUpdateMotionDecisions[1];
		doUpdateMotionDecisions[0].falseBranch = &doUpdateMotionActions[0];

		doUpdateMotionDecisions[1].decFuncPtr = &RobotPlayer::beingShot;
		doUpdateMotionDecisions[1].trueBranch = &doUpdateMotionActions[1];
		doUpdateMotionDecisions[1].falseBranch = &doUpdateMotionActions[2];

        //updateMotionActions
        doUpdateMotionActions[0].actFuncPtr = &RobotPlayer::doNothing;
        doUpdateMotionActions[1].actFuncPtr = &RobotPlayer::evade;
        doUpdateMotionActions[2].actFuncPtr = &RobotPlayer::doAStar;*/

        //doUpdateFlag Decision Tree
		doUpdateFlagDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
		doUpdateFlagDecisions[0].trueBranch = &doUpdateFlagDecisions[1];
		doUpdateFlagDecisions[0].falseBranch = &doUpdateFlagActions[0];

		doUpdateFlagDecisions[1].decFuncPtr = &RobotPlayer::gotFlag;
		doUpdateFlagDecisions[1].trueBranch = &doUpdateFlagDecisions[2];
		doUpdateFlagDecisions[1].falseBranch = &doUpdateFlagActions[0];

		doUpdateFlagDecisions[2].decFuncPtr = &RobotPlayer::isSticky;
		doUpdateFlagDecisions[2].trueBranch = &doUpdateFlagActions[0];
		doUpdateFlagDecisions[2].falseBranch = &doUpdateFlagDecisions[3];

		doUpdateFlagDecisions[3].decFuncPtr = &RobotPlayer::isTeam;
		doUpdateFlagDecisions[3].trueBranch = &doUpdateFlagDecisions[4];
		doUpdateFlagDecisions[3].falseBranch = &doUpdateFlagActions[1];

        doUpdateFlagDecisions[4].decFuncPtr = &RobotPlayer::isMyTeam;
        doUpdateFlagDecisions[4].trueBranch = &doUpdateFlagActions[1];
        doUpdateFlagDecisions[4].falseBranch = &doUpdateFlagActions[0];

        //doUpdateFlag Actions
		doUpdateFlagActions[0].actFuncPtr = &RobotPlayer::doNothing;
		doUpdateFlagActions[1].actFuncPtr = &RobotPlayer::dropFlag;

        //doUpdateShoot Decision Tree
        doUpdateShootDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
        doUpdateShootDecisions[0].trueBranch = &doUpdateShootDecisions[1];
        doUpdateShootDecisions[0].falseBranch = &doUpdateShootActions[0];

        doUpdateShootDecisions[1].decFuncPtr = &RobotPlayer::fireReady;
        doUpdateShootDecisions[1].trueBranch = &doUpdateShootDecisions[2];
        doUpdateShootDecisions[1].falseBranch = &doUpdateShootActions[0];

        doUpdateShootDecisions[2].decFuncPtr = &RobotPlayer::timerDone;
        doUpdateShootDecisions[2].trueBranch = &doUpdateShootDecisions[3];
        doUpdateShootDecisions[2].falseBranch = &doUpdateShootActions[0];

        doUpdateShootDecisions[3].decFuncPtr = &RobotPlayer::willMiss;
        doUpdateShootDecisions[3].trueBranch = &doUpdateFlagDecisions[4];
        doUpdateShootDecisions[3].falseBranch = &doUpdateFlagActions[0];

        doUpdateShootDecisions[4].decFuncPtr = &RobotPlayer::isBuilding;
        doUpdateShootDecisions[4].trueBranch = &doUpdateShootActions[0];
        doUpdateShootDecisions[4].falseBranch = &doUpdateShootDecisions[5];

        doUpdateShootDecisions[5].decFuncPtr = &RobotPlayer::isTeammate;
        doUpdateShootDecisions[5].trueBranch = &doUpdateShootActions[1];
        doUpdateShootDecisions[5].falseBranch = &doUpdateShootActions[2];

        //doUpdateShoot Actions
        doUpdateShootActions[0].actFuncPtr = &RobotPlayer::doNothing;
        doUpdateShootActions[1].actFuncPtr = &RobotPlayer::setShotTimer;
        doUpdateShootActions[2].actFuncPtr = &RobotPlayer::fireAndReset;

	}

	DecisionPtr DecisionTrees::doUpdateMotionDecisions[2];
	ActionPtr DecisionTrees::doUpdateMotionActions[3];

    DecisionPtr DecisionTrees::doUpdateFlagDecisions[5];
    ActionPtr DecisionTrees::doUpdateFlagActions[2];

    DecisionPtr DecisionTrees::doUpdateShootDecisions[6];
    ActionPtr DecisionTrees::doUpdateShootActions[3];


}; // end of namespace
