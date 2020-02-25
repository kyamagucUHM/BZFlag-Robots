/* bzflag
 * Copyright (c) 1993-2018 Tim Riker
 *
 * This package is free software;  you can redistribute it and/or
 * modify it under the terms of the license found in the file
 * named COPYING that should have accompanied this file.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

 /*
  *
  */

#ifndef BZF_ROBOT_PLAYER_H
#define BZF_ROBOT_PLAYER_H

#include "common.h"

  /* system interface headers */
#include <vector>

/* interface header */
#include "LocalPlayer.h"

/* local interface headers */
#include "Region.h"
#include "RegionPriorityQueue.h"
#include "ServerLink.h"

//A* algorithm
#include "Astar.h"

class RobotPlayer : public LocalPlayer
{
public:
    RobotPlayer(const PlayerId&,
        const char* name, ServerLink*,
        const char* _motto);

    float       getTargetPriority(const Player*) const;
    const Player* getTarget() const;
    void        setTarget(const Player*);
    static void     setObstacleList(std::vector<BzfRegion*>*);

    void        restart(const float* pos, float azimuth);
    void        explodeTank();

private:
    int getCenterOfMass(float hoodSize, float teamCom[3]);
    int getAlignment(float hoodSize, float teamAlign[3], float* avgAzi);
    int getSeparation(float hoodSize, float teamSep[3]);
    void RobotPlayer::getMyBase(TeamColor teamColor, float location[3]);
    bool RobotPlayer::haveFlag(void);
	void aStarSearch(float grid[ROW][COL], Pair src, Pair dest);
    void RobotPlayer::findFlag(float location[3]);
    Player* findPlayer(PlayerId id);
    void        doUpdate(float dt);
    void        doUpdateMotion(float dt);
    BzfRegion* findRegion(const float p[2], float nearest[2]) const;
    float       getRegionExitPoint(
        const float p1[2], const float p2[2],
        const float a[2], const float targetPoint[2],
        float mid[2], float& priority);
    void       findPath(RegionPriorityQueue& queue,
        BzfRegion* region, BzfRegion* targetRegion,
        const float targetPoint[2], int mailbox);

    void       projectPosition(const Player* targ, const float t, float& x, float& y, float& z) const;
    void       getProjectedPosition(const Player* targ, float* projpos) const;
    static const float CohesionVector;
    static const float SeparationVector;
    static const float AlignVector;
    static const float PathVector;

private:
    const Player* target;
    std::vector<RegionPoint>    path;
    int         pathIndex;
    float       timerForShot;
    bool        drivingForward;
    static std::vector<BzfRegion*>* obstacleList;
};

#endif // BZF_ROBOT_PLAYER_H

// Local Variables: ***
// mode: C++ ***
// tab-width: 4 ***
// c-basic-offset: 4 ***
// indent-tabs-mode: nil ***
// End: ***
// ex: shiftwidth=4 tabstop=4
