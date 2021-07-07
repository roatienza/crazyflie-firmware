/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie controlfirmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program isfree software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public Licensefor more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Modifiedfrom wall_follower.c
 * Rowel Atienza Copyright 2021
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <onnx.h>
#include <math.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include "usec_time.h"

#include "policy.h"
#include <sys/resource.h>

#define DEBUG_MODULE "FLYCONTROL"

static void
setAbsoluteSetpoint(setpoint_t * setpoint,float x,float y,float z,float yaw)
{
    bool absolute = false;
	setpoint->mode.z = modeAbs;
	setpoint->position.z = z;

    /*setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    */

    if(absolute){
	    setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = yaw;

	    setpoint->mode.x = modeAbs;
	    setpoint->position.x = x;
	    setpoint->mode.y = modeAbs;
	    setpoint->position.y = y;
    }else{
	    setpoint->mode.yaw = modeVelocity;
	    setpoint->attitudeRate.yaw = yaw;

	    setpoint->mode.x = modeVelocity;
	    setpoint->mode.y = modeVelocity;
	    setpoint->velocity.x = x;
	    setpoint->velocity.y = y;
	    setpoint->velocity_body = true;
    }
}

//States
typedef enum {
	idle,
	lowUnlock,
	unlocked,
	stopping
}		StateOuterLoop;

StateOuterLoop	stateOuterLoop = idle;

/*
 * Thresholdsfor the unlocking procedure of the top sensor of the
 * multiranger
 */
static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

/* Handling the height setpoint */
static const float spHeight = 0.5f;
static const uint16_t radius = 300;

float		cmdX = 0.0f;
float		cmdY = 0.0f;
float		cmdZ = 0.0f;
float		cmdAngDeg = 0.0f;
bool        desiredHeight = false;

#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)

void
appMain()
{
	/* Getting Logging IDs of the multiranger */
    DEBUG_PRINT("Start free heap: %d bytes\n", xPortGetFreeHeapSize());
    HeapStats_t pxHeapStats;
    vPortGetHeapStats( &pxHeapStats );
    DEBUG_PRINT("Avail heap space: %d bytes\n", pxHeapStats.xAvailableHeapSpaceInBytes);
    DEBUG_PRINT("Min free: %d bytes\n", pxHeapStats.xMinimumEverFreeBytesRemaining);
	logVarId_t	idUp = logGetVarId("range", "up");
	logVarId_t	idLeft = logGetVarId("range", "left");
	logVarId_t	idRight = logGetVarId("range", "right");
	logVarId_t	idFront = logGetVarId("range", "front");
	logVarId_t	idBack = logGetVarId("range", "back");

	/* Getting the Logging IDs of the state estimates */
	/*logVarId_t	idStabilizerYaw = logGetVarId("stabilizer", "yaw");*/
	logVarId_t	idHeightEstimate = logGetVarId("stateEstimate", "z");

	/* Getting Param IDs of the deck driver initialization */
	paramVarId_t	idPositioningDeck = paramGetVarId("deck", "bcFlow2");
	paramVarId_t	idMultiranger = paramGetVarId("deck", "bcMultiranger");

	setpoint_t	setpoint;
	//state_t		current_state;



	/* Check if decks are properly mounted */
	uint8_t		positioningInit = paramGetUint(idPositioningDeck);
	uint8_t		multirangerInit = paramGetUint(idMultiranger);
	uint16_t	up = 0;
	float		heightEstimate = 0;
	float		frontRange = 0;
	float		backRange = 0;
	float		leftRange = 0;
	float       rightRange = 0;
    float       state[] = {frontRange, leftRange, backRange, rightRange};
	/*float		estYawDeg = logGetFloat(idStabilizerYaw);*/
    uint16_t	up_o = 0;
	float		cmdHeight = 0;

    float *py = 0;
    struct actor act;
    alloc_actor_ctx(&act); 
    if(act.ctx){
        act.input = onnx_tensor_search(act.ctx, "input");
        act.output = onnx_tensor_search(act.ctx, "output");
    }else{
        act.input = 0;
        act.output = 0;
    }
	vTaskDelay(M2T(3000));
    //const int window = 3;
    //const int state_len = sizeof(state)/sizeof(state[0]);
    //float avg[state_len][window];
    //int ci = 0;
	DEBUG_PRINT("Waiting for activation ...\n");

	while (1) {
         //uint64_t timeStart = usecTimestamp();
		 vTaskDelay(M2T(10));
         //struct rusage memory;
         //getrusage(RUSAGE_SELF, &memory);
		/*
		 * Get the upper range sensor value(usedfor startup and
		 * stopping)
		 */
		up = logGetUint(idUp);

		/* Get Height estimate */
		heightEstimate = logGetFloat(idHeightEstimate);

		/* Get all multiranger values */
        const float factor = 1000.0f;
		frontRange = (float)logGetUint(idFront) / factor;
		backRange = (float)logGetUint(idBack) / factor;
		leftRange = (float)logGetUint(idLeft) / factor;
		rightRange = (float)logGetUint(idRight) / factor;
        state[0] = frontRange;
        state[1] = leftRange;
        state[2] = backRange;
        state[3] = rightRange;
        //for(int i=0; i<state_len; i++){
        //    avg[i][ci] = state[i];
        //}
        /*
        ci += 1;
        ci = ci % window;
        float sum = 0.0f;
        for(int i=0; i<state_len; i++){
            sum = 0.0f;
            for(int j=0; j<window; j++){
                sum += avg[i][j];
            }
            sum /= window;
            state[i] = sum;
        }
        */

        //DEBUG_PRINT("state:[%f, %f, %f, %f], height: %f\n", (double)state[0], (double)state[1], (double)state[2], (double)state[3], (double)heightEstimate);
        //uint64_t timeBeforeExec = usecTimestamp();
        if(act.input){
            onnx_tensor_apply(act.input, (void *)state, sizeof(state));
            onnx_run(act.ctx);
            py = act.output->datas;
            //DEBUG_PRINT("acton:[%f, %f, %f]\n", (double)py[0], (double)py[1], (double)py[2]);
        }else{
            py = 0;
        }
        //uint64_t timeAfterExec = usecTimestamp();


		/*
		 * If the crazyflie is unlocked by the hand, continue with
		 * state machine
		 */
		if (stateOuterLoop == unlocked) {
			/* Get the heading and convert it to rad */
			/*estYawDeg = logGetFloat(idStabilizerYaw);*/
			/*estYawRad = estYawDeg * (float)M_PI / 180.0f;*/

			/* Adjust height based on up ranger input */
			up_o = radius - MIN(up, radius);
			cmdHeight = spHeight - up_o / factor;

			cmdX = 0.0f;
			cmdY = 0.0f;
			cmdZ = cmdHeight;
			cmdAngDeg = 0.0f;

			/*
			 * Only go to the state machine if the crazyflie has
			 * reached a certain height
			 */
			if ( (desiredHeight || (heightEstimate > spHeight - 0.1f)) && py) {
				//cmdAngDeg = cmdAngWRad * 180.0f / (float)M_PI;
				/*DEBUG_PRINT("direction: %d, estYawRad=%f, sideRange=%f,frontRange=%f\n", direction, (double)estYawRad, (double)sideRange, (double)frontRange);
                 */
                const float delta = 0.01f;
                const float angle = 90.0f;
                if(py[0] > py[1]){
                    if(py[0] > py[2]){
                        //left
			            cmdAngDeg = angle;
                    }else{
                        //fwd
			            cmdX = delta;
                    }
                }else if(py[1] > py[2]){
                    //right
			        cmdAngDeg = -angle;
                }else{
                    //fwd
			        cmdX = delta;
                }
                desiredHeight = true;
			}
			/*
			 * Turn velocity commands into setpoints and send it
			 * to the commander
			 */
            if(desiredHeight){
                cmdZ = cmdHeight;
            }
            setAbsoluteSetpoint(&setpoint, cmdX, cmdY, cmdZ, cmdAngDeg);
			commanderSetSetpoint(&setpoint, 3);
            
            //uint64_t timeDone = usecTimestamp();

            //uint64_t execTime = (timeAfterExec - timeBeforeExec);
            //uint64_t loopTime = (timeDone - timeStart);
            //DEBUG_PRINT("execTime:%llu usec, loopTime: %llu usec\n", execTime, loopTime);
            //DEBUG_PRINT("Runtime free heap: %d bytes\n", xPortGetFreeHeapSize());
            //DEBUG_PRINT("%ld\n", memory.ru_ixrss);
			//commanderGetSetpoint(&setpoint, &current_state);
			//DEBUG_PRINT("x: %f, yt: %f, yaw: %f\n", (double)current_state.position.x, (double)current_state.position.y, (double)current_state.attitude.yaw);

			/* Handling stopping with hand above the crazyflie */
			if (cmdHeight < spHeight - 0.2f) {
				stateOuterLoop = stopping;
				DEBUG_PRINT("Stopping....\n");
			    cmdX = 0.0f;
			    cmdY = 0.0f;
			    cmdAngDeg = 0.0f;
		        while( (double)(heightEstimate = logGetFloat(idHeightEstimate)) > 0.05 ){
                    cmdHeight = heightEstimate - 0.1f;
			        setAbsoluteSetpoint(&setpoint, cmdX, cmdY, cmdHeight, cmdAngDeg);
			        commanderSetSetpoint(&setpoint, 3);
		            vTaskDelay(M2T(10));
                }
                desiredHeight = false;
			}
		} else {

			/* Handeling locking and unlocking */
			if (stateOuterLoop == stopping && up > stoppedTh) {
				DEBUG_PRINT("%i", up);
				stateOuterLoop = idle;
				DEBUG_PRINT("S\n");
			}
			/*
			 * If the up multiranger is activatedfor thefirst
			 * time, prepare to be unlocked
			 */
			if (up < unlockThLow && stateOuterLoop == idle && up > 0.001f) {
				DEBUG_PRINT("Waitingfor hand to be removed!\n");
				stateOuterLoop = lowUnlock;
			}
			/*
			 * Unlock CF if hand above is removed, and if the
			 * positioningdeckand multiranger deck is initalized.
			 */
			if (up > unlockThHigh && stateOuterLoop == lowUnlock && positioningInit && multirangerInit) {
				DEBUG_PRINT("Unlocked!\n");
				stateOuterLoop = unlocked;
			}
			/* Stop the crazyflie with idle or stopping state */
			if (stateOuterLoop == idle || stateOuterLoop == stopping) {
				memset(&setpoint, 0, sizeof(setpoint_t));
				commanderSetSetpoint(&setpoint, 3);
			}
		}
	}
    free_actor_ctx(&act);
}

PARAM_GROUP_START(app)
PARAM_GROUP_STOP(app)
LOG_GROUP_START(app)
LOG_ADD(LOG_FLOAT, cmdX, &cmdX)
LOG_ADD(LOG_FLOAT, cmdY, &cmdY)
LOG_ADD(LOG_UINT8, stateOuterLoop, &stateOuterLoop)
LOG_GROUP_STOP(app)
