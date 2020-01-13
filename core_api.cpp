/* 046267 Computer Architecture - Winter 2019/20 - HW #4 */

#include "core_api.h"
#include "sim_api.h"
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <stdint.h>

typedef enum {ADD, SUB} op;
typedef enum {RUNNING, FINISHED} threadState;
using std::vector;

class thread{
public:
    int id;
    int isReady;
    tcontext regsTable;
    uint32_t pc;
    Instruction instDest;
    thread(int id, uint32_t init_pc):id(id),isReady(0), pc(init_pc){} //TODO: make sure regsTable doesn't need initialization

    // look what is the next instruction and call the relevant function and update the return value in memory update the pc
    threadState execute(int currCycle){
        int opc;
        SIM_MemInstRead(pc,  instDest, id);
        pc = pc + 4; //TODO: make sure the pc is the correct way
        opc = instDest.opcode;
        switch (opc) {
            case CMD_NOP: // NOP
                break;
            case CMD_SUB:
            case CMD_SUBI:
                addOrSubOrI(instDest.dst_index, instDest.src1_index, instDest.src2_index_imm,SUB,instDest.isSrc2Imm);
                break;
            case CMD_ADD:
            case CMD_ADDI:
                addOrSubOrI(instDest.dst_index, instDest.src1_index, instDest.src2_index_imm,ADD,instDest.isSrc2Imm);
                break;
            case CMD_LOAD:
                load(instDest.dst_index, instDest.src1_index, instDest.src2_index_imm,instDest.isSrc2Imm);
                break;
            case CMD_STORE:
                store(instDest.dst_index, instDest.src1_index, instDest.src2_index_imm,instDest.isSrc2Imm);
                break;
            case CMD_HALT:
                return FINISHED;
        }
        return RUNNING;
    }

    //choose imm or register val
    int setSrc2(int regAddOrImm, bool isImm){
        return (isImm)? regAddOrImm : regsTable.reg[regAddOrImm];
    }
    //implement add and update regs
    void addOrSubOrI(int destReg, int src1Reg, int src2Reg, op operation, bool imm){
        int val, src1, src2;
        src1 = regsTable.reg[src1Reg];
        src2 =  setSrc2(src2Reg, imm);

        //choose ADD or SUB
        if(operation == ADD)
            val = src1 + src2;
        else val = src1 -src2;
        //update regs value
        regsTable.reg[destReg] = val;
    }

    //implement load and update regs
    void load(int destReg, int src1Reg, int src2Reg, bool imm){
        int val, src1, src2;
        src1 = regsTable.reg[src1Reg];
        src2 =  setSrc2(src2Reg, imm);
        regsTable.reg[destReg] = regsTable.reg[src1 +  src2];
    }

    //implement store
    void store(int destReg, int src1Reg, int src2Reg, bool imm){
        int val, src1, src2, dest;
        src1 = regsTable.reg[src1Reg];
        src2 =  setSrc2(src2Reg, imm);
        dest = regsTable.reg[destReg];
        regsTable.reg[dest+src2] = regsTable.reg[src1];
    }
};


///////////////////////////////////////////////////////////////////////////////////////////
// CPU API
///////////////////////////////////////////////////////////////////////////////////////////
/*
 * virtual class for CPUs
 * */
class core{
protected:
    int loadLatency;
    int storeLatency;
    int threadsNum;
    int switchCyclesPenalty;

    int cycle;
    int cpuReadyCycle; // next possible cycle to run (if cpu penalty)
    int instructionCount;
    int curThread; // current running thread. when changed - have to wait penalty
    //int activeThreadsNum;
    int finishedThreads;
    //bool finished;
    vector<thread> threadVec;

public:
    core(int loadLat, int storeLat, int switchPen, int threadsNum):
    loadLatency(loadLat), storeLatency(storeLat), threadsNum(threadsNum),
    switchCyclesPenalty(switchPen), cycle(0), cpuReadyCycle(0), instructionCount(0),
    curThread(0), finishedThreads(0){
        for(int i = 0; i < threadsNum; i++){
            threadVec[i] = thread(i, 0);
        }
    }

    bool isFinished(){
        return finishedThreads == threadsNum;
    }
    virtual int getThreadTORun();
    void execute(int curCycle){
        cycle = curCycle;
        if(cycle < cpuReadyCycle){
            return; // have waiting penalty - not ready execute
        }
        int threadId = getThreadTORun();
        if(threadId == -1){
            //none of the threads can run - idle
            return;
        }
        else if(threadId != curThread){
            // context switch - penalty
            if(switchCyclesPenalty > 0){
                // surely can't run in this cycle
                cpuReadyCycle = cycle + switchCyclesPenalty;
                return;
            }
        }
        else {
            // current thread continue or no context switch penalty
            int status = threadVec[threadId].execute(cycle); // TODO: check if need to get output (waiting penalty...)
            if (status == FINISHED){
                finishedThreads++;
            }
            instructionCount++; // TODO: can be replaced with counting all the instructions from memory
        }
    }

};

class blockedMT: public core{

public:

    // return which thread turn to run (Round Robin)
    // blockedMT - switch on event
    int getThreadTORun(){
        if(threadVec[curThread].isReady){
            //current running thread can still run
            return curThread;
        }
        // find next ready thread
        for(int i = 1; i < threadsNum; i++){
            int tmpThreadID = (curThread + 1)% threadsNum ;
            if(threadVec[tmpThreadID].isReady){
                // do context switch
                curThread = tmpThreadID;
                return curThread;
            }
        }
        // none of the threads can run - must Idle
        return -1;
    }

};
class fineGrainedMT: public core{

public:

    // return which thread turn to run (Round Robin)
    // blockedMT - switch on event
    int getThreadTORun(){

        // find next ready thread
        for(int i = 1; i <= threadsNum; i++){
            int tmpThreadID = (curThread + 1)% threadsNum ;
            if(threadVec[tmpThreadID].isReady){
                // do context switch
                curThread = tmpThreadID;
                return curThread;
            }
        }
        // none of the threads can run - must Idle
        return -1;
    }


};

//#####################################################################################
// CORE private functions
//#####################################################################################
blockedMT * pBlockedMT;
fineGrainedMT * pFineGrainedMT;


//####################################################################################################
//  CORE API (for main)
//####################################################################################################
void CORE_BlockedMT() {
    // get simulator params
    int loadLatency = SIM_GetLoadLat();
    int storeLatency = SIM_GetStoreLat();
    int threadsNum = SIM_GetThreadsNum();
    int switchCycles = SIM_GetSwitchCycles();
    pBlockedMT = new blockedMT(); // TODO: create c'tor
    int cycle = 0;
    while(!pBlockedMT->isFinished()){
        pBlockedMT->execute(cycle);
        cycle++; // end of cycle
    }


}

void CORE_FinegrainedMT() {
    // get simulator params
    int loadLatency = SIM_GetLoadLat();
    int storeLatency = SIM_GetStoreLat();
    int threadsNum = SIM_GetThreadsNum();
    int switchCycles = SIM_GetSwitchCycles();
    pFineGrainedMT = new fineGrainedMT(); // TODO: create c'tor
    int cycle = 0;
    while(!pFineGrainedMT->isFinished()){
        pFineGrainedMT->execute(cycle);
        cycle++; // end of cycle
    }



}

double CORE_BlockedMT_CPI(){
	return 0;
}

double CORE_FinegrainedMT_CPI(){
	return 0;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
}


//
//class roundRobin{
//private:
//    int threadNum;
//
//public:
//    roundRobin(int threadNum): threadNum(threadNum){};
//    int getNextThread;
//};
