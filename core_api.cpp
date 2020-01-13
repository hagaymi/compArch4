/* 046267 Computer Architecture - Winter 2019/20 - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>
#include <vector>
using std::vector;

class thread{
public:
    int isReady;
    int reg[REGS_COUNT];
    unsigned int pc;
    thread():isReady(0){} //TODO: make sure regs doesn't need initialization

    // look what is the next instruction and call the relevant function
    void execute(int currCycle);
    //implement add and update regs
    void add(int destReg, int src1Reg, int src2Reg);
    //implement addi and update regs
    void addi(int destReg, int src1Reg, int imm);
    //TODO: implement sub and subi using same func as add or new func

    //implement load and update regs
    void load(int destReg, unsigned int address);
    //implement store
    void store(unsigned int address, int src1Reg);
    //TODO: make sure I don't need to implement hault
};


///////////////////////////////////////////////////////////////////////////////////////////
// CPU API
///////////////////////////////////////////////////////////////////////////////////////////
/*
 * virtual class for CPUs
 * */
class core{
protected:
    int cycle;
    int instructionCount;

    int loadLatency;
    int storeLatency;
    int threadsNum;
    int activeThreadsNum;
    int switchCycles;
    bool finished;
    vector<thread> threadVec;

public:
    core();
    bool isFinished();
    void execute(int curCycle);

};

class blockedMT: public core{

};
class fineGrainedMT: public core{

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
