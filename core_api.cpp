/* 046267 Computer Architecture - Winter 2019/20 - HW #4 */

#include "core_api.h"
#include "sim_api.h"
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <stdint.h>
#include <iostream>


using std::cout;
using std::endl;
#define DEBUG 0
void log(const char * msg){
    if(DEBUG){
        std::cout << msg << std::endl;
    }
}


typedef enum {ADD, SUB} op;
typedef enum {RUNNING, FINISHED} threadState;
using std::vector;

/// initial tcontext struct - regs = {0}
/// \param r tcontext instance to initiate
void init(tcontext r){
    for(int i=0; i< REGS_COUNT; i++){
        r.reg[i]=0;
    }
}

/// copys registers of contest
/// \param dest dest context
/// \param src source context
void copyArray(tcontext* dest, tcontext* src){
    for(int i=0; i < REGS_COUNT; i++){
        dest->reg[i] = src->reg[i];
    }
}

/// class of thread
/// holds thread context and do instructions executions
class thread{
private:
    int id;
    int loadPenalty;
    int storePenalty;
    int readyCycle;
    threadState state;

    uint32_t pc;
   //
public:
    tcontext regsTable;
public:
    /// c'tor
    /// \param id
    /// \param loadPenalty
    /// \param storePenalty
    /// \param init_pc
    thread(int id, int loadPenalty, int storePenalty, uint32_t init_pc):id(id), loadPenalty(loadPenalty),
            storePenalty(storePenalty),readyCycle(-1), state(RUNNING),pc(init_pc), regsTable(){
        init(regsTable);
    } //TODO: make sure regsTable doesn't need initialization

    /// check if thread is ready for execution
    /// \param currCycle  current cycle
    /// \return ture / false (can thread run in current cycle)
    bool isReady(int currCycle){
        if(state == FINISHED)
            return false;
        return currCycle > readyCycle;
    }

    /// look what is the next instruction and call the relevant function and update the return value in memory update the pc
    /// \param currCycle
    /// \return if thread is finished or not
    threadState execute(int currCycle){
       // int opc;
        Instruction instDest;
        SIM_MemInstRead(pc,  &instDest, id);
       //
        pc = (uint32_t) (pc + 1); //TODO: make sure the pc is the correct way

        switch (instDest.opcode) {
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
                readyCycle = currCycle + loadPenalty;
                break;
            case CMD_STORE:
                store(instDest.dst_index, instDest.src1_index, instDest.src2_index_imm,instDest.isSrc2Imm);
                readyCycle = currCycle + storePenalty;
                break;
            case CMD_HALT:
                state = FINISHED;
                return FINISHED;
        }
        return RUNNING;
    }

    ///choose imm or register val
    /// \param regAddOrImm
    /// \param isImm
    /// \return
    int setSrc2(int regAddOrImm, bool isImm){
        return (isImm)? regAddOrImm : regsTable.reg[regAddOrImm];
    }
    ///implement add and update regs
    /// \param destReg
    /// \param src1Reg
    /// \param src2Reg
    /// \param operation
    /// \param imm
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

    /// implement load and update regs
    /// \param destReg
    /// \param src1Reg
    /// \param src2Reg
    /// \param imm
    void load(int destReg, int src1Reg, int src2Reg, bool imm){
        int val, src1, src2;
        src1 = regsTable.reg[src1Reg];
        src2 =  setSrc2(src2Reg, imm);
        int res = SIM_MemDataRead((uint32_t)(src1+src2), &(regsTable.reg[destReg]));
        if(DEBUG){
            cout << "thread id: " << id << endl;
            cout << "load: " << "dst reg: " << destReg << " memory addr:  " << (uint32_t)(src1+src2) << endl;
            cout << "       " << " value: " << regsTable.reg[destReg] << endl;
        }
    }

    //implement store
    void store(int destReg, int src1Reg, int src2Reg, bool imm){
        int val, src1, src2, dest;
        src1 = regsTable.reg[src1Reg];
        src2 =  setSrc2(src2Reg, imm);
        dest = regsTable.reg[destReg];
        SIM_MemDataWrite((uint32_t)(dest+src2), src1);
        if(DEBUG){
            cout << "thread id: " << id << endl;
            cout << "store: " <<  "memory addr: " <<(uint32_t)(dest+src2) << " value: " << src1 << endl;
        }
    }
};


///////////////////////////////////////////////////////////////////////////////////////////
// CPU API
///////////////////////////////////////////////////////////////////////////////////////////
/*
 * virtual class for CPUs
 * */
class core{
public:
    int loadLatency;
    int storeLatency;
    int threadsNum;
    int switchCyclesPenalty;

    int cycle;
    int cpuReadyCycle; // next possible cycle to run (if cpu penalty)
    int instructionCount;
    int curThread; // current running thread. when changed - have to wait penalty
    //int activeThreadsNum;
    int finishedThreads; // counts finished threads
    //bool finished;
    vector<thread> threadVec;

    core(int loadLat, int storeLat, int switchPen, int threadsNum):
    loadLatency(loadLat), storeLatency(storeLat), threadsNum(threadsNum),
    switchCyclesPenalty(switchPen), cycle(0), cpuReadyCycle(0), instructionCount(0),
    curThread(0), finishedThreads(0){
        threadVec.reserve(threadsNum);
        for(int i = 0; i < threadsNum; i++){
            threadVec[i] = thread(i, loadLatency, storeLatency, 0);
        }
    }

    bool isFinished(){
        return finishedThreads == threadsNum;
    }
    virtual int getThreadTORun(){};
    virtual void execute(int curCycle){};

};

class blockedMT: public core{

public:

    // return which thread turn to run (Round Robin)
    // blockedMT - switch on event
    blockedMT(int loadLat, int storeLat, int switchPen, int threadsNum) : core(loadLat, storeLat, switchPen,
                                                                               threadsNum) {}

    int getThreadTORun(){
        if(threadVec[curThread].isReady(cycle)){
            //current running thread can still run
            return curThread;
        }
        // find next ready thread
        for(int i = 1; i < threadsNum; i++){
            int tmpThreadID = (curThread + i)% threadsNum ;
            if(threadVec[tmpThreadID].isReady(cycle)){ // TODO: need to replace in thread's method
                // do context switch
                //curThread = tmpThreadID;
                return tmpThreadID;
            }
        }
        // none of the threads can run - must Idle
        return -1;
    }
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
        else if(threadId != curThread){ // TODO: make sure in fineGrained switchPenalty=0
            // context switch - penalty
            if(switchCyclesPenalty > 0){
                // surely can't run in this cycle
                cpuReadyCycle = cycle + switchCyclesPenalty;
                curThread = threadId;
                return;
            }
        }
        else {
            // current thread continue or no context switch penalty
            if(threadId < 0 || threadId >= threadsNum )
                throw ("illegal thread ID ");
            int status = threadVec[threadId].execute(cycle); // TODO: check if need to get output (waiting penalty...)
            if (status == FINISHED){
                finishedThreads++;
            }
            instructionCount++; // TODO: can be replaced with counting all the instructions from memory
        }
    }
};
class fineGrainedMT: public core{

public:
    fineGrainedMT(int loadLat, int storeLat, int switchPen, int threadsNum) : core(loadLat, storeLat, switchPen,
                                                                                   threadsNum) {}

    // return which thread turn to run (Round Robin)
    // blockedMT - switch on event
    int getThreadTORun(){
        if(cycle == 0){
            // for first execution
            return 0;
        }
        // find next ready thread
        for(int i = 1; i <= threadsNum; i++){
            int tmpThreadID = (curThread + i)% threadsNum ;
            if(threadVec[tmpThreadID].isReady(cycle)){
                // do context switch
                //curThread = tmpThreadID;
                return tmpThreadID;
            }
        }
        // none of the threads can run - must Idle
        return -1;
    }

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
//
        else {
            // current thread continue or no context switch penalty
            curThread = threadId; //TODO: verify the right place to update current thread (for idle cases)
            int status = threadVec[threadId].execute(cycle); // TODO: check if need to get output (waiting penalty...)
            if (status == FINISHED){
                finishedThreads++;
            }
            instructionCount++; // TODO: can be replaced with counting all the instructions from memory
        }
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
    pBlockedMT = new blockedMT(loadLatency, storeLatency, switchCycles, threadsNum); // TODO: create c'tor
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
    pFineGrainedMT = new fineGrainedMT(loadLatency, storeLatency, switchCycles, threadsNum); // TODO: create c'tor
    int cycle = 0;
    while(!pFineGrainedMT->isFinished()){
        pFineGrainedMT->execute(cycle);
        cycle++; // end of cycle
    }



}

double CORE_BlockedMT_CPI(){
    double cycles = (double)pBlockedMT->cycle + 1;
    double instructions = (double)pBlockedMT->instructionCount;
	return cycles/instructions;
}

double CORE_FinegrainedMT_CPI(){
    double cycles = (double)pFineGrainedMT->cycle + 1;
    double instructions = (double)pFineGrainedMT->instructionCount;
    return cycles/instructions;}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    copyArray(&context[threadid],&pBlockedMT->threadVec[threadid].regsTable);
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    copyArray(&context[threadid],&pFineGrainedMT->threadVec[threadid].regsTable);
}




