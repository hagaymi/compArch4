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
    int loadPenalty;
    int storePenalty;
    int readyCycle;
    tcontext regsTable{};
    uint32_t pc;
    Instruction instDest{};
    thread(int id, int loadPenalty, int storePenalty, uint32_t init_pc):id(id), loadPenalty(loadPenalty), storePenalty(storePenalty),readyCycle(0), pc(init_pc){} //TODO: make sure regsTable doesn't need initialization

    bool isReady(int currCycle){
        return currCycle > readyCycle;
    }
    // look what is the next instruction and call the relevant function and update the return value in memory update the pc
    threadState execute(int currCycle){
        int opc;
        SIM_MemInstRead(pc,  &instDest, id);
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
                readyCycle = currCycle + loadPenalty;
                break;
            case CMD_STORE:
                store(instDest.dst_index, instDest.src1_index, instDest.src2_index_imm,instDest.isSrc2Imm);
                readyCycle = currCycle + storePenalty;
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
