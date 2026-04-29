/*
 * Copyright (c) 2025 Technical University of Munich
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/lvp/lvp_stride.hh"

#include "base/intmath.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "debug/LVP.hh"
#include "debug/SP.hh"

namespace gem5
{

LVPStride::LVPStride(const LVPStrideParams &p)
    : ValuePredictor(p),
      lvpTable("LVPT", p.table_entries, p.table_assoc,
          p.table_replacement_policy, p.table_indexing_policy,
          LVPEntry(genTagExtractor(p.table_indexing_policy))),
      confThreshold(p.confidence_threshold),
      saveThreshold(p.savings_threshold),
      confResetToZero(p.confidence_reset_to_zero),
      useStride(p.use_stride),
      firstDump(true),
      lvpstats(this)
{
    DPRINTF(LVP, "Creating Stride Value predictor\n");

    if (!isPowerOf2(p.table_entries)) {
        fatal("LVPT entries is not a power of 2!");
    }
}

// LVPEntry* findEntry(ThreadID tid, Addr inst_addr)
// {

// }

#if 1

VPResult
LVPStride::lookup(ThreadID tid, Addr inst_addr, InstSeqNum seq_num)
{
    stats.lookups++;

    // Addr idx = index(inst_addr);
    LVPEntry::KeyType key = index(tid, inst_addr);
    LVPEntry * entry = lvpTable.findEntry(key);
    VPResult result;
    result.value = 0;
    result.predict = false;
    result.taken = LVP_STRONG_UNPREDICTABLE;

    if (entry && entry->tid == tid) {
        // DPRINTF(BTB, "BTB::%s: hit PC: %#x, idx:%#x \n",
        //              __func__, instPC, idx);
            // Get the number of inflights
        unsigned inflights = numInflights(inst_addr);
            // The value is this instance + in-flights * the stride
        result.value = entry->value + ((inflights + 1) * entry->stride);
        if (entry->confidence >= confThreshold) {
            // && entry->saving >= saveThreshold

            result.predict = true;
            // result.value = entry->value + (inflights * entry->stride);
            result.taken = LVP_PREDICTABLE;

            // Push the prediction instance to the in-flight queue
            inflightPred.push_front({inst_addr, seq_num});

            DPRINTF(LVP, "VP for iaddr=%#x, lastval=%llu,"
                "stride=%i, inflights=%i, conf=%i\n",
                    inst_addr, entry->value,
                    entry->stride, inflights, entry->confidence);
        }
        lvpTable.accessEntry(entry);
    }

    DPRINTF(LVP, "LVP::%s(iaddr=%#x, sn=%i)"
        "res:[pred=%i, value=%llu, taken=%i] IFsize=%i\n",
            __func__, inst_addr, seq_num,
            result.predict, result.value, result.taken, inflightPred.size());

    return result;
}


void
LVPStride::update(ThreadID tid, Addr inst_addr, InstSeqNum seq_num, Addr load_address,
                  RegVal correct_val, RegVal predicted_val,
                  LVPType classification, Cycles rn_to_ex_delay, bool critical)
{

    stats.totalLoads++;
    auto& ls = loadStats[inst_addr];
    ls.exec++;
    auto& la = ls.accesses[curTick()];
    la.predicted = predicted_val;
    la.correct = correct_val;
    la.predict=-1;
    //uint64_t d = rn_to_ex_delay;
    la.delta=0;

    if (critical) {
        //DPRINTF(SP,"Critical Load: %llu \n",inst_addr);
        ls.critical++;
    }

    LVPEntry::KeyType key = index(tid, inst_addr);
    LVPEntry * entry = lvpTable.findEntry(key);
    if (classification == LVP_PREDICTABLE) {
        ls.pred++;
        la.predict=1;
        if (predicted_val != correct_val) {
            ls.incorrect++;
            stats.incorrect++;
            DPRINTF(SP,"Misprediction: pval %llu,cval %llu,"
                "iaddr %llu, str %llu\n",
                predicted_val,correct_val, inst_addr, entry->stride);
            la.delta = -1;
        } else {
            ls.correct++;
            stats.correct++;
            uint64_t d = rn_to_ex_delay;
            lvpstats.valuePredSavedCyclesLog2.sample(d > 0 ? floorLog2(d) : 0);
            lvpstats.valuePredSavedCycles.sample(d);
            lvpstats.totalSavedCycles+=d;
            ls.savings+=d;
            la.delta = d;
            if (critical) {
                ls.crit_savings+=d;
            }

            if (entry->stride == 0) {
                lvpstats.constantCorrect++;
                if (entry->value == 0) {
                    lvpstats.numZeroConstLoads++;
                } else if (entry->value == 1) {
                    lvpstats.numOneConstLoads++;
                }
            } else {
                lvpstats.strideCorrect++;
            }
        }
    }
    DPRINTF(LVP, "LVP::%s(iaddr=%#x, sn=%llu, pval=%#x, cval=%#x, class=%i) pred=%i, correct=%i, delay=%i\n",
            __func__, inst_addr, seq_num, predicted_val,
            correct_val, classification,
            classification == LVP_PREDICTABLE,
            predicted_val == correct_val, rn_to_ex_delay);



    if (entry == nullptr) {
        entry = lvpTable.findVictim(key);
        lvpTable.insertEntry(key, entry);

        entry->confidence = 0;
        entry->saving = 0;
        entry->tid = tid;
        entry->stride = 0;
        entry->value = correct_val;
        DPRINTF(LVP, "Allocate new entry: %llu\n", entry->value);
        return;
    }

    lvpTable.accessEntry(entry);
    uint64_t last_value = entry->value;
    uint64_t pred_value = entry->value + entry->stride;
    int64_t stride = useStride ? ((int64_t)correct_val - (int64_t)last_value) : 0;

    // Update the latest value
    entry->value = correct_val;
    if (classification == LVP_PREDICTABLE) {
        assert(inflightPred.size() && (inflightPred.back().sn == seq_num));
        inflightPred.pop_back();
    }

    la.conf=entry->confidence;

    if (last_value != correct_val) {
        if (entry->confidence > 0) {
            DPRINTF(LVP, "Test\n");
            entry->confidence = confResetToZero ? 0 : entry->confidence - 1;
        }
        if (entry->confidence == 0) {
            entry->stride = stride;
        }
    } else {
        if (entry->confidence < confThreshold) {
            entry->confidence++;
        }

    }
    entry->saving = rn_to_ex_delay;

    DPRINTF(LVP, "Entry update: pred=%i, conf=%i, val=%li,"
        "last_val %li, pred_val %li, stride=%li, IFsize=%i\n",
            pred_value != correct_val, entry->confidence, entry->value,
            last_value, pred_value, entry->stride, inflightPred.size());
}

void
LVPStride::squash(InstSeqNum seq_num)
{
    DPRINTF(LVP, "Squash inflight prediction until sn:%llu\n", seq_num);
    while (!inflightPred.empty() && inflightPred.front().sn > seq_num) {
        inflightPred.pop_front();
    }
}

unsigned
LVPStride::numInflights(Addr iaddr)
{
    unsigned n = 0;
    for (auto& e : inflightPred) {
        if (e.iaddr == iaddr)
            n++;
    }
    return n;
}

void
LVPStride::addpenalty(Cycles delta, Addr load_addr)
{
    uint64_t d=delta;
    lvpstats.totalPenaltyCycles+=d;
    lvpstats.penaltyCycles.sample(d);
    lvpstats.penaltyCyclesLog2.sample(d > 0 ? floorLog2(d) : 0);

    auto& ls = loadStats[load_addr];
    ls.penalty+=d;

    DPRINTF(SP,"Penalty: %" PRIu64" from Load %llu\n", d, load_addr);
}

void
LVPStride::dump_and_reset(const std::string& filename)
{
    std::ios::openmode mode = firstDump
        ? std::ios::out
        : (std::ios::out | std::ios::app);

    std::ofstream fileStream_sum(
        simout.resolve(filename+"_summary.csv"), mode);
    std::ofstream fileStream_acc(
        simout.resolve(filename+"_accesses.csv"), mode);


    if (!fileStream_sum.good())
        panic("Could not open %s for writing\n", filename+"_summary.csv");
    if (!fileStream_acc.good())
        panic("Could not open %s for writing\n", filename+"_accesses.csv");
    if (firstDump) {
        ccprintf(fileStream_sum,
                "pc,exec,pred,correct,incorrect,"
                "penalty,savings,critical,crit_savings\n");
        ccprintf(fileStream_acc,
                "pc,tick,predict,predicted,correct,delta,confidence\n");
    }else{
        ccprintf(fileStream_sum,"-1,0,0,0,0,0,0,0,0\n");
        ccprintf(fileStream_acc,"-1,0,0,0,0,0\n");
    }

    // Dump stats for summary
    for (auto& li : loadStats) {
        ccprintf(fileStream_sum,"%llu,%i,%i,%i,%i,%i,%i,%i,%i\n",
                 li.first,
                 li.second.exec,
                 li.second.pred,
                 li.second.correct,
                 li.second.incorrect,
                 li.second.penalty,
                 li.second.savings,
                 li.second.critical,
                 li.second.crit_savings
                );
    }
    fileStream_sum.close();

    //Dump stats for per access save
    for (auto& li : loadStats) {
        const auto& stat = li.second;

            for (const auto& acc : stat.accesses) {
            ccprintf(fileStream_acc,"%llu,%llu,%i,%llu,%llu,%i,%llu\n",
                li.first,
                acc.first,
                acc.second.predict,
                acc.second.predicted,
                acc.second.correct,
                acc.second.delta,
                acc.second.conf
            );
        }
    }
    fileStream_acc.close();

    loadStats.clear();
    firstDump = false;
}

#else

// @todo Create some sort of return struct that has both whether or not the
// address is valid, and also the address.  For now will just use addr = 0 to
// represent invalid entry.
VPResult
LVPStride::lookup(ThreadID tid, Addr inst_addr)
{
    stats.lookups++;

    Addr idx = index(inst_addr);
    LVPEntry * entry = &(lvpMap[idx]);
    VPResult result;
    result.predict = false;
    result.taken = LVP_STRONG_UNPREDICTABLE;

    if (entry != nullptr && entry->tid == tid) {
        // DPRINTF(BTB, "BTB::%s: hit PC: %#x, idx:%#x \n",
        //              __func__, instPC, idx);
        if (entry->confidence >= confThreshold) {
            result.predict = true;
            result.value = entry->value;
            result.taken = LVP_PREDICTABLE;
        }
        // result.value
        // lvpTable.accessEntry(entry);
    }
    DPRINTF(LVP, "LVP::%s(iaddr=%#x) res:[pred=%i, value=%#x, conf=%i]\n",
            __func__, inst_addr, result.predict, result.value, result.taken);

    return result;
}


void
LVPStride::update(ThreadID tid, Addr inst_addr, Addr load_address,
                          RegVal correct_val, RegVal predicted_val,
                          LVPType classification)
{
    Addr idx = index(inst_addr);
    LVPEntry * entry = &(lvpMap[idx]);

    DPRINTF(LVP, "LVP::%s(iaddr=%#x, pval=%#x, cval=%#x, class=%i) conf=%i, pred=%i, correct=%i\n",
            __func__, inst_addr, predicted_val, correct_val, classification,
            entry->confidence, classification == LVP_PREDICTABLE, predicted_val == correct_val);

    if (classification == LVP_PREDICTABLE) {
        if (predicted_val != correct_val) {
            stats.incorrect++;
        } else {
            stats.correct++;
        }
    }
    stats.totalLoads++;

    if (entry->value != correct_val) {
        entry->value = correct_val;
        entry->tid = tid;
        entry->confidence = 0;
    } else {
        if (entry->confidence < confThreshold) {
            entry->confidence++;
        }
    }
}

#endif


Addr
LVPStride::index(Addr addr) {
    return (addr >> instShiftAmt);
}

LVPStride::LVPEntry::KeyType
LVPStride::index(ThreadID tid, Addr inst_addr) {
    return LVPEntry::KeyType{(inst_addr >> instShiftAmt) ^ tid, false};
}

LVPStride::LVPStrideStats::LVPStrideStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(constantCorrect, statistics::units::Count::get(),
               "Number of VP lookups"),
      ADD_STAT(strideCorrect, statistics::units::Count::get(),
               "Number of VP lookups"),
      ADD_STAT(numZeroConstLoads, statistics::units::Count::get(),
               "Number of constant loads with value 0"),
      ADD_STAT(numOneConstLoads, statistics::units::Count::get(),
               "Number of constant loads with value 1"),
      ADD_STAT(valuePredSavedCyclesLog2, statistics::units::Count::get(),
               "Required for Top-Down, number of committed instructions"),
      ADD_STAT(valuePredSavedCycles, statistics::units::Count::get(),
               "Required for Top-Down, number of committed instructions"),
      ADD_STAT(penaltyCyclesLog2, statistics::units::Count::get(),
               "Required for Top-Down, penalty"),
      ADD_STAT(penaltyCycles, statistics::units::Count::get(),
               "Required for Top-Down, penalty"),
      ADD_STAT(totalPenaltyCycles, statistics::units::Count::get(),
               "Required for Top-Down, total penalty"),
      ADD_STAT(totalSavedCycles, statistics::units::Count::get(),
               "Required for Top-Down, total savings")
{
    valuePredSavedCyclesLog2.init(0, 15, 1).flags(statistics::pdf);
    valuePredSavedCycles.init(0, 40, 2).flags(statistics::pdf);
    penaltyCyclesLog2.init(0, 15, 1).flags(statistics::pdf);
    penaltyCycles.init(0, 40, 2).flags(statistics::pdf);
}

} // namespace gem5::branch_prediction
