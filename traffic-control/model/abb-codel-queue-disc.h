/*
 * Copyright (c) 2023 Gongbing Hong
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * ABB, the Adaptive Bandwidth Binning queue discipline
 * Based on ns2 simulation code by Gongbing Hong
 *
 * Authors: Gongbing Hong <hgb.bus@gmail.com>
 *
 */

#ifndef ABB_CODEL_QUEUE_DISC
#define ABB_CODEL_QUEUE_DISC

#include "ns3/queue-disc.h"

#include <list>
#include <map>

namespace ns3
{

/**
 * \ingroup traffic-control
 *
 * \brief A bin queue used by the ABBCoDel queue disc
 * Multiple flows may share the same bin dynamically.
 */

class ABBCoDelBin : public QueueDiscClass
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief ABBCoDelBin constructor
     */
    ABBCoDelBin();

    ~ABBCoDelBin() override;

    /**
     * \enum BinStatus
     * \brief Used to determine the status of this bin queue
     */
    enum BinStatus
    {
        INACTIVE,
        NEW_BIN,
        OLD_BIN
    };

    /**
     * \brief Set the deficit for this bin
     * \param deficit the deficit for this bin
     */
    void SetDeficit(uint32_t deficit);

    /**
     * \brief Get the deficit for this bin
     * \return the deficit for this bin
     */
    int32_t GetDeficit() const;

    /**
     * \brief Increase the deficit for this bin
     * \param deficit the amount by which the deficit is to be increased
     */
    void IncreaseDeficit(int32_t deficit);

    /**
     * \brief Set the status for this bin
     * \param status the status for this bin
     */
    void SetStatus(BinStatus status);

    /**
     * \brief Get the status of this bin
     * \return the status of this bin
     */
    BinStatus GetStatus() const;

    /**
     * \brief Set the index for this bin
     * \param index the index for this bin
     */
    void SetIndex(uint32_t index);

    /**
     * \brief Get the index of this bin
     * \return the index of this bin
     */
    uint32_t GetIndex() const;

    /**
     * \brief Set the bandwidth threshold for this bin
     * \param bwt the bandwidth threshould for this bin in units of mbps
     */
    void SetBandwidthThreshold(double bwt);

    /**
     * \brief Get the bandwidth threshold of this bin
     * \return the bandwidth threshold of this bin
     */
    double GetBandwidthThreshold() const;

    /**
     * \brief Set the weight for this bin
     * \param weight the weight for this bin
     */
    void SetWeight(double weight);

    /**
     * \brief Get the weight of this bin
     * \return the weight of this bin 
     */
    double GetWeight() const;

    /**
     * \brief Increase the weight for this bin
     * \param weight the amount by which the weight is to be increased
     */
    void IncreaseWeight(double weight);

  private:
    int32_t m_deficit;      //!< the deficit for this bin
    BinStatus m_status;     //!< the status of this bin
    uint32_t m_index;       //!< the index for this bin

    double m_bwthreshold;   //!< the bandwidth consumption threshold in units of mbps
    double m_weight;        //!< the weight for this bin is the sum of the weights 
                            //!< of all flows classified into this bin
};

/**
 * \ingroup traffic-control
 *
 * \brief An ABBCoDel packet queue disc
 */

class ABBCoDelQueueDisc : public QueueDisc
{
  public:
    /**
     * \brief Get the type ID
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief ABBCoDelQueuDisc constructor
     */
    ABBCoDelQueueDisc();

    ~ABBCoDelQueueDisc() override;

    // delete copy constructor and assignment operator to avoid misuse
    ABBCoDelQueueDisc(const ABBCoDelQueueDisc&) = delete;
    ABBCoDelQueueDisc& operator=(const ABBCoDelQueueDisc&) = delete;

    /**
     * \brief Set the quantum value (unweighted)
     *
     * \param quantum The number of bytes each bin gets to dequeue on each round of the scheduling
     * algorithm
     */
    void SetQuantum(uint32_t quantum);

    /**
     * \brief Get the quantum value (unweighted)
     *
     * \returns The number of bytes each bin gets to dequeue on each round of the scheduling
     * algorithm
     */
    uint32_t GetQuantum() const;

    // Reasons for dropping packets
    static constexpr const char* UNCLASSIFIED_DROP =
        "Unclassified drop"; //!< No packet filter able to classify packet
    static constexpr const char* OVERLIMIT_DROP = "Overlimit drop"; //!< Overlimit dropped packets

  private:
    bool DoEnqueue(Ptr<QueueDiscItem> item) override;
    Ptr<QueueDiscItem> DoDequeue() override;
    bool CheckConfig() override;                // called before InitializeParams
    void InitializeParams() override;           // called after CheckConfig

    void Optimize();

    struct FlowInfo
    {
        /// Flow id (for human eyes only)
        uint32_t m_id;

        /// The flow weight
        double m_weight;

        /// The ID of the bin this flow is classified to
        uint32_t m_binId;
        
        /// Internal flow allocation flag for flow reclassification
        bool m_alloced;

        /// Average service (departure) rate (in units of mbps)
        double m_avgServiceRate;

        /// Average arrival rate (in units of mbps)
        double m_avgArrivalRate;

        // exponential moving average method is used:
        // https://en.wikipedia.org/wiki/Exponential_smoothing
        /// Exponential moving average smoothing factor
        double m_rateWeight;

        /// Last service sampling start time
        Time m_lastSampleTime;

        /// Number of packets received for current service sample
        uint32_t m_nPacketsReceived;
        /// Number of data bytes received for current service sample 
        uint64_t m_nBytesReceived;
        /// Number of packets sent for current service sample
        uint32_t m_nPacketsSent;
        /// Number of data bytes sent for current service sample
        uint64_t m_nBytesSent;

        /// constructor
        FlowInfo(uint32_t id, double weight);

        void PacketReceived(Ptr<QueueDiscItem> item);
        void PacketSent(Ptr<QueueDiscItem> item);
        void UpdateRates();
    };

    /**
     * \brief Get flow info
     * 
     * \param item The item containing a packet from a flow
     * \returns Reference to ABBFlowInfo structure
     */
    FlowInfo& GetFlowInfo(Ptr<QueueDiscItem> item);

    /**
     * \brief Drop a packet from the head of the bin with the largest current byte count
     * \return the index of the bin with the largest current byte count
     */
    uint32_t ABBCoDelDrop();

    // ECN uses two bits in IP packets: 
    //      00 means ECN not supported;
    //      01 means ECN Capable Transport(1) or ECT1 (not commonly used, being redefined by L4S);
    //      10 means ECN Capable Transport(0) or ECT0 (used mostly);
    //      11 means CE (Congestion Experienced marked by qdisc/router)
    //      (When CE marking is on, receiving end needs to signal sender using transport layer)
    // This is set by default to true!!!
    bool m_useEcn;  //!< True if ECN is used (packets are marked instead of being dropped)

    std::string m_interval;         //!< CoDel interval attribute
    std::string m_target;           //!< CoDel target attribute

    uint32_t m_quantum;             //!< Deficit assigned to flow bins at each round

    uint32_t m_nBins;               //!< Number of bins (equivalent to m_flows in FqCoDel code)
    uint32_t m_rcInterval;          //!< Number of seconds for flow reclassification
    double m_channelBW;             //!< Channel bandwith (capacity) in mbps

    uint32_t m_dropBatchSize;       //!< Max number of packets dropped from the fat flow

    // This is set by default to a very large value; essentially it is turned off by default
    Time m_ceThreshold;             //!< Threshold above which to CE mark
    // Default to false - L4S stands for Low Latency, Low Loss, and Scalable Throughput
    bool m_useL4s;                  //!< True if L4S is used (ECT1 packets are marked at CE threshold)

    std::list<Ptr<ABBCoDelBin>> m_newBins;  //!< The list of new bins
    std::list<Ptr<ABBCoDelBin>> m_oldBins;  //!< The list of old bins

    Ptr<ABBCoDelBin> m_bins[6*5];           //!< ABB CoDel bins (up to 6; not all of them are used)

    std::vector<FlowInfo> m_flows;              //!< list of flows
    std::map<uint32_t, uint32_t> m_flowIndices; //!< Map flow id to flow index in flow list

    Callback<uint32_t, Ptr<QueueDiscItem>> m_flowIdCb;
    Callback<double, Ptr<QueueDiscItem>> m_flowWeightCb;
};

} // namespace ns3

#endif /* ABB_CODEL_QUEUE_DISC */
