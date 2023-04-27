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

#include "abb-codel-queue-disc.h"
#include "codel-queue-disc.h"

#include "ns3/log.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/queue.h"
#include "ns3/string.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ABBCoDelQueueDisc");

NS_OBJECT_ENSURE_REGISTERED(ABBCoDelBin);

TypeId
ABBCoDelBin::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ABBCoDelBin")
                            .SetParent<QueueDiscClass>()
                            .SetGroupName("TrafficControl")
                            .AddConstructor<ABBCoDelBin>();
    return tid;
}

ABBCoDelBin::ABBCoDelBin()
    : m_deficit(0),
      m_status(INACTIVE),
      m_index(0),
      m_bwthreshold(0.0),
      m_weight(0.0)
{
    NS_LOG_FUNCTION(this);
}

ABBCoDelBin::~ABBCoDelBin()
{
    NS_LOG_FUNCTION(this);
}

void
ABBCoDelBin::SetDeficit(uint32_t deficit)
{
    NS_LOG_FUNCTION(this << deficit);
    m_deficit = deficit;
}

int32_t
ABBCoDelBin::GetDeficit() const
{
    NS_LOG_FUNCTION(this);
    return m_deficit;
}

void
ABBCoDelBin::IncreaseDeficit(int32_t deficit)
{
    NS_LOG_FUNCTION(this << deficit);
    int32_t old_deficit = m_deficit;
    m_deficit += deficit;
    NS_LOG_INFO("Deficit changed from " << old_deficit << " to " << m_deficit);
}

void
ABBCoDelBin::SetStatus(BinStatus status)
{
    NS_LOG_FUNCTION(this << status);
    m_status = status;
}

ABBCoDelBin::BinStatus
ABBCoDelBin::GetStatus() const
{
    NS_LOG_FUNCTION(this);
    return m_status;
}

void
ABBCoDelBin::SetIndex(uint32_t index)
{
    NS_LOG_FUNCTION(this << index);
    m_index = index;
}

uint32_t
ABBCoDelBin::GetIndex() const
{
    NS_LOG_FUNCTION(this);
    return m_index;
}

void
ABBCoDelBin::SetBandwidthThreshold(double bwt)
{
    NS_LOG_FUNCTION(this << bwt);
    m_bwthreshold = bwt;
}

double
ABBCoDelBin::GetBandwidthThreshold() const
{
    NS_LOG_FUNCTION(this);
    return m_bwthreshold;
}

void
ABBCoDelBin::SetWeight(double weight)
{
    NS_LOG_FUNCTION(this << weight);
    m_weight = weight;
}

double
ABBCoDelBin::GetWeight() const
{
    NS_LOG_FUNCTION(this);
    return m_weight;
}

void
ABBCoDelBin::IncreaseWeight(double weight)
{
    NS_LOG_FUNCTION(this << weight);
    double old_weight = m_weight;
    m_weight += weight;
    NS_LOG_INFO("Bin " << m_index << " weight increased from " << old_weight << " to " << m_weight);
}

NS_OBJECT_ENSURE_REGISTERED(ABBCoDelQueueDisc);

ABBCoDelQueueDisc::FlowInfo::FlowInfo(uint32_t id, double weight)
    : m_id(id), m_weight(weight), m_binId(0), m_alloced(false),
      m_avgServiceRate(0.0), m_avgArrivalRate(0.0),
      m_rateWeight(0.4), m_lastSampleTime(0),
      m_nPacketsReceived(0), m_nBytesReceived(0), 
      m_nPacketsSent(0), m_nBytesSent(0)
{
    NS_LOG_FUNCTION(this << id << weight);
    NS_ASSERT_MSG(m_weight > 0, "Bin weight must be positive number");
}

void
ABBCoDelQueueDisc::FlowInfo::PacketReceived(Ptr<QueueDiscItem> item)
{
    m_nPacketsReceived++;
    m_nBytesReceived += item->GetSize();
}

void
ABBCoDelQueueDisc::FlowInfo::PacketSent(Ptr<QueueDiscItem> item)
{
    m_nPacketsSent++;
    m_nBytesSent += item->GetSize();
}

void
ABBCoDelQueueDisc::FlowInfo::UpdateRates()
{
    NS_LOG_FUNCTION(this);

    Time currentTime = Simulator::Now();
    double sampleTime = (currentTime - m_lastSampleTime).GetSeconds();

    if (sampleTime > 0)
    {
        double arrivalRateSample = m_nBytesReceived * 8 / sampleTime / 1000000.0;   // in mbps
        m_avgArrivalRate = (1 - m_rateWeight) * m_avgArrivalRate + m_rateWeight * arrivalRateSample;
        double departureRateSample = m_nBytesSent * 8 / sampleTime / 1000000.0;     // in mbps
        m_avgServiceRate = (1 - m_rateWeight) * m_avgServiceRate + m_rateWeight * departureRateSample;
    }

    // reset for next sample
    m_nPacketsReceived = 0;
    m_nBytesReceived = 0;
    m_nPacketsSent = 0;
    m_nBytesSent = 0;

    m_lastSampleTime = currentTime;
}

TypeId
ABBCoDelQueueDisc::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ABBCoDelQueueDisc")
            .SetParent<QueueDisc>()
            .SetGroupName("TrafficControl")
            .AddConstructor<ABBCoDelQueueDisc>()
            .AddAttribute("UseEcn",
                          "True to use ECN (packets are marked instead of being dropped)",
                          BooleanValue(true),
                          MakeBooleanAccessor(&ABBCoDelQueueDisc::m_useEcn),
                          MakeBooleanChecker())
            .AddAttribute("Interval",
                          "The CoDel algorithm interval for each ABBCoDel bin (queue)",
                          StringValue("100ms"),
                          MakeStringAccessor(&ABBCoDelQueueDisc::m_interval),
                          MakeStringChecker())
            .AddAttribute("Target",
                          "The CoDel algorithm target queue delay for each ABB bin (queue)",
                          StringValue("5ms"),
                          MakeStringAccessor(&ABBCoDelQueueDisc::m_target),
                          MakeStringChecker())
            .AddAttribute("MaxSize",
                          "The maximum number of packets accepted by this queue disc",
                          QueueSizeValue(QueueSize("10240p")),
                          MakeQueueSizeAccessor(&QueueDisc::SetMaxSize, &QueueDisc::GetMaxSize),
                          MakeQueueSizeChecker())
            .AddAttribute("Bins",
                          "The number of bins (queues) used to classify incoming packets from all flows",
                          UintegerValue(5),
                          MakeUintegerAccessor(&ABBCoDelQueueDisc::m_nBins),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("RcInterval",
                          "Flow reclassification interval in the number of seconds",
                          UintegerValue(1),
                          MakeUintegerAccessor(&ABBCoDelQueueDisc::m_rcInterval),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("ChannelBW",
                          "Channel bandwidth in the unit of Mbps",
                          DoubleValue(1000.0),
                          MakeDoubleAccessor(&ABBCoDelQueueDisc::m_channelBW),
                          MakeDoubleChecker<double>(10.0))
            .AddAttribute("DropBatchSize",
                          "The maximum number of packets dropped from the fat bin",
                          UintegerValue(64),
                          MakeUintegerAccessor(&ABBCoDelQueueDisc::m_dropBatchSize),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("CeThreshold",
                          "The ABBCoDel CE threshold for marking packets",
                          TimeValue(Time::Max()),
                          MakeTimeAccessor(&ABBCoDelQueueDisc::m_ceThreshold),
                          MakeTimeChecker())
            .AddAttribute("UseL4s",
                          "True to use L4S (only ECT1 packets are marked at CE threshold)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ABBCoDelQueueDisc::m_useL4s),
                          MakeBooleanChecker())
            .AddAttribute("FlowIdCb",
                          "The flow id callback for finding the flow id of a QueueDiscItem",
                          CallbackValue(),
                          MakeCallbackAccessor(&ABBCoDelQueueDisc::m_flowIdCb),
                          MakeCallbackChecker())
            .AddAttribute("FlowWeightCb",
                          "The flow weight callback for finding the flow weight of a QueueDiscItem",
                          CallbackValue(),
                          MakeCallbackAccessor(&ABBCoDelQueueDisc::m_flowWeightCb),
                          MakeCallbackChecker());
    return tid;
}

ABBCoDelQueueDisc::ABBCoDelQueueDisc()
    : QueueDisc(QueueDiscSizePolicy::MULTIPLE_QUEUES, QueueSizeUnit::PACKETS),
      m_quantum(0) 
{
    NS_LOG_FUNCTION(this);
}

ABBCoDelQueueDisc::~ABBCoDelQueueDisc()
{
    NS_LOG_FUNCTION(this);
}

void
ABBCoDelQueueDisc::SetQuantum(uint32_t quantum)
{
    NS_LOG_FUNCTION(this << quantum);
    m_quantum = quantum;
}

uint32_t
ABBCoDelQueueDisc::GetQuantum() const
{
    return m_quantum;
}

bool
ABBCoDelQueueDisc::DoEnqueue(Ptr<QueueDiscItem> item)
{
    NS_LOG_FUNCTION(this << item);

    // find the flow of the item
    FlowInfo& flow = GetFlowInfo(item);
    flow.PacketReceived(item);      // rate sample update

    // find the bin of the flow
    uint32_t binId = flow.m_binId;
    if (m_bins[binId]->GetStatus() == ABBCoDelBin::INACTIVE)
    {
        NS_LOG_INFO("Inactive bin " << binId << " now becomes new bin");
        double binW = m_bins[binId]->GetWeight();
        m_bins[binId]->SetStatus(ABBCoDelBin::NEW_BIN);
        m_bins[binId]->SetDeficit(std::ceil(binW * m_quantum));
        m_newBins.push_back(m_bins[binId]);
    }

    m_bins[binId]->GetQueueDisc()->Enqueue(item);

    NS_LOG_DEBUG("Packet enqueued into bin " << binId << "; flow id " << flow.m_id
                << "; packet size " << item->GetSize());

    if (GetCurrentSize() > GetMaxSize())
    {
        NS_LOG_DEBUG("Overload; enter ABBCodelDrop ()");
        ABBCoDelDrop();
    }

    return true;
}

Ptr<QueueDiscItem>
ABBCoDelQueueDisc::DoDequeue()
{
    NS_LOG_FUNCTION(this);

    Ptr<ABBCoDelBin> bin;
    Ptr<QueueDiscItem> item;

    do
    {
        bool found = false;

        // start with new bins to look for a bin to dequeue a packet from
        while (!found && !m_newBins.empty())
        {
            bin = m_newBins.front();    // accessing the front w/o removing it
            uint32_t binId = bin->GetIndex();

            if (bin->GetDeficit() <= 0)
            {
                double binW = bin->GetWeight();

                NS_LOG_DEBUG("Increase deficit for new-to-be-old bin index " << binId);
                bin->IncreaseDeficit(std::ceil(binW * m_quantum));

                NS_LOG_INFO("Bin " << binId << " now becomes old bin");
                bin->SetStatus(ABBCoDelBin::OLD_BIN);
                m_oldBins.splice(m_oldBins.end(), m_newBins, m_newBins.begin());
            }
            else
            {
                NS_LOG_DEBUG("Found a new bin " << binId << " with positive deficit");
                found = true;
            }
        }

        // if no new bins, look for a bin in old bins to dequeue a packet from
        while (!found && !m_oldBins.empty())
        {
            bin = m_oldBins.front();    // accessing the front w/o removing it
            uint32_t binId = bin->GetIndex();

            if (bin->GetDeficit() <= 0)
            {
                double binW = bin->GetWeight();

                NS_LOG_DEBUG("Increase deficit for old bin index " << binId);
                bin->IncreaseDeficit(std::ceil(binW * m_quantum));
                m_oldBins.splice(m_oldBins.end(), m_oldBins, m_oldBins.begin());
            }
            else
            {
                NS_LOG_DEBUG("Found an old bin " << binId << " with positive deficit");
                found = true;
            }
        }

        if (!found)
        {
            NS_LOG_DEBUG("No bin found to dequeue a packet");
            return nullptr;
        }

        uint32_t binId = bin->GetIndex();
        NS_LOG_INFO("Dequeuing a packet from bin " << binId << " ...");
        item = bin->GetQueueDisc()->Dequeue();

        if (!item)
        {
            NS_LOG_DEBUG("Could not get a packet from the selected bin (queue)");
            if (!m_newBins.empty())     // was a new bin
            {   // make it an old bin to receive further packets for scheduling
                NS_LOG_INFO("Bin " << binId << " now becomes old bin");
                bin->SetStatus(ABBCoDelBin::OLD_BIN);
                m_oldBins.push_back(bin);
                m_newBins.pop_front();
            }
            else // this bin is an old bin without packets: make it inactive
            {
                NS_LOG_INFO("Bin " << binId << " now becomes inactive bin");
                bin->SetStatus(ABBCoDelBin::INACTIVE);
                m_oldBins.pop_front();
            }
        }
        else
        {
            NS_LOG_DEBUG("Dequeued packet " << item->GetPacket());

            FlowInfo& flow = GetFlowInfo(item);
            flow.PacketSent(item);      // rate sample update
        }

    } while (!item);

    bin->IncreaseDeficit(item->GetSize() * -1);

    return item;
}

bool
ABBCoDelQueueDisc::CheckConfig()        // called before InitializeParams
{
    NS_LOG_FUNCTION(this);

    if (GetNQueueDiscClasses() > 0)
    {
        NS_LOG_ERROR("ABBCoDelQueueDisc cannot have classes");
        return false;
    }

    if (GetNInternalQueues() > 0)
    {
        NS_LOG_ERROR("ABBCoDelQueueDisc cannot have internal queues");
        return false;
    }

    // we are at initialization time. If the user has not set a quantum value,
    // set the quantum to the MTU of the device (if any)
    if (!m_quantum)
    {
        Ptr<NetDeviceQueueInterface> ndqi = GetNetDeviceQueueInterface();
        Ptr<NetDevice> dev;
        // if the NetDeviceQueueInterface object is aggregated to a
        // NetDevice, get the MTU of such NetDevice
        if (ndqi && (dev = ndqi->GetObject<NetDevice>()))
        {
            m_quantum = dev->GetMtu();
            NS_LOG_DEBUG("Setting the quantum to the MTU of the device: " << m_quantum);
        }

        if (!m_quantum)
        {
            NS_LOG_ERROR("The quantum parameter cannot be null");
            return false;
        }
    }

    if (!m_nBins)
    {
        NS_LOG_ERROR("The number of bins parameter cannot be null");
        return false;
    }
    if (!m_rcInterval)
    {
        NS_LOG_ERROR("The reclassification interval parameter cannot be null");
        return false;
    }
    if (m_channelBW <= 0.0)
    {
        NS_LOG_ERROR("The channel bandwidth (capacity) parameter cannot be null");
        return false;
    }

    // sanity check
    NS_ABORT_MSG_IF(m_nBins > sizeof(m_bins)/sizeof(m_bins[0]), 
                    "The number of bins parameter exceeds the limit of preallocated bins");

    if (m_useL4s)
    {
        NS_ABORT_MSG_IF(m_ceThreshold == Time::Max(), "CE threshold not set");
        if (m_useEcn == false)
        {
            NS_LOG_WARN("Enabling ECN as L4S mode is enabled");
        }
    }

    // check if flow id/weight callbacks are set
    if (m_flowIdCb.IsNull())
    {
        NS_LOG_ERROR("The flow id callback parameter cannot be null");
        return false;
    }
    if (m_flowWeightCb.IsNull())
    {
        NS_LOG_ERROR("The flow weight callback parameter cannot be null");
        return false;
    }

    return true;
}

void
ABBCoDelQueueDisc::InitializeParams()   // called after CheckConfig
{
    NS_LOG_FUNCTION(this);

    // create bins and CoDel queue disc
    for (uint32_t i=0; i<m_nBins; i++) {
        Ptr<CoDelQueueDisc> codel = CreateObject<CoDelQueueDisc>();

        codel->SetAttribute("MaxSize", QueueSizeValue(GetMaxSize()));
        codel->SetAttribute("Interval", StringValue(m_interval));
        codel->SetAttribute("Target", StringValue(m_target));

        codel->SetAttribute("UseEcn", BooleanValue(m_useEcn));
        codel->SetAttribute("CeThreshold", TimeValue(m_ceThreshold));
        codel->SetAttribute("UseL4s", BooleanValue(m_useL4s));

        codel->Initialize();

        m_bins[i] = CreateObject<ABBCoDelBin>();
        m_bins[i]->SetQueueDisc(codel);
        m_bins[i]->SetIndex(i);
        m_bins[i]->SetBandwidthThreshold(m_channelBW * 1.1);  // by default
        AddQueueDiscClass(m_bins[i]);
    }

    Simulator::Schedule(Seconds(m_rcInterval), &ABBCoDelQueueDisc::Optimize, this);
}

void
ABBCoDelQueueDisc::Optimize()
{
    NS_LOG_FUNCTION(this);

    // update flow rates using current sample
    for (FlowInfo& flow : m_flows)
    {
        flow.UpdateRates();
    }

    // To avoid skewing the threshold calculation, remove flows of:
    //      low rate or inactive
    //      flow id = 0 (currently ARP packets)
    for (FlowInfo& flow : m_flows)
    {
        flow.m_alloced = (flow.m_id == 0 || flow.m_avgServiceRate <= 0.1) ? true : false;
    }

    // tentatively set all bins bandwidth thresholds to channel capacity
    for (uint32_t i = 0; i < m_nBins; i++)
    {
        m_bins[i]->SetBandwidthThreshold(m_channelBW * 1.1);    // 10% safety
    }

    // calculate bin bandwidth
    for (uint32_t i = 0; i < m_nBins-1; i++)    // the last bin is catch-all
    {
        double weightsUnalloced = 0.0;          // sum of weights of unalloced flows
        double bwLeft = m_channelBW;            // bandwidth left for new allocations

        for (FlowInfo& flow : m_flows)
        {
            if (flow.m_alloced)
            {
                bwLeft -= flow.m_avgServiceRate;
            }
            else
            {
                weightsUnalloced += flow.m_weight;
            }
        }

        if (weightsUnalloced == 0)
        {
            NS_LOG_INFO("Done with all flow allocation");
            break;
        }
        
        double nextBwThreshold = bwLeft / weightsUnalloced;
        m_bins[i]->SetBandwidthThreshold(nextBwThreshold);

        bool newFlowAlloced = false;
        for (FlowInfo& flow : m_flows)
        {
            if (!flow.m_alloced && (flow.m_avgServiceRate / flow.m_weight <= nextBwThreshold))
            {
                flow.m_alloced = true;
                newFlowAlloced = true;
            }
        }

        // TODO: if there are no flows for this bin, what should happen??
        NS_ABORT_MSG_IF(!newFlowAlloced, "No flow allocated for bin " << i);
    }

    // calculate bin weights and reclassify flows
    NS_LOG_INFO("Reset all bin weights ...");
    for (uint32_t i = 0; i < m_nBins; i++)
    {
        m_bins[i]->SetWeight(0.0);
    }
    for (FlowInfo& flow : m_flows)
    {
        uint32_t oldBin = flow.m_binId;

        // flow with id = 0 or low rate or inactive goes to first bin always
        if (flow.m_id == 0 || flow.m_avgServiceRate <= 0.1)
        {
            flow.m_binId = 0;
        }
        // unresponsive, high rate flow goes to the last bin
        else if ((flow.m_avgArrivalRate - flow.m_avgServiceRate) / flow.m_avgArrivalRate > 0.10)
        {
            flow.m_binId = m_nBins - 1;
        }
        // reclassify the flows into their respective bins based on consumption
        else
        {
            double normalizedBWConsumed = flow.m_avgServiceRate / flow.m_weight;
            flow.m_binId = m_nBins - 1;     // last bin catches all unallocated
            for (uint32_t i = 0; i < m_nBins - 1; i++)
            {
                if (normalizedBWConsumed <= m_bins[i]->GetBandwidthThreshold())
                {
                    flow.m_binId = i;
                    break;
                }
            }
        }

        // increase corresponding bin weight for this flow
        NS_LOG_INFO("Flow " << flow.m_id << " bin change from " << oldBin << " to " << flow.m_binId);
        m_bins[flow.m_binId]->IncreaseWeight(flow.m_weight);
    }

    Simulator::Schedule(Seconds(m_rcInterval), &ABBCoDelQueueDisc::Optimize, this);
}

ABBCoDelQueueDisc::FlowInfo&
ABBCoDelQueueDisc::GetFlowInfo(Ptr<QueueDiscItem> item)
{
    NS_LOG_FUNCTION(this << item);

    uint32_t flowId = m_flowIdCb(item);
    if (m_flowIndices.find(flowId) == m_flowIndices.end())      // new flow
    {
        double flowWeight = m_flowWeightCb(item);

        NS_LOG_INFO("new flow id = " << flowId << ", weight = " << flowWeight);
        m_flows.push_back(FlowInfo(flowId, flowWeight));    // new flow in first bin (0)
        m_flowIndices[flowId] = m_flows.size() - 1;

        // now increase the weight of the bin (0)
        m_bins[0]->IncreaseWeight(flowWeight);
    }

    FlowInfo& flow = m_flows[m_flowIndices[flowId]];
    NS_ASSERT(flowId == flow.m_id);

    return flow;
}

uint32_t
ABBCoDelQueueDisc::ABBCoDelDrop()
{
    NS_LOG_FUNCTION(this);

    uint32_t maxBacklog = 0;
    uint32_t index = 0;
    Ptr<QueueDisc> qd;

    /* Queue is full! Find the fat bin and drop packet(s) from it */
    for (uint32_t i = 0; i < GetNQueueDiscClasses(); i++)
    {
        qd = GetQueueDiscClass(i)->GetQueueDisc();
        uint32_t bytes = qd->GetNBytes();
        if (bytes > maxBacklog)
        {
            maxBacklog = bytes;
            index = i;
        }
    }

    /* Our goal is to drop half of this fat bin backlog */
    uint32_t len = 0;
    uint32_t count = 0;
    uint32_t threshold = maxBacklog >> 1;
    qd = GetQueueDiscClass(index)->GetQueueDisc();
    Ptr<QueueDiscItem> item;

    do
    {
        NS_LOG_DEBUG("Drop packet (overflow); count: " << count << " len: " << len
                                                       << " threshold: " << threshold);
        item = qd->GetInternalQueue(0)->Dequeue();
        DropAfterDequeue(item, OVERLIMIT_DROP);
        len += item->GetSize();
    } while (++count < m_dropBatchSize && len < threshold);

    return index;
}

} // namespace ns3
