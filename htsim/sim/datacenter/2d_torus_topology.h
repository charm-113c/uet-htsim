// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef TWO_DIMENSIONAL_TORUS
#define TWO_DIMENSIONAL_TORUS
#include <cmath>
#include <cstdint>
#include <vector>
#include "2d_torus_switch.h"
#include "config.h"
#include "eventlist.h"
#include "logfile.h"
#include "loggers.h"
#include "loggertypes.h"
#include "pipe.h"
#include "route.h"
#include "switch.h"
#include "topology.h"

#ifndef QT
#define QT
typedef enum {
    UNDEFINED,
    RANDOM,
    ECN,
    COMPOSITE,
    PRIORITY,
    CTRL_PRIO,
    FAIR_PRIO,
    LOSSLESS,
    LOSSLESS_INPUT,
    LOSSLESS_INPUT_ECN,
    COMPOSITE_ECN,
    COMPOSITE_ECN_LB,
    SWIFT_SCHEDULER,
    ECN_PRIO,
    AEOLUS,
    AEOLUS_ECN
} queue_type;
#endif  //! QT

/**
 * In a 2D torus topology, we may visualise switches/nodes as being on an n*m grid.
 * Every switch is connected to its four neighbours to the left, right, up and down. Switches at the
 * extremities of the grid must also have four neighbours. So, for instance: a switch at coordinate
 * (x=0, y) for a fixed y will connect with the switch at (x=n, y), thus creating a "wrap-around"
 * link.
 * This class aims to implement one such topology, with switches capable of adaptive routing.
 *
 * Furthermore, this class also aims to allow simulating a theoretical satellite constellation,
 * where the satellites are capable of computing and communicating, effectively forming a
 * space-borne computer cluster; indeed, a 2D torus is a natural topology choice for satellite
 * constellations.
 */
class TwoDimensionalTorusTopology : public Topology {
public:
    EventList* _eventlist;
    Logfile* _logfile;
    QueueLoggerFactory* _q_logger_factory;

    //! n*m 2D vector of switches
    vector<vector<Switch*>> _switches;
    //! 3D vector for Pipe: first 2D correspond to associated switch, 3rd dimension represents the
    //! virtual channels
    vector<vector<vector<Pipe*>>> _northbound_pipes;
    vector<vector<vector<Pipe*>>> _eastbound_pipes;
    vector<vector<vector<Pipe*>>> _southbound_pipes;
    vector<vector<vector<Pipe*>>> _westbound_pipes;
    //! Each pipe is preceded by its own egress queue
    vector<vector<vector<Queue*>>> _north_egress_queues;
    vector<vector<vector<Queue*>>> _east_egress_queues;
    vector<vector<vector<Queue*>>> _south_egress_queues;
    vector<vector<vector<Queue*>>> _west_egress_queues;
    //! To each egress queue we associate the ingress queue opposite to it (although ingress queues
    //! are only used for lossless input queues)
    vector<vector<vector<Queue*>>> _north_ingress_queues;
    vector<vector<vector<Queue*>>> _east_ingress_queues;
    vector<vector<vector<Queue*>>> _south_ingress_queues;
    vector<vector<vector<Queue*>>> _west_ingress_queues;
    // NOTE: The order N, E, S, W is the default used throughout the topology for directions

    //! Minimal constructor to set defaults, used to not bloat full constructor
    TwoDimensionalTorusTopology(Logfile* logfile,
                                QueueLoggerFactory* q_log_factory,
                                uint32_t n,
                                uint32_t m,
                                bool is_constellation);

    TwoDimensionalTorusTopology(Logfile* logfile,
                                QueueLoggerFactory* q_log_factory,
                                uint32_t n,
                                uint32_t m,
                                uint32_t n_virtual_channels,
                                queue_type queue_type,
                                mem_b queue_size,
                                linkspeed_bps linkspeed,
                                torus_routing_strategy routing_strategy,
                                bool is_constellation,
                                simtime_picosec latency,
                                uint32_t altitude,
                                float_t inclination);

    ~TwoDimensionalTorusTopology();  // TODO: check if needed

    virtual vector<const Route*>* get_bidir_paths(uint32_t src, uint32_t dest, bool reverse);
    virtual vector<uint32_t>* get_neighbours(uint32_t src);

    // TODO: simple, check if needed though
    virtual void add_switch_loggers(Logfile& log, simtime_picosec sample_period);

private:
    //! In satellite constellation, N is the number of orbital planes and M is the number of
    //! satellite per plane
    uint32_t _n;
    uint32_t _m;
    uint32_t _n_virtual_channels;

    queue_type _queue_type;
    mem_b _queue_size_b;
    // TODO: divide bandwidth by n_virtual_channels where relevant
    linkspeed_bps _linkspeed_bps;
    torus_routing_strategy _routing_strategy;

    bool _is_constellation;
    //! Latency is only used when *not* simulating a constellation, and thus can be considered
    //! constant
    simtime_picosec _latency;
    //! If simulating a constellation, shell altitude and orbital inclination allows calculating
    //! latency between nodes/satellites
    uint32_t _altitude_in_m;
    float_t _inclination_in_deg;

    void set_n_virtual_channels(uint32_t n_virtual_channels);
    void set_queue_params(queue_type queue_type, mem_b queue_size);
    void set_linkspeed(linkspeed_bps linkspeed);
    void set_routing_strategy(torus_routing_strategy routing_strategy);
    void set_constellation_params(uint32_t altitude, float_t inclination);
    void set_switch_latency(simtime_picosec latency);

    Queue* alloc_queue(QueueLogger* queue_logger);
};

#endif  //! TWO_DIMENSIONAL_TORUS
