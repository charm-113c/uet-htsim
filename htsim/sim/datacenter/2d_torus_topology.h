// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef TWO_DIMENSIONAL_TORUS
#define TWO_DIMENSIONAL_TORUS
#include <cstdint>
#include <memory>
#include <ostream>
#include <vector>
#include "config.h"
#include "eventlist.h"
#include "firstfit.h"
#include "logfile.h"
#include "loggers.h"
#include "main.h"
#include "network.h"
#include "pipe.h"
#include "randomqueue.h"
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
typedef enum { UPLINK, DOWNLINK } link_direction;
#endif

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

    //! 2D vector of switches
    vector<vector<Switch*>> _switches;
    //! Separation between horizontal and vertical Pipes needed to simulate inter-satellite links
    //! 3D vector for Pipe: first 2D correspond to associated switch, 3rd dimension represents the
    //! virtual channels
    vector<vector<vector<Pipe*>>> _horizontal_pipes;
    vector<vector<vector<Pipe*>>> _vertical_pipes;
    // Note: in a 2D torus switches have 4 neighbours, so we associate an h_pipe and a v_pipe to
    // each so that every switch is connected to 4 others

    //! Queues themselves don't need the separation, merely keeping convention for clarity and
    //! consistency
    vector<vector<vector<Queue*>>> _horizontal_queues;
    vector<vector<vector<Queue*>>> _vertical_queues;

    TwoDimensionalTorusTopology(EventList* eventlist,
                                Logfile* logfile,
                                uint32_t n,
                                uint32_t m,
                                queue_type queue_type,
                                mem_b queue_size,
                                linkspeed_bps linkspeed,
                                bool is_constellation,
                                simtime_picosec latency,
                                uint32_t altitude);
    ~TwoDimensionalTorusTopology();  // TODO: check if needed

    virtual vector<const Route*>* get_bidir_paths(uint32_t src, uint32_t dest, bool reverse);
    virtual vector<uint32_t>* get_neighbours(uint32_t src);

    // TODO: simple, check if needed though
    virtual void add_switch_loggers(Logfile& log, simtime_picosec sample_period);

private:
    //! In satellite constellation, N is the number of orbital planes and M is the number of
    //! satellite per plane
    uint32_t _N;
    uint32_t _M;

    queue_type _queue_type;
    mem_b _queue_size;
    linkspeed_bps _linkspeed_bps;

    bool _is_constellation;
    //! Latency is only used when *not* simulating a constellation, and thus can be considered
    //! constant
    simtime_picosec _latency;
    //! If simulating a constellation, shell altitude allows calculating latency between
    //! nodes/satellites
    uint32_t _altitude_in_m;
};

#endif
