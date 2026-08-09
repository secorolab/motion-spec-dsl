/*
 * This is an auto-generated file. Do not edit it directly.
 *
 * FSM: admittance_arc_single_fsm
 * FSM Description: Home, descend to contact, move forward, trace an arc, and yield to external force through admittance control.
 *
 * -----------------------------------------------------
 * Usage example:
 * -----------------------------------------------------

#include "coord2b/functions/event_loop.h"
#include "coord2b/functions/fsm.h"
#include "admittance_arc_single_fsm.hpp"

struct user_data {

};

void yyyy_behavior(struct user_data *userData, struct events *eventData) {
    // ... do something

    produce_event(eventData, admittance_arc_single_fsm::E_ZZZZ);
}

void fsm_behavior(struct events *eventData, struct user_data *userData) {
    if (consume_event(eventData, admittance_arc_single_fsm::E_XXXX)) {
        yyyy_behavior(userData, eventData);
    }
    ...
}

int main() {

    struct user_data userData = {};
    struct fsm_nbx *fsm = admittance_arc_single_fsm::create_fsm();
    if (!fsm) return 1;

    while (true) {
        produce_event(fsm->eventData, admittance_arc_single_fsm::E_STEP);

        // run state machine, event loop
        fsm_behavior(fsm->eventData, &userData);
        fsm_step_nbx(fsm);
        reconfig_event_buffers(fsm->eventData);
    }

    admittance_arc_single_fsm::destroy_fsm(fsm);
    return 0;
}

 * -----------------------------------------------------
 */

#ifndef ADMITTANCE_ARC_SINGLE_FSM_HPP
#define ADMITTANCE_ARC_SINGLE_FSM_HPP

#include "coord2b/types/fsm.h"
#include "coord2b/types/event_loop.h"
#include <new>


namespace admittance_arc_single_fsm {

struct fsm_nbx * create_fsm();
void destroy_fsm(struct fsm_nbx * fsm);

// sm states
enum e_states {
    S_START = 0,
    S_HOME,
    S_TOUCHDOWN,
    S_FORWARD,
    S_ARC,
    S_ADMITTANCE,
    S_DONE,
    NUM_STATES
};

/// This FSM's own IRI; the tables below name its parts.
static constexpr const char * FSM_URI = "https://secorolab.github.io/models/admittance-arc-single/fsm/admittance_arc_single_fsm";

static constexpr const char * STATE_URIS[NUM_STATES] = {
    "https://secorolab.github.io/models/admittance-arc-single/fsm/S_START",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/S_HOME",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/S_TOUCHDOWN",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/S_FORWARD",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/S_ARC",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/S_ADMITTANCE",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/S_DONE",
};

// sm events
enum e_events {
    E_STEP = 0,
    E_HOME_SETTLED,
    E_CONTACT,
    E_FORWARD_DONE,
    E_FORCE_DETECTED,
    E_FORCE_GONE,
    E_ARC_CONTACT,
    E_TABLE_CONTACT,
    E_ARC_ENTERED,
    E_ADMITTANCE_ENTERED,
    NUM_EVENTS
};

static constexpr const char * EVENT_URIS[NUM_EVENTS] = {
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_STEP",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_HOME_SETTLED",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_CONTACT",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_FORWARD_DONE",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_FORCE_DETECTED",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_FORCE_GONE",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_ARC_CONTACT",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_TABLE_CONTACT",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_ARC_ENTERED",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/E_ADMITTANCE_ENTERED",
};

// sm transitions
enum e_transitions {
    T_START_HOME = 0,
    T_HOME_TOUCHDOWN,
    T_TOUCHDOWN_FORWARD,
    T_FORWARD_ARC,
    T_ARC_ADMITTANCE,
    T_ARC_DONE,
    T_ADMITTANCE_DONE,
    T_ADMITTANCE_ARC,
    NUM_TRANSITIONS
};

static constexpr const char * TRANSITION_URIS[NUM_TRANSITIONS] = {
    "https://secorolab.github.io/models/admittance-arc-single/fsm/T_START_HOME",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/T_HOME_TOUCHDOWN",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/T_TOUCHDOWN_FORWARD",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/T_FORWARD_ARC",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/T_ARC_ADMITTANCE",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/T_ARC_DONE",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/T_ADMITTANCE_DONE",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/T_ADMITTANCE_ARC",
};

// sm reactions
enum e_reactions {
    R_STEP_START = 0,
    R_HOME_SETTLED,
    R_CONTACT,
    R_FORWARD_DONE,
    R_FORCE_DETECTED,
    R_ARC_CONTACT,
    R_TABLE_CONTACT,
    R_FORCE_GONE,
    NUM_REACTIONS
};

static constexpr const char * REACTION_URIS[NUM_REACTIONS] = {
    "https://secorolab.github.io/models/admittance-arc-single/fsm/R_STEP_START",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/R_HOME_SETTLED",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/R_CONTACT",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/R_FORWARD_DONE",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/R_FORCE_DETECTED",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/R_ARC_CONTACT",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/R_TABLE_CONTACT",
    "https://secorolab.github.io/models/admittance-arc-single/fsm/R_FORCE_GONE",
};

inline struct fsm_nbx * create_fsm() {

    struct fsm_nbx * fsm   = new (std::nothrow) fsm_nbx{
        .numReactions      = NUM_REACTIONS,
        .numTransitions    = NUM_TRANSITIONS,
        .numStates         = NUM_STATES,
        .states            = nullptr,
        .startStateIndex   = S_START,
        .endStateIndex     = S_DONE,
        .currentStateIndex = S_START,
        .eventData         = nullptr,
        .reactions         = nullptr,
        .transitions       = nullptr
    };
    if (!fsm) return nullptr;

    // sm states
    struct state * states = new (std::nothrow) state[NUM_STATES]{
        {.name = "S_start"},
        {.name = "S_home"},
        {.name = "S_touchdown"},
        {.name = "S_forward"},
        {.name = "S_arc"},
        {.name = "S_admittance"},
        {.name = "S_done"}
    };

    // sm transition table
    struct transition * transitions = new (std::nothrow) transition[NUM_TRANSITIONS]{
        {
            .startStateIndex = S_START,
            .endStateIndex   = S_HOME,
        },
        {
            .startStateIndex = S_HOME,
            .endStateIndex   = S_TOUCHDOWN,
        },
        {
            .startStateIndex = S_TOUCHDOWN,
            .endStateIndex   = S_FORWARD,
        },
        {
            .startStateIndex = S_FORWARD,
            .endStateIndex   = S_ARC,
        },
        {
            .startStateIndex = S_ARC,
            .endStateIndex   = S_ADMITTANCE,
        },
        {
            .startStateIndex = S_ARC,
            .endStateIndex   = S_DONE,
        },
        {
            .startStateIndex = S_ADMITTANCE,
            .endStateIndex   = S_DONE,
        },
        {
            .startStateIndex = S_ADMITTANCE,
            .endStateIndex   = S_ARC,
        }
    };

    // sm reaction table
    struct event_reaction * reactions = new (std::nothrow) event_reaction[NUM_REACTIONS]{
        {
            .conditionEventIndex = E_STEP,
            .transitionIndex     = T_START_HOME,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_HOME_SETTLED,
            .transitionIndex     = T_HOME_TOUCHDOWN,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_CONTACT,
            .transitionIndex     = T_TOUCHDOWN_FORWARD,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_FORWARD_DONE,
            .transitionIndex     = T_FORWARD_ARC,
            .numFiredEvents      = 1,
            .firedEventIndices   = new unsigned int[1]{
                                     E_ARC_ENTERED
                                   },
        },
        {
            .conditionEventIndex = E_FORCE_DETECTED,
            .transitionIndex     = T_ARC_ADMITTANCE,
            .numFiredEvents      = 1,
            .firedEventIndices   = new unsigned int[1]{
                                     E_ADMITTANCE_ENTERED
                                   },
        },
        {
            .conditionEventIndex = E_ARC_CONTACT,
            .transitionIndex     = T_ARC_DONE,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_TABLE_CONTACT,
            .transitionIndex     = T_ADMITTANCE_DONE,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_FORCE_GONE,
            .transitionIndex     = T_ADMITTANCE_ARC,
            .numFiredEvents      = 1,
            .firedEventIndices   = new unsigned int[1]{
                                     E_ARC_ENTERED
                                   },
        }};

    if (!states || !transitions || !reactions) {
        delete[] states;
        delete[] transitions;
        delete[] reactions;
        delete fsm;
        return nullptr;
    }

    for (unsigned int i = 0; i < NUM_REACTIONS; ++i) {
        if (reactions[i].numFiredEvents > 0 && !reactions[i].firedEventIndices) {
            for (unsigned int j = 0; j < NUM_REACTIONS; ++j) {
                delete[] reactions[j].firedEventIndices;
            }
            delete[] reactions;
            delete[] transitions;
            delete[] states;
            delete fsm;
            return nullptr;
        }
    }

    // sm event data
    struct events * eventData = new (std::nothrow) events{};
    _Bool * currentEvents = new (std::nothrow) _Bool[NUM_EVENTS]{false};
    _Bool * futureEvents = new (std::nothrow) _Bool[NUM_EVENTS]{false};
    if (!eventData || !currentEvents || !futureEvents) {
        delete[] states;
        delete[] transitions;
        if (reactions) {
            for (unsigned int i = 0; i < NUM_REACTIONS; ++i) {
                delete[] reactions[i].firedEventIndices;
            }
        }
        delete[] reactions;
        delete[] currentEvents;
        delete[] futureEvents;
        delete eventData;
        delete fsm;
        return nullptr;
    }
    eventData->numEvents     = NUM_EVENTS;
    eventData->currentEvents = currentEvents;
    eventData->futureEvents  = futureEvents;

    // sm fsm struct
    fsm->states      = states;
    fsm->eventData   = eventData;
    fsm->reactions   = reactions;
    fsm->transitions = transitions;

    return fsm;
}

inline void destroy_fsm(struct fsm_nbx * fsm) {
    if (!fsm) return;
    if (fsm->reactions) {
        for (unsigned int i = 0; i < fsm->numReactions; ++i) {
            delete[] fsm->reactions[i].firedEventIndices;
            fsm->reactions[i].firedEventIndices = nullptr;
            fsm->reactions[i].numFiredEvents = 0;
        }
    }
    if (fsm->eventData) {
        delete[] fsm->eventData->currentEvents;
        delete[] fsm->eventData->futureEvents;
        delete fsm->eventData;
        fsm->eventData = nullptr;
    }
    delete[] fsm->reactions;
    delete[] fsm->transitions;
    delete[] fsm->states;
    delete fsm;
}

} // namespace admittance_arc_single_fsm

#endif // ADMITTANCE_ARC_SINGLE_FSM_HPP