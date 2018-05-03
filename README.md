# ASI-CORE-0

An agent architecture candidate core for Artificial Super Intelligence (ASI).

This library contains a set of structs and traits useful for programming agents.
It is designed for agents that must deal with real time concurrency, delays and failures.
In this design, cognitive capabilities are treated as sensors/actuators.

The library contains just the core abstraction of the agent architecture.
It is intentionally designed to be low level in order to build basic
experience of ASI architecture core designs.
This project does not focus on AGI (Artificial General Intelligence),
but how to control super-human-level artificial intelligence safely.
The basic idea is to design agents that attempts to break/protect system safety intentionally.
These agents can be used later to test the safety of the final ASI design.

For example, by forcing an agent to behave sequentially, some safety properties
are added to the system.
Another example is an agent that tries to allocate as much memory as possible.

The long term goal is to evaluate the practical aspects of reliable neutral judgements,
since this solves a weak version of the control problem in the form of a [Polite Zen Robot](https://github.com/advancedresearch/path_semantics/blob/master/papers-wip/the-polite-zen-robot.pdf).

*Notice! This code is for research purposes only and should NEVER be used in system criticial applications!*

*Notice! Using this design in self-improving ASI without careful supervision could be extremely dangerous!*

### Motivation

The intention of this library is to be used to construct a safe ASI.
It is a *candidate to a core* and not guaranteed to be used in final design.

Some property of this design:

- Very small, such that all parts of it can be easily understood.
- Modular.
- Close to theoretical definition of an agent, making it easier to learn from experiments.
- Expecting concurrency and delays by default, which is a realistic world assumption.
- Not treating cognitive capabilities as "special", they are just sensors and actuators.
- Reducing decision making to a problem of connecting sensors and actuators.
- Time step is deterministic to allow automatic testing of deterministic decision procedures.
- Safe and predictable shut down procedure.

One particular research area this library will be used for,
is safety aspects related to practical engineering of the core of an ASI.
This is about composition of various architectures and their properties.
In order to learn from experience in this field,
one must work with a design intended to work in the real world.

This core reduces the problem of decision making to a small set of fixed actions.
In principle, these actions are the same for agents.
*The decison procedure only connects sensor and actuators, plus managing memory*.
Memory management is low level to let the agent have precise control.

By using a small set of actions that is possible while still making the system
practical in the real world, one can generalize results more easily from one
agent to another.

It is also believed that this design makes it easier to focus on the core problem of ASI:
How to make best use of the information and actions that are available,
instead of treating those as something that comes at close to zero cost.
Since the agent is restricted to connecting input and output,
it must behave in a way such that self-extensions are sensors/actuators it can reason about.
Hopefully this will also lead to some insights about self-improvement.

Reducing the decision procedure also helps checking what the agent does while running.
The runtime can be customized to test or debug agents.

Because of this simple design, it is also possible to wrap decision procedures
within another to force it behave sequentially (see `Sequential` for more information).

### The Decison Maker

The "brain" of an agent is called a decision maker.
The decison maker decides how to use sensors and actuators.

A decision maker has no control over cognitive processes.
Instead, one must design sensors and actuators that work together to
perform cognitive operations.
The decision maker must plan and make the right choices such that these cognitive
processes work properly.
One benefit of this design, if some sensors and actuators allows
expanding the range of sensors and actuators,
then the decision maker can extend the agent's cognitive processes while running.

For each step, the decision maker outputs an action.
This action can terminate the runtime, wait some seconds,
output some memory state to an actuator,
input some memory state from a sensor,
allocate a new memory slot, free a memory slot or swap two memory slots.

### No Reward Signal

There is no explicit reward signal communicated to the agent.
The decision maker must decide itself how to learn over time.
In reinforcement learning, the reward signal can be encoded into the decision maker.

### Shutting Down and Waking Up from Sleep

For safety purposes, termination is controlled externally.
The agent can be given a warning to shut down within some desired time interval.
It is required to report back an estimate of how long it takes to shut down,
and it must start preparing the shut down immediately.
This request of shut down can be called repeatedly to measure progress toward
a safe termination state.

If the controller is impatient, the runtime can be asked to kill the agent.
This will stop execution of the agent immediately even if the agent has not reached a safe state.
(It does not morally kill the agent as it might be resurrected from backup memory).

An agent might choose to put itself to sleep, by waiting for some seconds while doing nothing.
The runtime can be asked to wake up the agent, to give it a task or bring attention to something.

### Limitations

When some input is requested or output is sent, the action can not be cancelled.
The runtime will force through the action even the decision maker regrets it.
The decision maker need to be sure that the action is safe to do,
or that the actuator can be interrupted by sending a concurrent memory state.

The decison maker is not allowed to know the number of sensors or actuators,
neither what type they have.
This information must be explicitly agreed upon or communicated through some protocol.
The decison maker should only depend on the type of memory states,
such that the knowledge the decison maker gets about the environment is solely
through memory.

## License

Licensed under either of
 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you shall be dual licensed as above, without any
additional terms or conditions.
