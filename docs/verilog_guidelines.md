# Verilog Coding Standard

## Scope

This standard was written with FPGAs in mind so some details may not apply,
or may be missing, in relation to ASICs. Nonetheless, the standard should still
largely be applicable to ASIC design.

## Verilog Language Versions

Use Verilog-2001, specifically its synthesizable subset, as it's well
supported across CAD tools. Anything not synthesizable is for
simulation/verification and outside the scope of this standard.


## Definitions and Inclusions

Use `` `include`` directives only to bring in common function
definitions which, due to a limitation of Verilog-2001, must be defined inside
the body of a module. A common example is an integer logarithm function to
calculate address width. Place the function in a separate file and include it
at the start of the body of modules which need it.

Use `` `define`` directives only for values which remain valid and
constant _everywhere and always_ in the design, don't depend on any
other less-constant values, and can't be passed as module parameters (e.g.:
opcode encodings for ALU functional units). Place these definitions into a
separate file and `` `include`` it at the beginning of files which need
them. Otherwise, construct constants within a module using
`localparam`.

## Basic Types and Values

All variables, modules, constants, etc... must be defined before they are
used in a file, going from top to bottom. If you have a wire connecting the
output of a later module to the input of an earlier module, define the wire
first, before the earliest module using it.

### Defaults

Use `` `default_nettype none`` at the start of each file, before
module definition. This causes any undefined variable to be an error.
Otherwise, accidentally undefined variables become 1-bit wires, which may cause
subtle bugs during synthesis.

For code you cannot control which assumes a default net type (e.g.
vendor-provided models and IP blocks), wrap the code as follows inside the
provided source file:

```
`default_nettype wire

...

`default_nettype none

```

### Initialization and Logic Values

Use only `reg` and `wire` types. All signals are
registers except where a wire is mandatory: module input ports, connecting
module ports together, and inferring tri-state I/O. _(See [Assign Statements](https://fpgacpu.ca/fpga/verilog.html#assign) for details.)_

Only use `0,1,X,Z` values, and avoid X and Z if at all possible.
Do not assign X to registers or wires, as X propagation will make testing
difficult, and isn't supported in 2-state simulators (e.g.: Verilator).
Instead, have a default catch-all or fall-through logic expression which takes
effect when all others don't. The same idea applies to Boolean tests. Z is only
used to define tri-state I/O, and only if the CAD vendor does not provide
tri-state I/O primitives, as the inference is unreliable.

All registers must be initialized (and reset, if necessary) with a value
which does not contain X or Z, either at definition or using an
`initial` block. _Note that some combinations of register reset and_
_initialization may not work well together depending on the FPGA family. See [Resets](https://fpgacpu.ca/fpga/verilog.html#resets) for details._

_Never assign an initial value to wires_, either using an explicit [assign](https://fpgacpu.ca/fpga/verilog.html#assign) statement or an implicit initial assignment (e.g.:
`wire foo = bar;`). Assigning a wire an initial value can cause
problem if you then mistakenly connect that wire to a module port output,
leading to multiple drivers on the wire and surprise X's in your simulation.
This situation easily happens if you convert a `reg` to a
`wire`, because the signal source is now in a module, and you forget
to remove the initial assignment on the `reg` (usually a zero).

### Parameterization

Wherever possible, parameterize the width of your variables rather than
using constants. It's more work initially, but makes modules more general and
avoids tedious and error-prone edits when something inevitably changes. If the
width can be set by the user, then use a module parameter. If it's an internal
module constant, use a localparam. Very, very rarely, use a \`define constant.
Never use a hardcoded literal integer: name it with a localparam instead.

Unfortunately, in Verilog-2001, module parameters and localparams are not
valid width specifiers for literal values, only \`define constants and literal
integers are. This makes initializing registers of parameterized width with
values of the exact same width impossible, and it's unclear if the implicit
width-extension for values like `'b0` or `'d1` work
correctly past 32 or 64 bits width, depending on the CAD tool. As a workaround,
use concatenation and replication to create the value you need of the proper
bit width.

Also note that localparam is not legal in the ports definition of a module
definition. That's only in SystemVerilog. Thus, if you need a constant
parameter for a port width or some other parameter calculation, you have to use
a parameter, and leave a comment that it should not be set at module
instantiation.

```
// This is 32-bits by default
localparam WORD_WIDTH = 72;

// Also legal to define bit width
localparam [WORD_WIDTH-1:0] WORD_CONSTANT = `SOME_DEFINED_CONSTANT;

// Not legal
reg [WORD_WIDTH-1:0] foo = WORD_WIDTH'b0;

// Legal workarounds
localparam WORD_ZERO      = {WORD_WIDTH{1'b0}};
localparam WORD_ONE       = {{WORD_WIDTH-1{1'b0}},1'b1};
localparam WORD_MINUS_ONE = ~WORD_ZERO;

reg [WORD_WIDTH-1:0] foo  = WORD_ZERO;

// Simpler, but unclear if extended with zeros or Xs past 32 or 64 bits
reg [WORD_WIDTH-1:0] foo = 'b0;
reg [WORD_WIDTH-1:0] foo = 'd42;

// Sometimes you have to clip at the point of use
localparam NARROW_ZERO = {NARROW_WIDTH{1'b0}};
reg [NARROW_WIDTH-1:0] bar = NARROW_ZERO;
always @(*) begin
    bar = WORD_CONSTANT [NARROW_WIDTH-1:0];
end

```

### Bit Widths

Always match bit widths of variable assignments. Even with correct implicit
zero/sign-extension, if the source and sink have different width, it will raise
a pointless warning in the CAD tool, obscuring other more important warnings.
And if the extension is incorrect, it will cause subtle bugs. As before,
concatenation and replication can avoid this problem, and then any bit width
mismatch warnings become significant.

```
// Common

localparam SOURCE_WIDTH = 32;
localparam SINK_WIDTH   = 16;
localparam SOURCE_ZERO  = {SOURCE_WIDTH{1'b0}};
localparam SINK_ZERO    = {SINK_WIDTH{1'b0}};

reg [SOURCE_WIDTH-1:0] source = SOURCE_ZERO;
reg [SINK_WIDTH-1:0]   sink   = SINK_ZERO;

// Zero extension

always @(*) begin
    sink = {SOURCE_ZERO, source};
end

// Signed extension

reg source_sign = 1'b0;

// Not a localparam since it's not a constant value
always @(*) begin
    source_sign = source[SOURCE_WIDTH-1];
    sink        = {{SOURCE_WIDTH{source_sign}}, source};
end

```

### Concatenations

Use concatenations to pack/unpack values instead of selecting ranges of
bits, as possible. Concatenating ranges of bits also works. This idiom is very
useful to meaningfully extract fields from raw data words (and to concisely
document the field format), to group related signals into a single wide
pipeline stage register, or to do fixed permutations of bits.

```
localparam A_WIDTH      = 1;
localparam B_WIDTH      = 2;
localparam C_WIDTH      = 3;
localparam PACK_WIDTH   = A_WIDTH + B_WIDTH + C_WIDTH;

reg [A_WIDTH-1:0]       foo;
reg [B_WIDTH-1:0]       bar;
reg [C_WIDTH-1:0]       baz;
reg [PACK_WIDTH-1:0]    all;

// Pack fields into a word
always @(*) begin
    all = {foo,bar,baz};
end

// Unpack the fields
always @(*) begin
    {foo,bar,baz} = all;
end

```

## Module Definition

### Parameters

When defining a module, give all parameters a default value of zero or an
empty string, to both tell the user what kind of value is expected, and also
make module elaboration fail if any of the parameters are not set at module
instantiation.

Use module parameters to define the bit widths of the module ports, as
localparams can only be defined in a module body, which is too late. If a width
is globally constant, or calculated from other module parameters and not
directly provided by the user, then use another module parameter to compute and
hold the value, and place a comment noting that this parameter is not to be
used at module instantiation. This is commonly done when a parameterized number
of items are passed to a module via concatenation, since Verilog-2001 ports are
always present (no conditional instantiation) and simple bit vectors (no
arrays of items).

### Ports

Always define the direction ( `input, output, inout`), type
( `wire` or `reg`), and name of each module port to reduce
boilerplate code, hint at the structure of the module to help the user, and to
avoid some synthesis and simulation problems:

- Defining port direction, type, and name together removes the need to define
  ports multiple times both in the module port section, then as variables in the
  module body. This was how it was done in Verilog-1995 and gets very long in
  large modules and creates useless code you must skip over each time.

- All inputs are necessarily wires. If an output is a wire, then it comes
  from a sub-module instance, which should be the common case in very modular
  design practice. If an output is a reg, then it comes from some local logic,
  which indicates a small, special case is handled here. All inout ports are
  wires, and only used for tri-state I/O at physical pins.

- If the port type is not defined, it is possible for different tools to
  infer different types in some cases, leading to inconsistent synthesis and
  simulation results.


If you have any register output ports, use an `initial` block
immediately after the module port definitions to initialize them to their
startup value, as ports cannot be initialized at definition like register
variables.

```
`default_nettype none

module Foo
#(
    parameter INPUT_WIDTH = 0,
    parameter INPUT_COUNT = 0,

    // Do not set at instantiation (localparam only legal here in SystemVerilog)
    parameter TOTAL_WIDTH = INPUT_WIDTH * INPUT_COUNT,
    parameter CONST_WIDTH = `SOME_GLOBAL_WIDTH
)
(
    input  wire                     clock,
    input  wire [TOTAL_WIDTH-1:0]   this_input,
    output wire [CONST_WIDTH-1:0]   that_output,
    output reg                      another_output,
    ...
);

    initial begin
        another_output = 1'b0;
    end

...

endmodule

```

When a module is moderately complicated and the clearest name of an internal
signal does not match the module port name it connects to, decouple the names
as follows, assuming you already haven't implicitly done so via a final
pipeline register stage.

```
// Port must be a reg type instead of wire, which denotes a special case to the reader,
// and must be initial'ized for proper simulation.
always @(*) begin
    port_name = internal_name;
end

```

Use port name suffixes to denote if they are inputs or outputs only when
there is unavoidable ambiguity (usually for very simple modules with
`data_in` and `data_out` ports or similar). See a [note by\\
@tom\_verbeure](https://twitter.com/tom_verbeure/status/1070001089624211456) on why `_o` and `_i` port name suffixes
can cause problems.

#### Vivado limitation in naming

Although it works in Verilog, and would be a great match for simple modules
with well-defined functionality, don't name any ports "in" or "out". Those
names are reserved words in VHDL, and Vivado's IP Integrator will reject those
names in Verilog source, even if legal. By extension, avoid reserved words in
both VHDL and Verilog for naming ports, variables, etc...

## Procedural Blocks and Assignments

### Assign Statements

Avoid the use of `assign` statements, except where demanded by
Verilog, as their use makes it hard to see the underlying design, and make the
implementation hard to control for size and performance. I have seen assigns
used as global variables all over a module, and all too often, as an amorphous
large number of assigns to describe the logic followed by a single always block
to register all the outputs. _(See also [Initialization and\_\
_Logic Values](https://fpgacpu.ca/fpga/verilog.html#init) for other assignment cases to avoid.)_

There are exactly two cases where you must use assign:

1. When inferring a tri-state I/O pin, since the pin must then be a wire, if
   you are not explicitly instantiating a tri-state I/O buffer (e.g.: IOBUF,
   OBUFT, etc...).


   _Inferring a tri-state I/O like this may fail if the module is not at the_
   _top-level of the design. I have been bitten by this in Vivado, where it created_
   _an AND gate instead of a tri-state buffer. Thus, if possible, implement the I/O_
   _blocks directly using the CAD vendor modules, which will instantiate correctly_
   _regardless of location in the design hierarchy. This is also a reason why_
   _FPGA-specific I/O ports [should be instantiated\_\
   _in their own enclosing Adapter module](https://fpgacpu.ca/fpga/system.html#adapter)._


   ```
   localparam WORD_WIDTH       = 36;
   localparam WORD_TRISTATE    = {WORD_WIDTH{1'bZ}};

   reg  [WORD_WIDTH-1:0]   data_out;
   reg                     output_enable;
   wire [WORD_WIDTH-1:0]   tristate_bus;

   assign tristate_bus = (output_enable == 1'b1) ? data_out : WORD_TRISTATE;

   ```

2. When dealing with dangling wires. This is a rare case which usually happens
   when a module in a `generate` block is conditionally instantiated,
   and you must connect the dangling wires from the rest of the system to
   something when it is not instantiated.


### Always Blocks

For synchronous designs, there are only two events normally needed to
trigger an `always` block: `@(posedge clock)` (clocked)
and `@(*)` (combinational). The only use of `@(negedge
clock)` is for capturing incoming I/O data which is sent out on the
posedge of the I/O clock, so as to sample the data close the center (in the
absence of programmable delay lines on the I/O). The one exception where
posedge and negedge logic may be (carefully!) mixed is in Clock Domain Crossing
(CDC) circuits.

### Blocking and Non-Blocking Assignments

The repeated dogma about Verilog assignments is exactly correct:

- Use only blocking assignments ( `=`) in `always @(*)` (combinational) blocks.

- Use only non-blocking assignments ( `<=`) in `always @(posedge clock)` blocks.


Blocking assignments should only be used in combinational blocks, despite
being legal when used with clocked logic, for two reasons:

1. The CAD tool may consider the destination of each blocking assignment as an
   unused register if the value is later used in the same clocked always block
   (since the value is used before the next clock edge, thus not from the output
   of the inferred register), which then raises a useless warning.

2. The blocking assignment takes effect immediately, rather than at the end
   of the simulation cycle, so depending on the order in which the simulator
   evaluates clocked always blocks, there could be race conditions giving
   different results between simulation runs, or between simulation and synthesis.
   If you want the deep reasons why, read the IEEE Standard 1364-2001, Section 5,
   "Scheduling Semantics". (Thanks go to Claire Wolf ( [@oe1cxw](https://twitter.com/oe1cxw)) for enlightening me.).


Non-blocking assignments should only be used in clocked always blocks, as
simulators may disagree on the interpretation or disallow the usage of
non-blocking assignments in combinational always blocks, yielding inconsistent
simulations across simulators, and maybe with synthesis results also. For
example, see the [COMBDLY\\
warning](https://www.veripool.org/wiki/verilator/Manual-verilator#ERRORS-AND-WARNINGS) explanation in Verilator.

Do not mix blocking and non-blocking assignments within an always block.
It's legal (unless they assign to the same register!), but not necessary and
error-prone.

For further blocking/non-blocking assignment code examples and explanations
(particularly for the Verilog event scheduling model), Clifford E. Cummings'
dramatically-titled ["Nonblocking\\
Assignments in Verilog Synthesis, Coding Styles That Kill!"](http://www.sunburst-design.com/papers/CummingsSNUG2000SJ_NBA_rev1_2.pdf) is a good,
clear read.

#### Design Rules for Assignments and Blocks

Blocking assignments ( `=`) enable a very useful and specific
design idiom: to break-down complex logic into simpler expressions we can then
think about sequentially and re-use as necessary.

**Backward Dependencies:** if you find you have a
backwards dependency between blocking assignments (an earlier line depends on a
later line), then either re-order the assignments, or split one assignment off
into its own `always()` block. Otherwise, the simulation and
synthesis will still be correct, but they may not match anymore. You will see
functionally correct behaviour in your simulations that does not match the
hardware, such as signals not changing when they should when they otherwise
have no effect on the calculations.

Thus, while simple logic can be correctly computed and assigned in a single
line in a clocked always block, it's often clearer to implement more complex
logic in a combinational block using blocking assignments and then register the
results in an immediately following clocked always block using non-blocking
assignments.

Given the above design rules, it's easy to selectively pipeline logic by
having the second always block be clocked or not (and the assignments changed
to match), without altering the logic or the layout of the code. It also gives
an easy way to estimate how much logic will be placed between registers, and
thus get an early grasp on critical paths.

```
// Breaking down nested ternary operators into two simpler lines,
// with unrelated and parallel logic alongside.

always @(*) begin
    part_one        = (cond1 == 1'b1) ? foo : bar;
    part_two        = (cond2 == 1'b0) ? baz : part_one;
    other_result    = wibble1 ^ wibble2;
end

// If we don't want to register these values,
// simply change the block trigger to @(*),
// and the assignments to blocking (=).

always @(posedge clock) begin
    part_two_reg        <= part_two;
    other_result_reg    <= other_result;
    another_result      <= blob1 & blob2;
end

```

#### Generate Blocks and For Loops

A generate block is used to iteratively or conditionally **instantiate**
entire modules or always blocks inside of it. For iteration, the index variable
must be a `genvar`, which should be declared inside the generate
block for clarity and to avoid re-using it elsewhere. _A generate block does_
_not enclose a genvar inside its scope._

If you need to iteratively create assignments, such as replicating the same
logic for multiple ports, then you do not need a generate block, and can use a
for loop **inside** an always block. Doing it this way is also necessary
sometimes if the logic may appear to cause conflicting updates to the same
variables. Synthesis tools expect all such assignments to be inside the same
always block, else the indeterminate scheduling of independent always blocks
prevents synthesis, and would make simulation non-deterministic. The index
variable of the for loop must be an `integer`, declared outside the
for loop and never re-used elsewhere.

**Do not re-use genvar or integer index variables, as this may not be**
**simulatable.** For example, re-using index variables in Icarus led to
strange, and silent, simulation failures where for loops ceased to function. I
do not know if this is an Icarus bug, or a known behaviour of the Verilog
simulation model. Thus, declare each index variable right before the for loop
using it, and never re-use it again in any other for loop.

## Logic Design

### Boolean Expressions

Express Boolean values behaviourally as equality/inequality tests against
the expected value, which clarifies the intent of the code, removes the need to
understand the polarity of each logic signal, and makes the bit width explicit,
which will avoid some bugs and warnings. If you must invert a comparison, be
sure to use the logical negation operator ( `!`), which always
returns 1 bit, rather than the bitwise negation ( `~`) of all bits.

```
// Rather than this
always @(*) begin
    C = A & ~B;
end

// Do this
always @(*) begin
    C = (A == 1'b1) && (B == 1'b0);
end

```

### if/else vs. ternary operators

Unless otherwise necessary, use ternary operators ( `?:`) instead
of `if/else` statements. There are four main reasons:

- If a variable is assigned a value in one clause of an if/else, but not the
  other, a latch may be synthesized. Using a ternary operator makes this error
  impossible.

- Ternary operators are the only way to conditionally assign values to
  localparams and module parameters.

- We can replace nested case/if/else statements with chained lines of ternary
  operators in blocking assignments, which make the code more compact and easier
  to follow. **But see note above regarding backward dependencies in blocking**
  **assignments.**
- In simulation, Verilog if/else statements treat X and Z values as false, and so
  will fail to propagate these values and give unexpected results:



  ```
  // Given the following...
  reg       foo = 1'bX;
  reg [1:0] bar = 2'b00;

  // Here, in simulation, foo takes on 2'b10, not 2'bXX
  always @(*) begin
      if (foo == 1b'1) begin
          bar = 2'b01;
      end
      else begin
          bar = 2'b10;
      end
  end

  // Here, in simulation, foo takes 2'bXX
  always @(*) begin
      bar = (foo == 1'b1) ? 2'b01 : 2'b10;
  end

  ```


On the other hand, if/else is necessary to conditionally instantiate logic
in `generate` blocks, required by some vendor code templates to
infer specific hardware, and unavoidable for some reset code.

#### Chaining Ternary Operators

_Never_ nest ternary operators, where one term of a ternary operator
is itself a ternary operator. That make for unreadable code. Instead, split the
logic into two blocking assignments, with the second ternary operator using the
output of the first as one of its terms. This style gives you useful
intermediate value signals during simulation, extends to an arbitrary number of
expressions which we can easily reason about in sequence, and becomes a useful
programming pattern for FSMs and other complex logic.

```
// Rather than this...
always @(*) begin
    result = (foo == 1'b1) ? ((bar == 1'b1) ? A : B) : C;
end

// ...do this!
always @(*) begin
    partial = (bar == 1'b1) ? A       : B;
    result  = (foo == 1'b1) ? partial : C;
end

```

### Estimating Logic Usage and Speed

When desiging logic, keep track of how many unique inputs are needed to
generate the output, and match that to the target FPGA Look-Up Tables (LUTs).
For example, if the FPGA has 6-input LUTs (6-LUTs), then any logic expression
of up to 6 terms can (and usually will) map to a single 6-LUT per bit of output
width. If you construct your logic as series of expressions of 6 or fewer terms
with registers in between, then you minimize the logic and interconnect delay,
and give the CAD tool more freedom to place and route.

Keeping the number of unique input terms in mind particularly applies to
multiplexers. A 4:1 mux has 6 inputs terms (4 input bits and 2 selector bits)
and so maps exactly to one 6-LUT per result bit, and can be registered "for
free". If you want to maximize speed, be wary of multiplexers wider than 8:1,
and avoid designing logic as a single large selection from many options: better
to pipeline a sequence of smaller selections.

_I'm glossing over fracturable LUTs and logic packing here, but those are_
_things we can usually take for granted from the CAD tool._

### State Machines

Separate your FSM from your data processing, which also enables you to break
a larger FSM into smaller ones. For example, each datapath can have its own FSM
to perform transactions, and these FSMs can be in turn controlled by another FSM
which deals with error-recovery.

In other words, don't place the state and data processing logic together
into a single large `case` statement, with one case per FSM state,
and nested if/else statements inside each case to handle the input/output
control signals. Figuring out all possible state transitions and control
signals, and eliminating redundant and illegal ones, isn't practical past
simple cases, and can lead to lots of seemingly arbitrary code the next reader
has to reverse-engineer back into a state diagram.

Instead, take advantage of the sequential ordering of blocking assignments
and of ternary operators, using the idiom of starting with a register and
sequentially testing and passing along its updated value in a combinational
always block, and registering the updated value in an immediately following
clocked always block. If none of the conditions are met, the register remains
unchanged.

Then, using this idiom, describe the possible states of the datapath
(independent of control), build up definitions of the basic datapath operations
(independent of state) and of the allowed transformations on the datapath (as
operations and the states they may occur in), and then express the state
transition and control signal logic in terms of those datapath transformations.
This approach describes the FSM at a higher level and reduces code duplication,
making it easier to read, debug, and maintain. Expect to need a larger than
usual number of comments in your FSM, as it depends on an external context (the
datapath) to be meaningful.

To see these idioms used to create a small but non-trivial FSM, have a look
at the [Skid Buffer](https://fpgacpu.ca/fpga/Pipeline_Skid_Buffer.html) module.

### Resets

( _Resets are one place where FPGA and ASIC design practices diverge. You_
_are also better off encapsulating all the issues discussed below inside a [Register module](https://fpgacpu.ca/fpga/Register.html)_)

Make use of the implicit power-on-reset in FPGAs, limit the number of things
which need an explicit asynchronous reset signal, and feed reset from a signal
synchronous to the clock. This means passing an external reset through a [CDC Bit Synchronizer](https://fpgacpu.ca/fpga/CDC_Bit_Synchronizer.html) and ensuring a
sufficient reset time.

On FPGAs, the hardware reset of a flip-flop is usually asynchronous and so
takes effect immediately rather than at the next clock edge, which can
cause subtle bugs: a register appears to fail to capture data in
behavioural simulation, or changes in impossible ways (within less than
a clock cycle) in timing-annotated post-synthesis simulation.

If you need to reset during normal operation, use a separate synchronous
"clear" signal controlling the data into the flip-flop. This may create extra
logic, but that logic gets folded into other logic feeding data to the
register, and would have been necessary anyway.

_True asynchronous resets are an exception when a clock isn't yet available_
_(e.g.: PLL reset), or when the synchronous control logic is wedged._

The configuration bitstream of an FPGA includes the initial state of all
registers and (most) on-chip memories, so most logic does not require a reset
signal to properly start operating. See Ken Chapman's [Get\\
Smart About Reset: Think Local, Not Global](https://www.xilinx.com/support/documentation/white_papers/wp272.pdf) for details. Set the initial
state of registers by assigning it at declaration, or assigning it inside an
`initial` block for module register output ports. Most on-chip
memories can be similarly initialized with a known content via a
`$readmemh()` directive, or some code in an `initial`
block. Refer to the [Single Port RAM](https://fpgacpu.ca/fpga/RAM_Single_Port.html) code
for an example of on-chip memory initialization.

#### Resets on Different FPGA Families

The following was contributed by [@ravenslofty](https://twitter.com/ravenslofty) (edited by me):

> Perhaps a better point against reg initialisation at declaration is its
> inconsistency across chips - not all FPGAs support all combinations of init and
> reset. FPGA flops can very roughly be split into four categories:
>
> - not initialisable (QuickLogic EOS S3)
>
> - constant zero (Intel Cyclone V; Lattice iCE40)
>
> - fully configurable (Xilinx 7 Series)
>
> - init-matches-set (Lattice ECP5)
>
>
> This means you can write a register init declaration which, when combined with
> an asynchronous set/reset wire, is impossible to directly satisfy in hardware.
> However, synchronous sets/resets can be emulated via input logic, so a global
> synchronous reset can always be implemented.

Fortunately, asynchronous resets to registers are rare, and not a good thing in
general (they inhibits register retiming, which enables simpler, faster
designs). So register initialization still works, except where you need an
explicit asynchronous reset.

#### The "Last Assignment Wins" Reset Idiom

The common idiom for resets uses an if/else statement, where each
register must be assigned a value in both clauses (else a latch may be
synthesized), which means all registers must be reset, maximizing the size of
the reset tree, which uses more routing and makes timing harder to meet.

```
// Common idiom, which has a problem...
always @(posedge clock) begin
    if (reset == 1'b1) begin
        foo <= FOO_RESET;
        bar <= BAR_RESET; // but does not need a reset!
    end
    else begin
        foo <= foo_next;
        bar <= bar_next;
    end
end

```

If you must reset some registers, use one of the following idioms instead to
minimize the size of the reset tree:

```
// Ternary operator assignment for reset
always @(posedge clock) begin
    foo <= (reset == 1'b1) ? FOO_RESET : foo_next;
    bar <= bar_next
end

// Using "last assignment wins" semantics for reset
always @(posedge clock) begin
    foo <= foo_next;
    bar <= bar_next;

    if (reset == 1'b1) begin
        foo <= FOO_RESET;
    end
end

```

Credit for the "last assignment wins" idiom goes to Olof Kindgren ( [@olofkindgren](https://twitter.com/OlofKindgren)): [Resetting\\
reset handling](https://olofkindgren.blogspot.com/2017/11/resetting-reset-handling.html). This idiom also [applies\\
to VHDL](https://stebanoid.blogspot.com/2018/06/minimalistic-resets-coding-style.html).

Alse see [this twitter\\
thread](https://twitter.com/oe1cxw/status/1040917788762812416) which discusses a subtlety of "last assignment wins" which explains
a subtlety of non-blocking assignments and ternary operators. Credit to
Claire Wolf ( [@oe1cxw](https://twitter.com/oe1cxw)).

#### Combining Asynchronous Reset and Clock Enable

On FPGAs, the flip-flop reset hardware is asynchronous, and this causes
difficulties when inferring flip-flops with clock\_enables from Verilog code.
The "last assignment wins" idiom to implement reset doesn't work here: having
two separate if-statements (one for clock\_enable followed by one for reset)
does not work when the reset is asynchronously specified alongside the clock in
the sensitivity list (as shown below), as there is no way to determine which
signal in the sensitivity list each if statement should respond to.

Thus, correct flip-flop hardware inference depends on explicitly expressing
the priority of the reset over the clock\_enable structurally with nested
if-statements, rather than implicitly through the Verilog event queue via the
"last assignment wins" idiom.

This is very likely the \*only\* place you will ever need an asynchronous
signal in a sensitivity list, or express explicit structural priority.

Also, even though the flip-flop reset hardware is asynchronous, it should be
fed by a synchronous reset signal, otherwise if the flip-flop was not already
at zero, it may flip to zero at a time such that the signal change reaches the
next flip-flops down the line within their metastability window.

```
`default_nettype none

module Register
#(
    parameter WORD_WIDTH  = 0,
    parameter RESET_VALUE = 0
)
(
    input   wire                        clock,
    input   wire                        clock_enable,
    input   wire                        areset,
    input   wire                        clear,
    input   wire    [WORD_WIDTH-1:0]    data_in,
    output  reg     [WORD_WIDTH-1:0]    data_out
);

    initial begin
        data_out = RESET_VALUE;
    end

    reg [WORD_WIDTH-1:0] selected = RESET_VALUE;

    always @(*) begin
        selected = (clear == 1'b1) ? RESET_VALUE : data_in;
    end

    always @(posedge clock or posedge areset) begin
        if (areset == 1'b1) begin
            data_out <= RESET_VALUE;
        end
        else begin
            if ((clock_enable == 1'b1) || (clear == 1'b1)) begin
                data_out <= selected;
            end
        end
    end

endmodule

```

#### Reset Sequencing

Various parts of a design may have to stay in reset for a minimum amount of
time to properly initialize (e.g.: 200us for DDR2 RAM), and have to come out of
reset in a certain order to avoid receiving undefined signals. You can sequence
the resets by creating delayed copies of the power-on-reset with a few counters
driven by the main clock, with the count values set as top-level design module
parameters.

## Some Corner Cases

### Simulated Clock Generation

In simulation, a race condition can exist at time zero between the initial
value assignment of a register and the first clock edge. For example:

```
reg clock = 1'b0; // Counts as a negedge at time zero! (X -> 0)
reg foo   = 1'b0; // Also does X -> 0 at time zero.

// Simulate the clock
always begin
    #`HALF_PERIOD clock = ~clock;
end

// It is unclear if the clock edge or the "foo" initialization will happen first,
// so "bar" can get X for one simulation cycle...
always @(negedge clock) begin
    bar <= foo;
end

```

This race condition is another reason to only use `@(posedge
clock)` in internal logic, but the same race condition will happen if the
simulation clock happens to be initialized to 1'b1.

Instead, the following clock generation idiom (credit: Claire Wolf ( [@oe1cxw](https://twitter.com/oe1cxw))) avoids the race condition by
making use of undefined values and the identity operator ( `===`)
instead of the equality ( `==`) operator:

```
// NOTE: clock is left uninitialized, and thus X in most simulators, and will
// not trigger a (X -> 0) edge until after the simulated clock half-period delay.
always begin
    #`HALF_PERIOD clock = (clock === 1'b0);
end

```

The [Simulation Clock](https://fpgacpu.ca/fpga/Simulation_Clock.html) module implements
this idiom.

### Constant Values

When a combinational procedural block contains only constant values on the
right hand side, your linter and/or simulator may raise warnings. For example:

```
reg foo;
always @(*) begin
    foo = 1'b0;
end

```

Since there can be no clock edge event or value change event of a right hand
side value to trigger the procedural block, it is possible that the assignment
never happens, depending on the interpretation of the CAD tool. This example is
contrived, but such a situation will occur if you have a module output port
which is both a register and outputs a constant. Since you cannot initialize a
register in a port definition, you must instead set the constant value in an
`initial` block, or at an extreme, change the `reg` port
to a `wire` and `assign` it a constant value. (But this
is strongly discouraged.) _(This corner case was pointed out (and fixed!) by_
_[Rodrigo Melo](https://twitter.com/rodrigomelo9ok).)_

### Avoiding Zero-Width Padding

When splitting a long bit vector into a number of words of a given width, or
concatenating multiple words into a single wider vector, it's possible that you
will need to pad a few unused bits, usually with zeros. Typically, you compute
the width difference or the division remainder to get the pad width, then
create the pad with a replication:

```
localparam PAD_WIDTH   = BIG_VECTOR_WIDTH % WORD_WIDTH;
localparam TOTAL_WIDTH = WORD_WIDTH * WORD_COUNT;        // Alternate
localparam PAD_WIDTH   = BIG_VECTOR_WIDTH - TOTAL_WIDTH; // Alternate
localparam PAD         = {PAD_WIDTH{1'b0}};

```

However, this fails if no pad is needed. Variables of zero width cannot
exist in Verilog, and a zero width replication is also illegal in Verilog-2001.
Zero width concatenations are partially fixed in Verilog-2005, and thus
SystemVerilog, but the above `PAD` would still fail at linting and
elaboration since there are no other sized elements being concatenated.

We can eliminate zero from the possible `PAD_WIDTH` values by
instead returning `WORD_WIDTH` if the pad width would have been
zero. This works because a pad is necessarily between 0 and
`WORD_WIDTH-1` bits wide. We can then check later and instantiate a
concatenation with or without a pad:

```
reg [BIG_VECTOR_WIDTH-1:0] baz;
reg [WORD_WIDTH-1:0]       wibble, ...;

generate
    if (PAD_WIDTH != WORD_WIDTH) begin
        always @(*) begin
            baz = {PAD, wibble, ....}; // Padding MSBs here
        end
    end
    else begin
        always @(*) begin
            baz = {wibble, ... };
        end
    end
endgenerate

```

The [Word Pad](https://fpgacpu.ca/fpga/word_pad_function.html) function implements this
idiom, returning the pad width or \`WORD\_WIDTH\` if the pad width would be zero.

#### Simple Cases for Zero-Padding

While the [Word Pad](https://fpgacpu.ca/fpga/word_pad_function.html) function enables
padding of arbitrary length at arbitrary positions (e.g.: you could pad in the
middle of a wide vector with a special pattern instead of only zeros), the
common case is to pad with zeros at the least-significant bits.

We can zero-pad the least significant bits of a wider vector automatically
by making sure the vector is initialized to zero, then using a vector part
select to place the concatenated words into the wider vector at just after the
location of the pad. This approach automatically handles the case where the pad
width equals zero without defining impossible zero-width variables or
replications.

```
localparam BIG_VECTOR_ZERO = {BIG_VECTOR_WIDTH{1'b0}};
localparam TOTAL_WIDTH     = WORD_WIDTH * WORD_COUNT;
localparam PAD_WIDTH       = BIG_VECTOR_WIDTH - TOTAL_WIDTH;

reg [BIG_VECTOR_WIDTH-1:0] baz = BIG_VECTOR_ZERO;
reg [WORD_WIDTH-1:0]       wibble, ...;

always @(*) begin
    baz [PAD_WIDTH +: TOTAL_WIDTH] = {wibble, ... };
end

```

Alternately, you could first concatenate the individual words into a vector
of `TOTAL_WIDTH` bits, then pass it through a [Width Adjuster](https://fpgacpu.ca/fpga/Width_Adjuster.html) to widen it to
`BIG_VECTOR_WIDTH`, then pass that wider vector through a [Bit Shifter](https://fpgacpu.ca/fpga/Bit_Shifter.html) to shift it left by
`PAD_WIDTH`, filling in the least-significant bits with any pattern
you wish.

* * *

[fpgacpu.ca](https://fpgacpu.ca/)